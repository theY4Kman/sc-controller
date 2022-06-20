#!/usr/bin/env python2
"""
SC Controller - DualSense driver

Extends HID driver with DS5-specific options.
"""

import ctypes
import logging
import sys
from datetime import datetime

from scc.constants import ControllerFlags, HapticPos, SCButtons, STICK_PAD_MAX, STICK_PAD_MIN
from scc.drivers.evdevdrv import (
    EvdevController,
    HAVE_EVDEV,
    get_axes,
    get_evdev_devices_from_syspath,
    make_new_device,
)
from scc.drivers.hiddrv import (
    AxisData,
    AxisDataUnion,
    AxisMode,
    AxisModeData,
    AxisType,
    BUTTON_COUNT,
    ButtonData,
    HIDController,
    HIDDecoder,
    HatswitchModeData,
    _lib,
    hiddrv_test,
)
from scc.drivers.usb import register_hotplug_device
from scc.lib import IntEnum, usb1
from scc.tools import init_logging, set_logging_level

log = logging.getLogger("DS5")

VENDOR_ID = 0x054c
PRODUCT_ID = 0x0ce6


class OperatingMode(IntEnum):
    DS4_COMPATIBILITY_MODE = 1 << 0
    DS5_MODE = 1 << 1


class PhysicalEffectControl(IntEnum):
    ENABLE_HAPTICS = 1 << 0 | 1 << 1
    TRIGGER_EFFECTS_RIGHT = 1 << 2
    TRIGGER_EFFECTS_LEFT = 1 << 3


class LightEffectControl(IntEnum):
    MIC_MUTE_LED_CONTROL_ENABLE = 1 << 0
    POWER_SAVE_CONTROL_ENABLE = 1 << 1
    LIGHTBAR_CONTROL_ENABLE = 1 << 2
    RELEASE_LEDS = 1 << 3
    PLAYER_INDICATOR_CONTROL_ENABLE = 1 << 4


class DualSenseHIDOutput(ctypes.Structure):
    _fields_ = [
        ('operating_mode', ctypes.c_ubyte),
        ('physical_effect_control', ctypes.c_ubyte),
        ('light_effect_control', ctypes.c_ubyte),

        ('motor_right', ctypes.c_ubyte),
        ('motor_left', ctypes.c_ubyte),

        ('unknown2', ctypes.c_ubyte * 4),
        ('mute_button_led', ctypes.c_ubyte),
        ('power_save_control', ctypes.c_ubyte),
        ('right_trigger_effect', ctypes.c_ubyte * 11),
        ('left_trigger_effect', ctypes.c_ubyte * 11),

        ('unknown3', ctypes.c_ubyte * 8),

        ('lightbar_control', ctypes.c_ubyte),
        ('lightbar_setup', ctypes.c_ubyte),
        ('led_brightness', ctypes.c_ubyte),

        ('player_leds', ctypes.c_ubyte),
        ('lightbar_red', ctypes.c_ubyte),
        ('lightbar_green', ctypes.c_ubyte),
        ('lightbar_blue', ctypes.c_ubyte),
    ]


ICON_COLORS = [
    (0.0, 1.0, 0.0),  # 0
    (0.0, 0.0, 1.0),  # 1
    (1.0, 0.0, 0.0),  # 2
    (1.0, 1.0, 0.0),  # 3
    (0.0, 1.0, 1.0),  # 4
    (1.0, 0.4, 0.0),  # 5
    (1.0, 0.0, 1.0),  # 6
]


class DS5Controller(HIDController):
    # Most of axes are the same
    BUTTON_MAP = (
        SCButtons.X,
        SCButtons.A,
        SCButtons.B,
        SCButtons.Y,
        SCButtons.LB,
        SCButtons.RB,
        1 << 64,
        1 << 64,
        SCButtons.BACK,
        SCButtons.START,
        SCButtons.STICKPRESS,
        SCButtons.RPAD,
        SCButtons.C,
        SCButtons.CPADPRESS,
    )

    flags = (ControllerFlags.EUREL_GYROS
             | ControllerFlags.HAS_RSTICK
             | ControllerFlags.HAS_CPAD
             | ControllerFlags.HAS_DPAD
             | ControllerFlags.SEPARATE_STICK
             | ControllerFlags.NO_GRIPS
             )

    def __init__(self, device, daemon, handle, config_file, config, test_mode=False):
        self._outputs = {}
        self._feedback_output = DualSenseHIDOutput(
            operating_mode=OperatingMode.DS5_MODE,
            physical_effect_control=PhysicalEffectControl.ENABLE_HAPTICS,
            motor_left=0,
            motor_right=0,
        )
        self._feedback_cancel_task = None
        super(DS5Controller, self).__init__(device, daemon, handle, config_file, config, test_mode)

    def _load_hid_descriptor(self, config, max_size, vid, pid, test_mode):
        # Overrided and hardcoded
        self._decoder = HIDDecoder()

        # Dpad works on DualSense!
        self._decoder.axes[AxisType.AXIS_LPAD_X] = AxisData(
            mode=AxisMode.HATSWITCH, byte_offset=8, size=8,
            data=AxisDataUnion(
                hatswitch=HatswitchModeData(
                    button=SCButtons.LPAD | SCButtons.LPADTOUCH,
                    min=STICK_PAD_MIN, max=STICK_PAD_MAX
                )
            )
        )

        # Sticks are the same as DS4
        self._decoder.axes[AxisType.AXIS_STICK_X] = AxisData(
            mode=AxisMode.AXIS, byte_offset=1, size=8,
            data=AxisDataUnion(
                axis=AxisModeData(
                    scale=1.0, offset=-127.5, clamp_max=257, deadzone=10
                )
            )
        )
        self._decoder.axes[AxisType.AXIS_STICK_Y] = AxisData(
            mode=AxisMode.AXIS, byte_offset=2, size=8,
            data=AxisDataUnion(
                axis=AxisModeData(
                    scale=-1.0, offset=127.5, clamp_max=257, deadzone=10
                )
            )
        )
        self._decoder.axes[AxisType.AXIS_RPAD_X] = AxisData(
            mode=AxisMode.AXIS, byte_offset=3, size=8,
            data=AxisDataUnion(
                axis=AxisModeData(
                    button=SCButtons.RPADTOUCH,
                    scale=1.0, offset=-127.5, clamp_max=257, deadzone=10
                )
            )
        )
        self._decoder.axes[AxisType.AXIS_RPAD_Y] = AxisData(
            mode=AxisMode.AXIS, byte_offset=4, size=8,
            data=AxisDataUnion(
                axis=AxisModeData(
                    button=SCButtons.RPADTOUCH,
                    scale=-1.0, offset=127.5, clamp_max=257, deadzone=10
                )
            )
        )

        # Triggers
        self._decoder.axes[AxisType.AXIS_LTRIG] = AxisData(
            mode=AxisMode.AXIS, byte_offset=5, size=8,  # Not sure about the size
            data=AxisDataUnion(
                axis=AxisModeData(
                    scale=1.0, clamp_max=1, deadzone=10
                )
            )
        )
        self._decoder.axes[AxisType.AXIS_RTRIG] = AxisData(
            mode=AxisMode.AXIS, byte_offset=6, size=8,  # Not sure about the size
            data=AxisDataUnion(
                axis=AxisModeData(
                    scale=1.0, clamp_max=1, deadzone=10
                )
            )
        )

        # Gyro
        # Leaving the AxisMode naming to match DS4
        self._decoder.axes[AxisType.AXIS_GPITCH] = AxisData(
            mode=AxisMode.DS4ACCEL, byte_offset=16
        )  # Pitch found
        self._decoder.axes[AxisType.AXIS_GROLL] = AxisData(
            mode=AxisMode.DS4ACCEL, byte_offset=20
        )  # Roll
        self._decoder.axes[AxisType.AXIS_GYAW] = AxisData(
            mode=AxisMode.DS4ACCEL, byte_offset=18
        )  # Yaw found
        self._decoder.axes[AxisType.AXIS_Q1] = AxisData(
            mode=AxisMode.DS4GYRO, byte_offset=26
        )
        self._decoder.axes[AxisType.AXIS_Q2] = AxisData(
            mode=AxisMode.DS4GYRO, byte_offset=22
        )
        self._decoder.axes[AxisType.AXIS_Q3] = AxisData(
            mode=AxisMode.DS4GYRO, byte_offset=24
        )

        # Touchpad
        self._decoder.axes[AxisType.AXIS_CPAD_X] = AxisData(
            mode=AxisMode.DS4TOUCHPAD, byte_offset=34
        )  # DualSense X
        self._decoder.axes[AxisType.AXIS_CPAD_Y] = AxisData(
            mode=AxisMode.DS4TOUCHPAD, byte_offset=35, bit_offset=4
        )  # DualSense Y

        # Button maps seem to work for standard arrangement (matching Xbox360)
        # Not enough information about the button event triggered when LT && RT are pressed?
        # Could be connected to adaptive triggers?
        self._decoder.buttons = ButtonData(
            enabled=True, byte_offset=8, bit_offset=4, size=14,  # Not sure about bit offset
            button_count=14
        )

        if test_mode:
            for x in xrange(BUTTON_COUNT):
                self._decoder.buttons.button_map[x] = x
        else:
            for x in xrange(BUTTON_COUNT):
                self._decoder.buttons.button_map[x] = 64
            for x, sc in enumerate(DS5Controller.BUTTON_MAP):
                self._decoder.buttons.button_map[x] = self.button_to_bit(sc)

        self._packet_size = 64

    def input(self, endpoint, data):
        # Special override for CPAD touch button
        if _lib.decode(ctypes.byref(self._decoder), data):
            if self.mapper:
                if ord(data[33]) >> 7:
                    # cpad is not touched
                    self._decoder.state.buttons &= ~SCButtons.CPADTOUCH
                else:
                    self._decoder.state.buttons |= SCButtons.CPADTOUCH
                self.mapper.input(
                    self,
                    self._decoder.old_state, self._decoder.state
                )

    def feedback(self, data):
        position, amplitude, period, count = data.data

        normalized_amp = float(amplitude) / 0x8000
        clamped_amp = int(normalized_amp * 0xff)
        half_amp = int(normalized_amp * 0x80)

        if position == HapticPos.LEFT:
            # NOTE: the left motor is heavier, so we must give it less oomph
            self._feedback_output.motor_left = half_amp
        elif position == HapticPos.RIGHT:
            self._feedback_output.motor_right = clamped_amp
        elif position == HapticPos.BOTH:
            self._feedback_output.motor_right = clamped_amp
            self._feedback_output.motor_left = half_amp

        duration = float(period) * count / 0x10000
        # The motors don't seem to perform reliably when shut off under 20ms
        duration = max(duration, 0.02)

        self.schedule_output('feedback', self._feedback_output)

        def clear_feedback(mapper):
            self._feedback_output.motor_right = self._feedback_output.motor_left = 0
            self.schedule_output('feedback', self._feedback_output)

        if self._feedback_cancel_task:
            self._feedback_cancel_task.cancel()
        self._feedback_cancel_task = self.mapper.schedule(duration, clear_feedback)

    def apply_config(self, config):
        icon = config['icon']
        led_level = config['led_level']
        self.configure(icon=icon, led_level=led_level)

    def configure(self, icon=None, led_level=100):
        lightbar_color = (0.0, 0.0, 1.0)  # blue by default
        if icon:
            basename, ext = icon.rsplit('.', 1)
            parts = basename.rsplit('-', 1)
            if parts:
                raw_idx = parts[-1]
                try:
                    icon_idx = int(raw_idx)
                except ValueError:
                    pass
                else:
                    if icon_idx < len(ICON_COLORS):
                        lightbar_color = ICON_COLORS[icon_idx]

        led_level_norm = float(led_level) / 100
        lightbar_color_bytes = tuple(
            int(color_norm * led_level_norm * 255)
            for color_norm in lightbar_color
        )

        output = DualSenseHIDOutput(
            operating_mode=OperatingMode.DS5_MODE,
            light_effect_control=LightEffectControl.LIGHTBAR_CONTROL_ENABLE,
            lightbar_red=lightbar_color_bytes[0],
            lightbar_green=lightbar_color_bytes[1],
            lightbar_blue=lightbar_color_bytes[2],
        )
        self.schedule_output('lightbar', output)

    def get_gyro_enabled(self):
        # Cannot be actually turned off, so it's always active
        # TODO: Maybe emulate turning off?
        return True

    def get_type(self):
        return "ds5"

    def get_gui_config_file(self):
        return "ds5-config.json"

    def __repr__(self):
        return "<DS5Controller %s>" % (self.get_id(),)

    def _generate_id(self):
        """
        ID is generated as 'ds5' or 'ds5:X' where 'X' starts as 1 and increases
        as controllers with same ids are connected.
        """
        magic_number = 1
        id = "ds5"
        while id in self.daemon.get_active_ids():
            id = "ds5:%s" % (magic_number,)
            magic_number += 1
        return id

    def schedule_output(self, output_id, output):
        self._outputs[output_id] = output

    def flush(self):
        super(DS5Controller, self).flush()

        while self._outputs:
            output_id, output = self._outputs.popitem()
            data = bytes(bytearray(output).ljust(64, b'\x00'))
            self.handle.interruptWrite(3, data)


class DS5EvdevController(EvdevController):
    TOUCH_FACTOR_X = STICK_PAD_MAX / 940.0
    TOUCH_FACTOR_Y = STICK_PAD_MAX / 470.0
    BUTTON_MAP = {
        304: "A",
        305: "B",
        307: "Y",
        308: "X",
        310: "LB",
        311: "RB",
        # TODO: Figure out what it is the purpose of the button event when using the trigger
        # 312: "LT2",
        # 313: "RT2",
        314: "BACK",
        315: "START",
        316: "C",
        317: "STICKPRESS",
        318: "RPAD"
        # 319: "CPAD",
    }
    AXIS_MAP = {
        0: {"axis": "stick_x", "deadzone": 4, "max": 255, "min": 0},
        1: {"axis": "stick_y", "deadzone": 4, "max": 0, "min": 255},
        3: {"axis": "rpad_x", "deadzone": 4, "max": 255, "min": 0},
        4: {"axis": "rpad_y", "deadzone": 8, "max": 0, "min": 255},
        2: {"axis": "ltrig", "max": 255, "min": 0},
        5: {"axis": "rtrig", "max": 255, "min": 0},
        16: {"axis": "lpad_x", "deadzone": 0, "max": 1, "min": -1},
        17: {"axis": "lpad_y", "deadzone": 0, "max": -1, "min": 1}
    }
    # TODO: Should the old button for DS4 map be removed? DualSense support came with kernel 5.12
    # BUTTON_MAP_OLD = {
    # 	304: "X",
    # 	305: "A",
    # 	306: "B",
    # 	307: "Y",
    # 	308: "LB",
    # 	309: "RB",
    # 	312: "BACK",
    # 	313: "START",
    # 	314: "STICKPRESS",
    # 	315: "RPAD",
    # 	316: "C",
    # 	# 317: "CPAD",
    # }
    # AXIS_MAP_OLD = {
    # 	0:  { "axis": "stick_x", "deadzone": 4, "max": 255, "min": 0 },
    # 	1:  { "axis": "stick_y", "deadzone": 4, "max": 0, "min": 255 },
    # 	2:  { "axis": "rpad_x", "deadzone": 4, "max": 255, "min": 0 },
    # 	5:  { "axis": "rpad_y", "deadzone": 8, "max": 0, "min": 255 },
    # 	3:  { "axis": "ltrig", "max": 32767, "min": -32767 },
    # 	4:  { "axis": "rtrig", "max": 32767, "min": -32767 },
    # 	16: { "axis": "lpad_x", "deadzone": 0, "max": 1, "min": -1 },
    # 	17: { "axis": "lpad_y", "deadzone": 0, "max": -1, "min": 1 }
    # }
    GYRO_MAP = {
        EvdevController.ECODES.ABS_RX: ('gpitch', 0.01),
        EvdevController.ECODES.ABS_RY: ('gyaw', 0.01),
        EvdevController.ECODES.ABS_RZ: ('groll', 0.01),
        EvdevController.ECODES.ABS_X: (None, 1),  # 'q2'
        EvdevController.ECODES.ABS_Y: (None, 1),  # 'q3'
        EvdevController.ECODES.ABS_Z: (None, -1),  # 'q1'
    }
    flags = (ControllerFlags.EUREL_GYROS
             | ControllerFlags.HAS_RSTICK
             | ControllerFlags.HAS_CPAD
             | ControllerFlags.HAS_DPAD
             | ControllerFlags.SEPARATE_STICK
             | ControllerFlags.NO_GRIPS
             )

    def __init__(self, daemon, controllerdevice, gyro, touchpad):
        config = {
            'axes': DS5EvdevController.AXIS_MAP,
            'buttons': DS5EvdevController.BUTTON_MAP,
            'dpads': {}
        }
        # if controllerdevice.info.version & 0x8000 == 0:
        # 	# Older kernel uses different mappings
        # 	# see kernel source, drivers/hid/hid-sony.c#L2748
        # 	config['axes'] = DS4EvdevController.AXIS_MAP_OLD
        # 	config['buttons'] = DS4EvdevController.BUTTON_MAP_OLD
        self._gyro = gyro
        self._touchpad = touchpad
        for device in (self._gyro, self._touchpad):
            if device:
                device.grab()
        EvdevController.__init__(self, daemon, controllerdevice, None, config)
        if self.poller:
            self.poller.register(touchpad.fd, self.poller.POLLIN, self._touchpad_input)
            self.poller.register(gyro.fd, self.poller.POLLIN, self._gyro_input)

    def _gyro_input(self, *a):
        new_state = self._state
        try:
            for event in self._gyro.read():
                if event.type == self.ECODES.EV_ABS:
                    axis, factor = DS5EvdevController.GYRO_MAP[event.code]
                    if axis:
                        new_state = new_state._replace(
                            **{axis: int(event.value * factor)}
                        )
        except IOError:
            # Errors here are not even reported, evdev class handles important ones
            return

        if new_state is not self._state:
            old_state, self._state = self._state, new_state
            if self.mapper:
                self.mapper.input(self, old_state, new_state)

    def _touchpad_input(self, *a):
        new_state = self._state
        try:
            for event in self._touchpad.read():
                if event.type == self.ECODES.EV_ABS:
                    if event.code == self.ECODES.ABS_MT_POSITION_X:
                        value = event.value * DS5EvdevController.TOUCH_FACTOR_X
                        value = STICK_PAD_MIN + int(value)
                        new_state = new_state._replace(cpad_x=value)
                    elif event.code == self.ECODES.ABS_MT_POSITION_Y:
                        value = event.value * DS5EvdevController.TOUCH_FACTOR_Y
                        value = STICK_PAD_MAX - int(value)
                        new_state = new_state._replace(cpad_y=value)
                elif event.type == 0:
                    pass
                elif event.code == self.ECODES.BTN_LEFT:
                    if event.value == 1:
                        b = new_state.buttons | SCButtons.CPADPRESS
                        new_state = new_state._replace(buttons=b)
                    else:
                        b = new_state.buttons & ~SCButtons.CPADPRESS
                        new_state = new_state._replace(buttons=b)
                elif event.code == self.ECODES.BTN_TOUCH:
                    if event.value == 1:
                        b = new_state.buttons | SCButtons.CPADTOUCH
                        new_state = new_state._replace(buttons=b)
                    else:
                        b = new_state.buttons & ~SCButtons.CPADTOUCH
                        new_state = new_state._replace(
                            buttons=b,
                            cpad_x=0, cpad_y=0
                            )
        except IOError:
            # Errors here are not even reported, evdev class handles important ones
            return

        if new_state is not self._state:
            old_state, self._state = self._state, new_state
            if self.mapper:
                self.mapper.input(self, old_state, new_state)

    def close(self):
        EvdevController.close(self)
        for device in (self._gyro, self._touchpad):
            try:
                self.poller.unregister(device.fd)
                device.ungrab()
            except:
                pass

    def get_gyro_enabled(self):
        # Cannot be actually turned off, so it's always active
        # TODO: Maybe emulate turning off?
        return True

    def get_type(self):
        return "ds5evdev"

    # TODO: Create ds5-config.json for GUI
    def get_gui_config_file(self):
        return "ds5-config.json"

    def __repr__(self):
        return "<DS5EvdevController %s>" % (self.get_id(),)

    def _generate_id(self):
        """
        ID is generated as 'ds5' or 'ds5:X' where 'X' starts as 1 and increases
        as controllers with same ids are connected.
        """
        magic_number = 1
        id = "ds5"
        while id in self.daemon.get_active_ids():
            id = "ds5:%s" % (magic_number,)
            magic_number += 1
        return id


def init(daemon, config):
    """ Registers hotplug callback for ds5 device """

    def hid_callback(device, handle):
        return DS5Controller(device, daemon, handle, None, None)

    def make_evdev_device(syspath, *whatever):
        devices = get_evdev_devices_from_syspath(syspath)
        # With kernel 4.10 or later, PS4 controller pretends to be 3 different devices.
        # 1st, determining which one is actual controller is needed
        controllerdevice = None
        for device in devices:
            count = len(get_axes(device))
            if count == 8:
                # 8 axes - Controller
                controllerdevice = device
        if not controllerdevice:
            log.warning("Failed to determine controller device")
            return None
        # 2nd, find motion sensor and touchpad with physical address matching controllerdevice
        gyro, touchpad = None, None
        phys = device.phys.split("/")[0]
        for device in devices:
            if device.phys.startswith(phys):
                axes = get_axes(device)
                count = len(axes)
                if count == 6:
                    # 6 axes
                    if EvdevController.ECODES.ABS_MT_POSITION_X in axes:
                        # kernel 4.17+ - touchpad
                        touchpad = device
                    else:
                        # gyro sensor
                        gyro = device
                    pass
                elif count == 4:
                    # 4 axes - Touchpad
                    touchpad = device
        # 3rd, do a magic
        if controllerdevice and gyro and touchpad:
            return make_new_device(DS5EvdevController, controllerdevice, gyro, touchpad)

    def fail_cb(syspath, vid, pid):
        if HAVE_EVDEV:
            log.warning(
                "Failed to acquire USB device, falling back to evdev driver. This is far from optimal."
                )
            make_evdev_device(syspath)
        else:
            log.error(
                "Failed to acquire USB device and evdev is not available. Everything is lost and DS5 support disabled."
                )
        # TODO: Maybe add_error here, but error reporting needs little rework so it's not threated as fatal
        # daemon.add_error("ds5", "No access to DS5 device")

    if config["drivers"].get("hiddrv") or (HAVE_EVDEV and config["drivers"].get("evdevdrv")):
        register_hotplug_device(hid_callback, VENDOR_ID, PRODUCT_ID, on_failure=fail_cb)
        if HAVE_EVDEV and config["drivers"].get("evdevdrv"):
            daemon.get_device_monitor().add_callback(
                "bluetooth",
                VENDOR_ID, PRODUCT_ID, make_evdev_device, None
            )
        return True
    else:
        log.warning("Neither HID nor Evdev driver is enabled, DS5 support cannot be enabled.")
        return False


if __name__ == "__main__":
    """ Called when executed as script """
    init_logging()
    set_logging_level(True, True)
    sys.exit(hiddrv_test(DS5Controller, ["054c:0ce6"]))
