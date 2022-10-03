#!/usr/bin/env python2
"""
SC-Controller - Autoswitch Daemon

Observes active window and commands scc-daemon to change profiles as needed.
"""
from __future__ import unicode_literals

import logging
import os
import sys

from scc.x11.autoswitcher import AutoSwitcher

log = logging.getLogger("AutoswitchDaemon")

if __name__ == "__main__":
	from scc.tools import init_logging, set_logging_level

	init_logging(suffix=" AutoSwitcher")
	set_logging_level('debug' in sys.argv, 'debug' in sys.argv)
	
	if "DISPLAY" not in os.environ:
		log.error("DISPLAY env variable not set.")
		sys.exit(1)
	
	d = AutoSwitcher()
	d.run()
	sys.exit(d.exit_code)
