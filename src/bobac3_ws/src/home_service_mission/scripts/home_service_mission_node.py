#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy

from home_service_modules.mission_runner import HomeServiceMission


if __name__ == "__main__":
    try:
        HomeServiceMission().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("Home mission failed: %s", exc)
        raise
