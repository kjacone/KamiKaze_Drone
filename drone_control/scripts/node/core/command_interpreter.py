#!/usr/bin/env python3
"""
drone_control/scripts/node/core/command_interpreter.py
Link to controllers/command_interpreter.py
"""

# This is a symbolic link - the actual implementation is in controllers/
# For ROS package compatibility, we import from the actual location

import os
import sys

import rospy

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from ...controllers.command_interpreter import CommandInterpreter

if __name__ == '__main__':
    try:
        interpreter = CommandInterpreter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass