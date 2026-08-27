#!/usr/bin/env python3
"""
drone_control/scripts/node/core/mission_manager.py
Link to controllers/mission_manager.py
"""

# This is a symbolic link - the actual implementation is in controllers/

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from controllers.mission_manager import MissionManager

if __name__ == '__main__':
    try:
        manager = MissionManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass