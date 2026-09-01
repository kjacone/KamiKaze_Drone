#!/usr/bin/env python3
"""
drone_control/scripts/startup_notifier.py
Startup notification and system initialization
"""

import time

import rospy
from std_msgs.msg import String


class StartupNotifier:
    """Notify system startup and initialization status"""
    
    def __init__(self):
        rospy.init_node('startup_notifier', anonymous=False)
        
        self.mode = rospy.get_param('~mode', 'production')
        self.simulation = rospy.get_param('~simulation', True)
        
        # Publishers
        self.status_pub = rospy.Publisher('/system_status', String, queue_size=10)
        
        # Notify startup
        self._notify_startup()
        
        rospy.loginfo("Startup notifier initialized")
        
    def _notify_startup(self):
        """Publish startup notification"""
        status_msg = String()
        status_msg.data = f"System startup: mode={self.mode}, simulation={self.simulation}"
        self.status_pub.publish(status_msg)
        
        rospy.loginfo(f"System started in {self.mode} mode, simulation={self.simulation}")

if __name__ == '__main__':
    try:
        notifier = StartupNotifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass