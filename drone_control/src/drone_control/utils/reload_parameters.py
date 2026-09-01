#!/usr/bin/env python3
# drone_control/scripts/reload_parameters.py

import os
import sys

import rospy
from std_srvs.srv import Empty, EmptyResponse


class ParameterReloadClient:
    """Client for reloading parameters at runtime"""
    
    def __init__(self):
        self.reload_service = "/reload_parameters"
        
        # Wait for service
        try:
            rospy.wait_for_service(self.reload_service, timeout=5.0)
            self.reload_proxy = rospy.ServiceProxy(self.reload_service, Empty)
            rospy.loginfo("Connected to parameter reload service")
        except rospy.ROSException:
            rospy.logwarn("Parameter reload service not available")
            self.reload_proxy = None
        
    def reload_all(self):
        """Reload all configuration files"""
        if self.reload_proxy is None:
            rospy.logerr("Parameter reload service not available")
            return False
            
        try:
            response = self.reload_proxy()
            rospy.loginfo("All parameters reloaded successfully")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to reload parameters: {e}")
            return False

def main():
    rospy.init_node('parameter_reload_client')
    
    client = ParameterReloadClient()
    
    if len(sys.argv) > 1:
        if sys.argv[1] == 'all':
            client.reload_all()
        else:
            rospy.logerr(f"Unknown argument: {sys.argv[1]}")
            print("Usage: rosrun drone_control reload_parameters.py all")
    else:
        print("Usage: rosrun drone_control reload_parameters.py all")
        print("Reloads all configuration files at runtime")

if __name__ == "__main__":
    main()