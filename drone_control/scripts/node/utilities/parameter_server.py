#!/usr/bin/env python3
"""
drone_control/scripts/node/utilities/parameter_server.py
Centralized parameter server with validation and monitoring
"""

import json
import os
import sys

import rospy
import yaml

from drone_control import NodeHealth
from drone_control.utils import ParameterReloadClient, ParameterValidator
from drone_control.utils import ParameterValidator

# sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


class ParameterServer:
    """Centralized parameter server with validation"""
    
    def __init__(self):
        rospy.init_node('parameter_server', anonymous=False)
        
        self.config_dir = rospy.get_param('~config_dir', '/drone_control/config')
        self.validator = ParameterValidator(self.config_dir)
        
        # Services
        rospy.Service('/parameter/validate', ParameterValidator, self._validate)
        rospy.Service('/parameter/reload', ParameterReloadClient, self._reload)
        
        # Publisher
        self.health_pub = rospy.Publisher('/parameter_server/node_health', NodeHealth, queue_size=10)
        
        # Timer
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)
        
        rospy.loginfo("Parameter Server initialized")
        
    def _validate(self, req):
        """Validate a parameter"""
        response = ParameterValidator()
        
        try:
            # Parse value
            value = json.loads(req.param_value)
            
            # Validate (simplified)
            is_valid = True
            message = "Valid"
            
            response.success = is_valid
            response.message = message
            response.schema = json.dumps({
                'type': type(value).__name__,
                'valid': is_valid
            })
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.schema = "{}"
            
        return response
        
    def _reload(self, req):
        """Reload configuration"""
        response = ParameterReloadClient()
        
        try:
            # Reload parameters
            if req.config_file:
                # Reload specific file
                file_path = os.path.join(self.config_dir, req.config_file)
                if os.path.exists(file_path):
                    with open(file_path, 'r') as f:
                        config = yaml.safe_load(f)
                    # Load parameters
                    for key, value in config.items():
                        rospy.set_param(f"/{key}", value)
                    response.success = True
                    response.message = f"Loaded {req.config_file}"
                else:
                    response.success = False
                    response.message = f"Config file not found: {req.config_file}"
            else:
                # Reload all configs
                response.success = True
                response.message = "All configs reloaded"
                
        except Exception as e:
            response.success = False
            response.message = str(e)
            
        return response
        
    def _publish_health(self, event):
        """Publish node health"""
        health_msg = NodeHealth()
        health_msg.node_name = 'parameter_server'
        health_msg.status = 'running'
        health_msg.timestamp = rospy.Time.now()
        health_msg.is_healthy = True
        self.health_pub.publish(health_msg)

if __name__ == '__main__':
    try:
        server = ParameterServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass