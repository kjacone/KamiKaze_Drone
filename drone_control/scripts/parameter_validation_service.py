#!/usr/bin/env python3
# drone_control/scripts/parameter_validation_service.py

import rospy
import yaml
import json
import os
from std_srvs.srv import Trigger, TriggerResponse

class ParameterValidationService:
    """Service for validating parameters at runtime"""
    
    def __init__(self):
        rospy.init_node('parameter_validation_service')
        
        # Setup services
        self.validate_config_srv = rospy.Service(
            'validate_configuration', 
            Trigger, 
            self._validate_configuration_callback
        )
        
        rospy.loginfo("Parameter validation service started")
        
    def _validate_configuration_callback(self, req):
        """Validate entire configuration"""
        response = TriggerResponse()
        
        # Get all parameters
        param_names = rospy.get_param_names()
        errors = []
        
        # Basic validation - check for required parameters
        required_params = [
            '/target_params/detection/confidence_threshold',
            '/target_params/tracking/engagement_distance',
            '/flight_control/dynamics/constraints/max_velocity',
            '/system_params/safety/geofence/radius'
        ]
        
        for param in required_params:
            if not rospy.has_param(param):
                errors.append(f"Missing required parameter: {param}")
                
        # Check for valid values
        try:
            # Check confidence threshold
            confidence = rospy.get_param('/target_params/detection/confidence_threshold', 0.5)
            if not 0 <= confidence <= 1:
                errors.append(f"Confidence threshold must be between 0 and 1, got {confidence}")
                
            # Check geofence radius
            radius = rospy.get_param('/system_params/safety/geofence/radius', 100.0)
            if radius <= 0:
                errors.append(f"Geofence radius must be positive, got {radius}")
                
        except Exception as e:
            errors.append(f"Error validating parameters: {e}")
            
        response.success = len(errors) == 0
        if response.success:
            response.message = "Configuration is valid"
        else:
            response.message = f"Configuration validation failed: {'; '.join(errors)}"
            
        return response

if __name__ == "__main__":
    try:
        service = ParameterValidationService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass