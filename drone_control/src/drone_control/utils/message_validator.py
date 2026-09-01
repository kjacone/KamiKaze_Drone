#!/usr/bin/env python3
"""
drone_control/scripts/utils/message_validator.py
ROS message validation utilities
"""

from typing import Any, Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import Image


class MessageValidator:
    """Validate ROS messages for integrity and correctness"""
    
    def __init__(self):
        self.validation_rules = {
            'image': {
                'min_width': 1,
                'max_width': 4096,
                'min_height': 1,
                'max_height': 4096,
                'valid_encodings': ['bgr8', 'rgb8', 'mono8', 'yuv422']
            },
            'point': {
                'bounds': {'x': (-1000, 1000), 'y': (-1000, 1000), 'z': (-1000, 1000)}
            },
            'pose': {
                'bounds': {'x': (-1000, 1000), 'y': (-1000, 1000), 'z': (-100, 100)}
            },
            'twist': {
                'bounds': {'linear': (-20, 20), 'angular': (-10, 10)}
            }
        }
        
    def validate_image(self, msg: Image) -> Tuple[bool, Optional[str]]:
        """Validate an Image message"""
        try:
            # Check dimensions
            if msg.width < self.validation_rules['image']['min_width']:
                return False, f"Image width too small: {msg.width}"
            if msg.width > self.validation_rules['image']['max_width']:
                return False, f"Image width too large: {msg.width}"
            if msg.height < self.validation_rules['image']['min_height']:
                return False, f"Image height too small: {msg.height}"
            if msg.height > self.validation_rules['image']['max_height']:
                return False, f"Image height too large: {msg.height}"
                
            # Check encoding
            if msg.encoding not in self.validation_rules['image']['valid_encodings']:
                return False, f"Invalid image encoding: {msg.encoding}"
                
            # Check data size
            expected_size = msg.width * msg.height
            if msg.encoding in ['bgr8', 'rgb8']:
                expected_size *= 3
            elif msg.encoding == 'mono8':
                expected_size *= 1
                
            if len(msg.data) != expected_size:
                return False, f"Image data size mismatch: {len(msg.data)} != {expected_size}"
                
            return True, None
            
        except Exception as e:
            return False, f"Image validation error: {e}"
            
    def validate_point(self, msg: Point) -> Tuple[bool, Optional[str]]:
        """Validate a Point message"""
        bounds = self.validation_rules['point']['bounds']
        
        if not (bounds['x'][0] <= msg.x <= bounds['x'][1]):
            return False, f"Point x out of bounds: {msg.x}"
        if not (bounds['y'][0] <= msg.y <= bounds['y'][1]):
            return False, f"Point y out of bounds: {msg.y}"
        if not (bounds['z'][0] <= msg.z <= bounds['z'][1]):
            return False, f"Point z out of bounds: {msg.z}"
            
        return True, None
        
    def validate_pose(self, msg: Pose) -> Tuple[bool, Optional[str]]:
        """Validate a Pose message"""
        bounds = self.validation_rules['pose']['bounds']
        
        # Validate position
        if not (bounds['x'][0] <= msg.position.x <= bounds['x'][1]):
            return False, f"Position x out of bounds: {msg.position.x}"
        if not (bounds['y'][0] <= msg.position.y <= bounds['y'][1]):
            return False, f"Position y out of bounds: {msg.position.y}"
        if not (bounds['z'][0] <= msg.position.z <= bounds['z'][1]):
            return False, f"Position z out of bounds: {msg.position.z}"
            
        # Validate orientation (quaternion should be normalized)
        q = msg.orientation
        norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if not 0.99 <= norm <= 1.01:
            return False, f"Quaternion not normalized: {norm}"
            
        return True, None
        
    def validate_twist(self, msg: Twist) -> Tuple[bool, Optional[str]]:
        """Validate a Twist message"""
        bounds = self.validation_rules['twist']['bounds']
        
        # Validate linear
        if not (bounds['linear'][0] <= msg.linear.x <= bounds['linear'][1]):
            return False, f"Linear x out of bounds: {msg.linear.x}"
        if not (bounds['linear'][0] <= msg.linear.y <= bounds['linear'][1]):
            return False, f"Linear y out of bounds: {msg.linear.y}"
        if not (bounds['linear'][0] <= msg.linear.z <= bounds['linear'][1]):
            return False, f"Linear z out of bounds: {msg.linear.z}"
            
        # Validate angular
        if not (bounds['angular'][0] <= msg.angular.x <= bounds['angular'][1]):
            return False, f"Angular x out of bounds: {msg.angular.x}"
        if not (bounds['angular'][0] <= msg.angular.y <= bounds['angular'][1]):
            return False, f"Angular y out of bounds: {msg.angular.y}"
        if not (bounds['angular'][0] <= msg.angular.z <= bounds['angular'][1]):
            return False, f"Angular z out of bounds: {msg.angular.z}"
            
        return True, None