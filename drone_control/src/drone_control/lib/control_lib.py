#!/usr/bin/env python3
"""
drone_control/lib/control_lib.py
Control library functions
"""

import math
from typing import Tuple

import numpy as np
from geometry_msgs.msg import Twist, Vector3


class ControlLibrary:
    """Control-related utility functions"""
    
    @staticmethod
    def calculate_velocity_to_target(current: Tuple[float, float, float], 
                                     target: Tuple[float, float, float],
                                     max_speed: float) -> Twist:
        """Calculate velocity command to move towards target"""
        # Calculate error
        error = np.array(target) - np.array(current)
        distance = np.linalg.norm(error)
        
        if distance < 0.1:
            return Twist()  # Zero velocity
            
        # Proportional control
        speed = min(max_speed, distance * 0.5)
        direction = error / distance
        
        cmd = Twist()
        cmd.linear.x = direction[0] * speed
        cmd.linear.y = direction[1] * speed
        cmd.linear.z = direction[2] * speed
        
        return cmd
        
    @staticmethod
    def calculate_yaw(current: Tuple[float, float, float],
                     target: Tuple[float, float, float]) -> float:
        """Calculate yaw angle to face target"""
        return math.atan2(target[1] - current[1], target[0] - current[0])
        
    @staticmethod
    def smooth_velocity(current: Twist, target: Twist, 
                        max_accel: float, dt: float) -> Twist:
        """Smooth velocity commands with acceleration limits"""
        smooth = Twist()
        
        # Linear velocity smoothing
        for axis in ['x', 'y', 'z']:
            cur = getattr(current.linear, axis)
            tgt = getattr(target.linear, axis)
            diff = tgt - cur
            max_change = max_accel * dt
            
            if abs(diff) > max_change:
                diff = math.copysign(max_change, diff)
                
            setattr(smooth.linear, axis, cur + diff)
            
        # Angular velocity smoothing
        for axis in ['x', 'y', 'z']:
            cur = getattr(current.angular, axis)
            tgt = getattr(target.angular, axis)
            diff = tgt - cur
            max_change = max_accel * dt
            
            if abs(diff) > max_change:
                diff = math.copysign(max_change, diff)
                
            setattr(smooth.angular, axis, cur + diff)
            
        return smooth