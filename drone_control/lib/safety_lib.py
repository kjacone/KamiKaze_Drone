#!/usr/bin/env python3
"""
drone_control/lib/safety_lib.py
Safety library functions
"""

import math
from typing import Tuple, List
from geometry_msgs.msg import Point

class SafetyLibrary:
    """Safety-related utility functions"""
    
    @staticmethod
    def check_geofence(position: Tuple[float, float, float],
                       center: Tuple[float, float, float],
                       radius: float) -> Tuple[bool, float]:
        """Check if position is within geofence"""
        distance = math.sqrt((position[0] - center[0])**2 + 
                            (position[1] - center[1])**2)
        return distance <= radius, distance
        
    @staticmethod
    def check_altitude(altitude: float, 
                       min_alt: float, 
                       max_alt: float) -> Tuple[bool, str]:
        """Check if altitude is within safe range"""
        if altitude < min_alt:
            return False, "Too low"
        elif altitude > max_alt:
            return False, "Too high"
        return True, "OK"
        
    @staticmethod
    def calculate_avoidance_vector(current: Tuple[float, float, float],
                                   obstacles: List[Tuple[float, float, float, float]],
                                   min_distance: float) -> Tuple[float, float, float]:
        """Calculate avoidance vector from obstacles"""
        avoidance = [0.0, 0.0, 0.0]
        
        for obs in obstacles:
            # obs: (x, y, z, radius)
            dx = current[0] - obs[0]
            dy = current[1] - obs[1]
            dz = current[2] - obs[2]
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance < min_distance + obs[3]:
                strength = 1.0 / (distance + 0.01)
                avoidance[0] += dx * strength
                avoidance[1] += dy * strength
                avoidance[2] += dz * strength
                
        # Normalize
        mag = math.sqrt(avoidance[0]**2 + avoidance[1]**2 + avoidance[2]**2)
        if mag > 0:
            avoidance = [a / mag for a in avoidance]
            
        return tuple(avoidance)