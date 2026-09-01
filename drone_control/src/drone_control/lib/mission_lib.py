#!/usr/bin/env python3
"""
drone_control/lib/mission_lib.py
Mission library functions
"""

import math
import time
from typing import Dict, List, Tuple

from geometry_msgs.msg import Point


class MissionLibrary:
    """Mission-related utility functions"""
    
    @staticmethod
    def generate_search_pattern(pattern_type: str, center: Point, radius: float, spacing: float) -> List[Point]:
        """Generate search pattern waypoints"""
        waypoints = []
        
        if pattern_type == "spiral":
            # Spiral pattern
            theta = 0
            while theta < 4 * math.pi:
                r = theta / (2 * math.pi) * spacing
                x = center.x + r * math.cos(theta)
                y = center.y + r * math.sin(theta)
                waypoints.append(Point(x, y, center.z))
                theta += 0.1
                
        elif pattern_type == "lawnmower":
            # Lawnmower pattern
            width = radius
            height = radius
            step = spacing
            
            y = -height / 2
            direction = 1
            while y <= height / 2:
                x = -width / 2
                while x <= width / 2:
                    waypoints.append(Point(center.x + x, center.y + y, center.z))
                    x += step / 2
                y += step
                direction *= -1
                
        return waypoints
        
    @staticmethod
    def calculate_distance(p1: Point, p2: Point) -> float:
        """Calculate distance between two points"""
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
        
    @staticmethod
    def calculate_heading(p1: Point, p2: Point) -> float:
        """Calculate heading angle from p1 to p2"""
        return math.atan2(p2.y - p1.y, p2.x - p1.x)