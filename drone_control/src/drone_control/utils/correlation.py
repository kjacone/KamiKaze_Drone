#!/usr/bin/env python3
"""
drone_control/scripts/utils/correlation.py
Correlation ID management for mission tracking
"""

import uuid
from typing import Optional

import rospy


def get_or_create_mission_id() -> str:
    """
    Get existing mission_id from ROS parameter or create a new one.
    
    Returns:
        str: The mission ID (UUID string)
    """
    if rospy.has_param("/mission_id"):
        return rospy.get_param("/mission_id")
    
    new_id = str(uuid.uuid4())
    rospy.set_param("/mission_id", new_id)
    return new_id


def get_mission_id() -> Optional[str]:
    """
    Get the current mission_id from ROS parameter without creating one.
    
    Returns:
        Optional[str]: The mission ID or None if not set
    """
    if rospy.has_param("/mission_id"):
        return rospy.get_param("/mission_id")
    return None


def clear_mission_id() -> None:
    """
    Clear the mission_id from ROS parameters.
    """
    if rospy.has_param("/mission_id"):
        rospy.delete_param("/mission_id")


def generate_correlation_id() -> str:
    """
    Generate a correlation ID for distributed tracing.
    
    Returns:
        str: A UUID string for correlation
    """
    return str(uuid.uuid4())


def get_or_create_correlation_id(prefix: str = "mission") -> str:
    """
    Get or create a correlation ID with optional prefix.
    
    Args:
        prefix: Optional prefix for the correlation ID
        
    Returns:
        str: Correlation ID (prefixed UUID)
    """
    if rospy.has_param("/correlation_id"):
        return rospy.get_param("/correlation_id")
    
    corr_id = f"{prefix}_{uuid.uuid4()}"
    rospy.set_param("/correlation_id", corr_id)
    return corr_id