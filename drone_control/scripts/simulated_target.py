#!/usr/bin/env python3
"""
drone_control/scripts/simulated_target.py
Simulated target generator for testing
"""

import math
import time

import rospy
from geometry_msgs.msg import Point, Vector3

from drone_control import TrackedTarget, TrackedTargets


class SimulatedTarget:
    """Generate simulated targets for testing"""
    
    def __init__(self):
        rospy.init_node('simulated_target', anonymous=True)
        
        self.enable = rospy.get_param('~enable', True)
        self.update_rate = rospy.get_param('~update_rate', 30)
        self.scenario = rospy.get_param('~scenario', 'default')
        
        # Publisher
        self.target_pub = rospy.Publisher('/tracked_targets', TrackedTargets, queue_size=10)
        
        # Target state
        self.target_position = Point()
        self.target_position.x = 10.0
        self.target_position.y = 0.0
        self.target_position.z = 2.0
        
        self.target_velocity = Vector3()
        self.target_velocity.x = -0.5
        self.target_velocity.y = 0.0
        self.target_velocity.z = 0.0
        
        self.start_time = time.time()
        
        rospy.loginfo(f"Simulated Target started in scenario: {self.scenario}")
        
        # Start publishing
        if self.enable:
            self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self._publish_target)
        
    def _publish_target(self, event):
        """Publish simulated target"""
        # Update target position (simple circular motion)
        elapsed = time.time() - self.start_time
        
        if self.scenario == 'default':
            # Circular motion
            radius = 5.0
            self.target_position.x = 10.0 + radius * math.cos(elapsed * 0.3)
            self.target_position.y = radius * math.sin(elapsed * 0.3)
            self.target_position.z = 2.0 + 0.5 * math.sin(elapsed * 0.2)
            
            # Update velocity
            self.target_velocity.x = -radius * 0.3 * math.sin(elapsed * 0.3)
            self.target_velocity.y = radius * 0.3 * math.cos(elapsed * 0.3)
            self.target_velocity.z = 0.1 * math.cos(elapsed * 0.2)
            
        elif self.scenario == 'approaching':
            # Target approaching drone
            distance = 20.0 - elapsed * 0.5
            if distance < 1.0:
                distance = 1.0
            self.target_position.x = distance
            self.target_position.y = 0.0
            self.target_position.z = 2.0
            self.target_velocity.x = -0.5
            self.target_velocity.y = 0.0
            self.target_velocity.z = 0.0
            
        elif self.scenario == 'evasive':
            # Target evading with zigzag
            self.target_position.x = 10.0 - elapsed * 0.3
            self.target_position.y = 3.0 * math.sin(elapsed * 0.5)
            self.target_position.z = 2.0 + 0.3 * math.sin(elapsed * 0.7)
            self.target_velocity.x = -0.3
            self.target_velocity.y = 1.5 * math.cos(elapsed * 0.5)
            self.target_velocity.z = 0.21 * math.cos(elapsed * 0.7)
        
        # Create message
        msg = TrackedTargets()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        
        target_msg = TrackedTarget()
        target_msg.id = 1
        target_msg.position = self.target_position
        target_msg.velocity = self.target_velocity
        
        # Calculate distance from drone
        distance = math.sqrt(
            self.target_position.x**2 + 
            self.target_position.y**2 + 
            self.target_position.z**2
        )
        target_msg.distance = distance
        target_msg.confidence = 0.85 + 0.1 * math.sin(time.time() * 0.1)
        target_msg.heading = math.atan2(self.target_position.y, self.target_position.x)
        target_msg.timestamp = rospy.Time.now()
        
        # Determine target state based on distance
        if distance < 2.0:
            target_msg.state = "engaged"
        elif distance < 5.0:
            target_msg.state = "tracking"
        else:
            target_msg.state = "searching"
        
        msg.targets.append(target_msg)
        msg.count = 1
        
        self.target_pub.publish(msg)
        
        rospy.logdebug(f"Published target at distance: {distance:.2f}m")

if __name__ == '__main__':
    try:
        SimulatedTarget()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass