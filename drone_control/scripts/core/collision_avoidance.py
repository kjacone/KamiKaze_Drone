#!/usr/bin/env python3
"""
drone_control/scripts/core/collision_avoidance.py
Collision avoidance using potential fields
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from drone_control.msg import SafetyStatus, NodeHealth
import sys
import os
import math

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from utils.error_handler import ErrorHandler
except ImportError:
    class ErrorHandler:
        def __init__(self, node_name):
            self.node_name = node_name
        def handle_error(self, error, context=None):
            rospy.logerr(f"[{self.node_name}] {error}: {context}")

class CollisionAvoidance:
    """Collision avoidance using potential fields"""
    
    def __init__(self):
        rospy.init_node('collision_avoidance', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='collision_avoidance')
        
        # Parameters
        self.enabled = rospy.get_param('~enable', True)
        self.min_distance = rospy.get_param('~min_distance', 1.5)
        self.safety_margin = rospy.get_param('~safety_margin', 1.2)
        self.avoidance_strength = rospy.get_param('~avoidance_strength', 2.0)
        self.simulation_mode = rospy.get_param('/use_simulation', True)
        
        # State
        self.current_pose = None
        self.obstacles = []
        self.avoidance_vector = np.zeros(3)
        self.last_check_time = rospy.Time.now()
        
        # Subscribers
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_callback)
        if not self.simulation_mode:
            rospy.Subscriber('/scan', LaserScan, self._scan_callback)
            rospy.Subscriber('/pointcloud', PointCloud2, self._pointcloud_callback)
        
        # Publishers
        self.safety_pub = rospy.Publisher('/safety_status', SafetyStatus, queue_size=10)
        self.health_pub = rospy.Publisher('/node_health', NodeHealth, queue_size=10)
        self.avoidance_pub = rospy.Publisher('/avoidance_vector', Point, queue_size=10)
        
        # Timer
        self.check_timer = rospy.Timer(rospy.Duration(0.1), self._check_obstacles)
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)
        
        rospy.loginfo("Collision Avoidance initialized")
        
    def _pose_callback(self, msg):
        self.current_pose = msg.pose
        
    def _scan_callback(self, msg):
        """Process laser scan data"""
        if not self.enabled:
            return
            
        for i, distance in enumerate(msg.ranges):
            if distance < self.min_distance * self.safety_margin and distance > 0:
                angle = msg.angle_min + i * msg.angle_increment
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                
                # Convert to world frame if needed
                if self.current_pose:
                    # Simple obstacle representation
                    self.obstacles.append((x, y, 0.0, distance))
                    
    def _pointcloud_callback(self, msg):
        """Process point cloud data"""
        # Simplified point cloud processing
        # In production, this would use PCL or similar
        pass
        
    def _check_obstacles(self, event):
        """Check for obstacles and compute avoidance"""
        if not self.enabled or self.current_pose is None:
            return
            
        if not self.obstacles:
            # No obstacles detected
            self.avoidance_vector = np.zeros(3)
            self._publish_avoidance()
            return
            
        # Calculate avoidance vector
        avoidance = np.zeros(3)
        drone_pos = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        ])
        
        for obs in self.obstacles:
            obs_pos = np.array([obs[0], obs[1], obs[2]])
            diff = drone_pos - obs_pos
            distance = np.linalg.norm(diff)
            
            if distance < self.min_distance * self.safety_margin:
                # Repulsive force
                strength = self.avoidance_strength / (distance + 0.01)
                avoidance += diff / distance * strength
                
        # Normalize and limit
        mag = np.linalg.norm(avoidance)
        if mag > 0:
            avoidance = avoidance / mag * min(mag, self.avoidance_strength)
            
        self.avoidance_vector = avoidance
        self._publish_avoidance()
        
        # Clear old obstacles (keep only recent)
        self.obstacles = []
        
    def _publish_avoidance(self):
        """Publish avoidance vector"""
        point = Point()
        point.x = self.avoidance_vector[0]
        point.y = self.avoidance_vector[1]
        point.z = self.avoidance_vector[2]
        self.avoidance_pub.publish(point)
        
    def get_avoidance_vector(self):
        """Get current avoidance vector"""
        return self.avoidance_vector
        
    def _publish_health(self, event):
        """Publish node health"""
        health_msg = NodeHealth()
        health_msg.node_name = 'collision_avoidance'
        health_msg.status = 'running' if self.enabled else 'disabled'
        health_msg.timestamp = rospy.Time.now()
        health_msg.is_healthy = True
        self.health_pub.publish(health_msg)

if __name__ == '__main__':
    try:
        avoidance = CollisionAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass