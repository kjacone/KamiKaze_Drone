#!/usr/bin/env python3
"""
drone_control/scripts/controllers/collision_avoidance.py
Enhanced collision avoidance using multi-sensor fusion
"""

import rospy
import numpy as np
from enum import Enum
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from drone_control.msg import TrackedTarget, SafetyStatus
from std_msgs.msg import String, Bool
import sys
import os
import math
import cv2
import time
from typing import List, Tuple, Optional, Dict, Set

sys.path.append(os.path.join(os.path.dirname(__file__), 'utils'))

try:
    from error_handler import ErrorHandler
    from lib.safety_lib import SafetyLibrary
except ImportError:
    class ErrorHandler:
        def __init__(self, node_name):
            self.node_name = node_name
        def handle_error(self, error, context=None):
            rospy.logerr(f"[{self.node_name}] {error}: {context}")
    
    class SafetyLibrary:
        @staticmethod
        def check_geofence(position, center, radius):
            distance = math.sqrt((position[0] - center[0])**2 + 
                                (position[1] - center[1])**2)
            return distance <= radius, distance
        
        @staticmethod
        def check_altitude(altitude, min_alt, max_alt):
            if altitude < min_alt:
                return False, "Too low"
            elif altitude > max_alt:
                return False, "Too high"
            return True, "OK"

class CollisionAvoidanceStatus(Enum):
    ACTIVE = "active"
    MONITORING = "monitoring"
    EVADING = "evading"
    EMERGENCY = "emergency"
    DISABLED = "disabled"

class CollisionAvoidance:
    """Enhanced collision avoidance using multi-sensor fusion"""
    
    def __init__(self):
        rospy.init_node('collision_avoidance', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='collision_avoidance')
        self.safety_lib = SafetyLibrary()
        
        # Configuration parameters
        self.min_obstacle_distance = rospy.get_param('~min_obstacle_distance', 1.0)
        self.max_obstacle_distance = rospy.get_param('~max_obstacle_distance', 20.0)
        self.avoidance_velocity = rospy.get_param('~avoidance_velocity', 3.0)
        self.avoidance_steering = rospy.get_param('~avoidance_steering', 0.5)
        self.simulation_mode = rospy.get_param('/use_simulation', True)
        self.test_mode = rospy.get_param('/test_mode', False)
        
        # Sensor configurations
        self.lidar_enabled = rospy.get_param('~lidar_enabled', False)
        self.camera_enabled = rospy.get_param('~camera_enabled', True)
        self.radar_enabled = rospy.get_param('~radar_enabled', False)
        self.infrared_enabled = rospy.get_param('~infrared_enabled', False)
        
        # State variables
        self.current_position = None
        self.current_velocity = None
        self.current_yaw = 0.0
        self.obstacle_points = []
        self.detected_obstacles = []
        self.lidar_data = None
        self.camera_detections = []
        self.radar_detections = []
        self.infrared_detections = []
        self.collision_warning_timer = 0.0
        self.emergency_stop_timer = 0.0
        self.status = CollisionAvoidanceStatus.MONITORING
        self.last_obstacle_detection = 0.0
        self.collision_risk_level = 0.0
        self.avoidance_path = []
        self.escape_vector = np.zeros(3)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odometry_callback)
        self.lidar_sub = rospy.Subscriber('/lidar/scan', LaserScan, self.lidar_callback) if self.lidar_enabled else None
        self.camera_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.camera_callback) if self.camera_enabled else None
        self.radar_sub = rospy.Subscriber('/radar/detections', String, self.radar_callback) if self.radar_enabled else None
        self.infrared_sub = rospy.Subscriber('/infrared/detections', String, self.infrared_callback) if self.infrared_enabled else None
        self.tracked_targets_sub = rospy.Subscriber('/tracked_targets', TrackedTarget, self.tracked_targets_callback)
        self.safety_status_sub = rospy.Subscriber('/safety_status', SafetyStatus, self.safety_status_callback)
        
        # Publishers
        self.control_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.avoidance_pub = rospy.Publisher('/collision_avoidance/commands', String, queue_size=10)
        self.obstacle_map_pub = rospy.Publisher('/collision_avoidance/obstacle_map', String, queue_size=10)
        self.avoidance_status_pub = rospy.Publisher('/collision_avoidance/status', String, queue_size=10)
        
        # Timers
        self.avoidance_timer = rospy.Timer(rospy.Duration(0.1), self.avoidance_loop)
        self.obstacle_map_timer = rospy.Timer(rospy.Duration(1.0), self.update_obstacle_map)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        rospy.loginfo("Collision Avoidance initialized")
    
    def odometry_callback(self, msg):
        """Update current vehicle state"""
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        self.current_velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Extract current yaw from quaternion
        from tf.transformations import euler_from_quaternion
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.current_yaw = yaw
    
    def lidar_callback(self, msg):
        """Process LiDAR data"""
        if not self.lidar_enabled:
            return
        
        try:
            # Convert LiDAR scan to point cloud
            points = []
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_increment = msg.angle_increment
            
            for i, range_val in enumerate(msg.ranges):
                if range_val > 0 and range_val < self.max_obstacle_distance:
                    angle = angle_min + i * angle_increment
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)
                    points.append((x, y, 0.0))
            
            self.lidar_data = points
            
            # Update obstacle list
            for point in points:
                self.update_obstacle_detection('lidar', point)
            
            rospy.logdebug(f"LiDAR detected {len(points)} points")
        except Exception as e:
            rospy.logwarn(f"LiDAR processing error: {e}")
    
    def camera_callback(self, msg):
        """Process camera data"""
        if not self.camera_enabled:
            return
        
        try:
            # Convert image to numpy array
            # Note: This is a simplified implementation
            # In practice, you'd use cv_bridge to convert ROS image to OpenCV
            rospy.logdebug("Camera data received (simplified processing)")
            
            # Simulate obstacle detection
            if not self.test_mode:
                # Real processing would go here
                pass
            else:
                # Test mode: simulate obstacles
                for i in range(5):
                    angle = math.radians(i * 72)  # 72 degrees apart
                    distance = self.min_obstacle_distance + i * 2.0
                    point = (distance * math.cos(angle), distance * math.sin(angle), 0.5)
                    self.update_obstacle_detection('camera', point)
        
        except Exception as e:
            rospy.logwarn(f"Camera processing error: {e}")
    
    def radar_callback(self, msg):
        """Process radar data"""
        if not self.radar_enabled:
            return
        
        try:
            # Parse radar detection message
            radar_data = json.loads(msg.data)
            
            for detection in radar_data.get('detections', []):
                point = (detection['x'], detection['y'], detection['z'])
                self.update_obstacle_detection('radar', point)
            
            rospy.logdebug(f"Radar detected {len(radar_data.get('detections', []))} objects")
        except Exception as e:
            rospy.logwarn(f"Radar processing error: {e}")
    
    def infrared_callback(self, msg):
        """Process infrared data"""
        if not self.infrared_enabled:
            return
        
        try:
            # Parse infrared detection message
            ir_data = json.loads(msg.data)
            
            for detection in ir_data.get('detections', []):
                point = (detection['x'], detection['y'], detection['z'])
                self.update_obstacle_detection('infrared', point)
            
            rospy.logdebug(f"Infrared detected {len(ir_data.get('detections', []))} objects")
        except Exception as e:
            rospy.logwarn(f"Infrared processing error: {e}")
    
    def tracked_targets_callback(self, msg):
        """Process tracked target updates"""
        if msg.targets:
            # Update obstacle list with tracked targets
            for target in msg.targets:
                point = (target.position.x, target.position.y, target.position.z)
                self.update_obstacle_detection('target', point)
    
    def safety_status_callback(self, msg):
        """Process safety status updates"""
        if not msg.is_safe:
            rospy.logwarn(f"Safety violation: {msg.violations}")
            self.status = CollisionAvoidanceStatus.EMERGENCY
            self.trigger_emergency_evasion()
    
    def update_obstacle_detection(self, sensor_type, point):
        """Update obstacle detection list"""
        # Check if this is a new obstacle
        is_new_obstacle = True
        
        for obstacle in self.detected_obstacles:
            if sensor_type == obstacle['sensor']:
                distance = math.sqrt((obstacle['point'][0] - point[0])**2 +
                                   (obstacle['point'][1] - point[1])**2 +
                                   (obstacle['point'][2] - point[2])**2)
                
                if distance < 0.5:  # Same obstacle within 0.5m
                    obstacle['point'] = point
                    obstacle['last_seen'] = time.time()
                    is_new_obstacle = False
                    break
        
        # Add new obstacle
        if is_new_obstacle:
            obstacle = {
                'point': point,
                'sensor': sensor_type,
                'distance': math.sqrt(point[0]**2 + point[1]**2 + point[2]**2),
                'last_seen': time.time(),
                'velocity': np.zeros(3),
                'type': 'unknown'
            }
            
            self.detected_obstacles.append(obstacle)
            self.last_obstacle_detection = time.time()
            
            rospy.loginfo(f"New obstacle detected by {sensor_type}: {point}, distance: {obstacle['distance']:.2f}m")
    
    def avoidance_loop(self, event):
        """Main avoidance loop"""
        current_time = time.time()
        
        # Update obstacle velocities
        self.update_obstacle_velocities()
        
        # Calculate collision risk
        self.collision_risk_level = self.calculate_collision_risk()
        
        # Check for immediate collision
        if self.collision_risk_level > 0.8:
            self.status = CollisionAvoidanceStatus.EMERGENCY
            self.trigger_immediate_evasion()
        elif self.collision_risk_level > 0.5:
            self.status = CollisionAvoidanceStatus.EVADING
        elif self.collision_risk_level > 0.2:
            self.status = CollisionAvoidanceStatus.MONITORING
        else:
            self.status = CollisionAvoidanceStatus.ACTIVE
        
        # Calculate escape vector
        self.calculate_escape_vector()
        
        # Execute avoidance maneuver
        if self.status == CollisionAvoidanceStatus.EVADING:
            self.execute_avoidance_maneuver()
        elif self.status == CollisionAvoidanceStatus.EMERGENCY:
            self.execute_emergency_evasion()
    
    def update_obstacle_velocities(self):
        """Update obstacle velocities based on movement patterns"""
        for obstacle in self.detected_obstacles:
            # Simple velocity estimation from position changes
            if 'last_position' in obstacle:
                prev_pos = obstacle['last_position']
                current_pos = obstacle['point']
                
                velocity = np.array([
                    current_pos[0] - prev_pos[0],
                    current_pos[1] - prev_pos[1],
                    current_pos[2] - prev_pos[2]
                ]) / 0.1  # 0.1 second time step
                
                obstacle['velocity'] = velocity
            
            obstacle['last_position'] = obstacle['point']
    
    def calculate_collision_risk(self):
        """Calculate collision risk based on obstacle proximity"""
        if self.current_position is None:
            return 0.0
        
        risk = 0.0
        close_obstacles = []
        
        for obstacle in self.detected_obstacles:
            distance = math.sqrt((obstacle['point'][0] - self.current_position[0])**2 +
                               (obstacle['point'][1] - self.current_position[1])**2 +
                               (obstacle['point'][2] - self.current_position[2])**2)
            
            # Calculate risk based on distance
            if distance < self.min_obstacle_distance:
                risk += 1.0  # High risk
                close_obstacles.append((obstacle, distance))
            elif distance < 2.0 * self.min_obstacle_distance:
                risk += 0.5  # Medium risk
                close_obstacles.append((obstacle, distance))
            elif distance < 5.0 * self.min_obstacle_distance:
                risk += 0.2  # Low risk
                close_obstacles.append((obstacle, distance))
        
        # Normalize risk
        if close_obstacles:
            # Adjust risk based on number of close obstacles
            risk_factor = min(1.0, risk / len(close_obstacles))
        else:
            risk_factor = 0.0
        
        return risk_factor
    
    def calculate_escape_vector(self):
        """Calculate escape vector to avoid collision"""
        self.escape_vector = np.zeros(3)
        
        if self.collision_risk_level < 0.2:
            return
        
        # Calculate weighted escape vector based on obstacle positions
        weights = []
        vectors = []
        
        for obstacle in self.detected_obstacles:
            # Calculate vector away from obstacle
            distance = math.sqrt((obstacle['point'][0] - self.current_position[0])**2 +
                               (obstacle['point'][1] - self.current_position[1])**2 +
                               (obstacle['point'][2] - self.current_position[2])**2)
            
            if distance < self.max_obstacle_distance:
                away_vector = np.array([
                    self.current_position[0] - obstacle['point'][0],
                    self.current_position[1] - obstacle['point'][1],
                    self.current_position[2] - obstacle['point'][2]
                ])
                
                if np.linalg.norm(away_vector) > 0:
                    away_vector = away_vector / np.linalg.norm(away_vector)
                    
                    # Weight by inverse distance
                    weight = 1.0 / (distance + 0.1)
                    
                    weights.append(weight)
                    vectors.append(away_vector)
        
        # Combine weighted vectors
        if weights:
            weights = np.array(weights)
            weighted_vectors = np.array(vectors)
            
            # Calculate weighted average
            combined_vector = np.sum(weighted_vectors * weights[:, np.newaxis], axis=0)
            
            if np.linalg.norm(combined_vector) > 0:
                combined_vector = combined_vector / np.linalg.norm(combined_vector)
                
                # Scale to desired escape speed
                self.escape_vector = combined_vector * self.avoidance_velocity
    
    def execute_avoidance_maneuver(self):
        """Execute avoidance maneuver"""
        if np.linalg.norm(self.escape_vector) == 0:
            return
        
        # Create control command
        control_command = String()
        command_data = {
            'type': 'avoidance_maneuver',
            'escape_vector': self.escape_vector.tolist(),
            'avoidance_velocity': self.avoidance_velocity,
            'avoidance_steering': self.avoidance_steering,
            'status': self.status.value
        }
        
        control_command.data = json.dumps(command_data)
        self.avoidance_pub.publish(control_command)
        
        # Apply escape vector to current velocity
        new_velocity = self.current_velocity + self.escape_vector
        
        # Limit velocity
        velocity_magnitude = np.linalg.norm(new_velocity)
        if velocity_magnitude > self.avoidance_velocity:
            new_velocity = new_velocity * (self.avoidance_velocity / velocity_magnitude)
        
        # Create twist command
        twist = Twist()
        twist.linear.x = new_velocity[0]
        twist.linear.y = new_velocity[1]
        twist.linear.z = new_velocity[2]
        
        # Publish control command
        self.control_pub.publish(twist)
        
        rospy.loginfo(f"Executing avoidance maneuver: {self.escape_vector}")
    
    def trigger_immediate_evasion(self):
        """Trigger immediate emergency evasion"""
        rospy.logerr("IMMEDIATE COLLISION EVASION TRIGGERED!")
        
        # Calculate emergency escape vector
        self.escape_vector = np.array([-self.current_velocity[0], -self.current_velocity[1], 0.0])
        velocity_magnitude = np.linalg.norm(self.escape_vector)
        
        if velocity_magnitude > 0:
            self.escape_vector = self.escape_vector / velocity_magnitude * self.avoidance_velocity * 2.0
        
        # Execute emergency maneuver
        self.execute_emergency_evasion()
    
    def trigger_emergency_evasion(self):
        """Execute emergency evasion maneuver"""
        # Stop current movement
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.linear.y = 0.0
        stop_twist.linear.z = 0.0
        self.control_pub.publish(stop_twist)
        
        # Publish emergency status
        status_msg = String()
        status_msg.data = json.dumps({
            'status': 'emergency_evasion',
            'reason': 'immediate_collision_imminent',
            'escape_vector': self.escape_vector.tolist()
        })
        self.avoidance_status_pub.publish(status_msg)
        
        # Start emergency timer
        self.emergency_stop_timer = time.time()
        
        # Change status
        self.status = CollisionAvoidanceStatus.EMERGENCY
        
        rospy.logwarn("Emergency evasion activated - STOPPING IMMEDIATELY")
    
    def update_obstacle_map(self, event):
        """Update and publish obstacle map"""
        if not self.detected_obstacles:
            return
        
        # Create obstacle map message
        map_data = {
            'timestamp': time.time(),
            'obstacle_count': len(self.detected_obstacles),
            'obstacles': []
        }
        
        for obstacle in self.detected_obstacles:
            map_data['obstacles'].append({
                'position': obstacle['point'],
                'sensor': obstacle['sensor'],
                'distance': obstacle['distance'],
                'velocity': obstacle['velocity'].tolist(),
                'type': obstacle['type']
            })
        
        # Publish obstacle map
        map_msg = String()
        map_msg.data = json.dumps(map_data)
        self.obstacle_map_pub.publish(map_msg)
    
    def publish_status(self, event):
        """Publish collision avoidance status"""
        status_msg = String()
        
        status_info = {
            'status': self.status.value,
            'obstacle_count': len(self.detected_obstacles),
            'collision_risk_level': self.collision_risk_level,
            'escape_vector': self.escape_vector.tolist(),
            'emergency_stop_timer': time.time() - self.emergency_stop_timer if self.emergency_stop_timer > 0 else 0.0,
            'enabled_sensors': {
                'lidar': self.lidar_enabled,
                'camera': self.camera_enabled,
                'radar': self.radar_enabled,
                'infrared': self.infrared_enabled
            }
        }
        
        status_msg.data = json.dumps(status_info)
        self.avoidance_status_pub.publish(status_msg)

if __name__ == '__main__':
    try:
        avoidance = CollisionAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass