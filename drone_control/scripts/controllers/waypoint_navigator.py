#!/usr/bin/env python3
"""
drone_control/scripts/controllers/waypoint_navigator.py
Advanced waypoint navigation with automatic route calculation
"""

import rospy
import numpy as np
from enum import Enum
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from drone_control.msg import TrackedTarget
from std_msgs.msg import String
import sys
import os
import math
import json
import time
from typing import List, Tuple, Optional, Dict, Set
from datetime import datetime, timedelta
import yaml

sys.path.append(os.path.join(os.path.dirname(__file__), 'utils'))

try:
    from error_handler import ErrorHandler
    from lib.control_lib import ControlLibrary
except ImportError:
    class ErrorHandler:
        def __init__(self, node_name):
            self.node_name = node_name
        def handle_error(self, error, context=None):
            rospy.logerr(f"[{self.node_name}] {error}: {context}")
    
    class ControlLibrary:
        @staticmethod
        def calculate_velocity_to_target(current, target, max_speed):
            return Twist()

class WaypointStatus(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    ARRIVED = "arrived"
    OFFTRACK = "offtrack"
    EMERGENCY = "emergency"
    RETURNING = "returning"

class WaypointNavigator:
    """Advanced waypoint navigation with automatic route calculation"""
    
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='waypoint_navigator')
        self.control_lib = ControlLibrary()
        
        # Configuration parameters
        self.max_waypoint_distance = rospy.get_param('~max_waypoint_distance', 10.0)
        self.approach_threshold = rospy.get_param('~approach_threshold', 2.0)
        self.compass_deviation_threshold = rospy.get_param('~compass_deviation_threshold', 30.0)
        self.speed_profile = rospy.get_param('~speed_profile', 'balanced')
        self.return_to_home_enabled = rospy.get_param('~return_to_home_enabled', True)
        self.autonomous_waypoints_enabled = rospy.get_param('~autonomous_waypoints_enabled', True)
        self.simulation_mode = rospy.get_param('/use_simulation', True)
        
        # State variables
        self.current_position = None
        self.current_velocity = None
        self.current_yaw = 0.0
        self.current_gps = None
        self.waypoint_sequence = []
        self.current_waypoint_index = 0
        self.mission_waypoints = []
        self.autonomous_waypoints = []
        self.search_pattern_waypoints = []
        self.status = WaypointStatus.IDLE
        self.last_waypoint_arrival = 0.0
        self.offtrack_timer = 0.0
        self.compass_history = []
        self.waypoint_deviation_history = []
        self.home_position = None
        self.last_position = None
        self.position_deviation = 0.0
        self.offtrack_detected = False
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odometry_callback)
        self.gps_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.tracked_targets_sub = rospy.Subscriber('/tracked_targets', TrackedTarget, self.tracked_targets_callback)
        self.mission_sub = rospy.Subscriber('/mission_status', String, self.mission_callback)
        
        # Publishers
        self.waypoint_pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.command_pub = rospy.Publisher('/drone_control/command', String, queue_size=10)
        self.navigator_status_pub = rospy.Publisher('/navigator_status', String, queue_size=10)
        
        # Timers
        self.navigation_timer = rospy.Timer(rospy.Duration(1.0), self.navigation_loop)
        self.waypoint_update_timer = rospy.Timer(rospy.Duration(0.5), self.update_waypoints)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        # Load configurations
        self.load_waypoint_configurations()
        self.load_home_position()
        
        rospy.loginfo("Waypoint Navigator initialized")
    
    def load_waypoint_configurations(self):
        """Load waypoint configurations from files"""
        # Load waypoint patterns
        self.waypoint_patterns = {
            'spiral': self.generate_spiral_pattern,
            'lawnmower': self.generate_lawnmower_pattern,
            'circle': self.generate_circle_pattern,
            'grid': self.generate_grid_pattern,
            'target_approach': self.generate_target_approach_pattern
        }
        
        # Load autonomous waypoint templates
        self.autonomous_templates = {
            'search_pattern': {
                'type': 'spiral',
                'radius_start': 10.0,
                'radius_end': 50.0,
                'spiral_rings': 3,
                'height': 10.0
            },
            'target_tracking': {
                'type': 'approach',
                'max_distance': 20.0,
                'height': 10.0
            },
            'area_search': {
                'type': 'grid',
                'grid_size': 5,
                'spacing': 10.0,
                'height': 10.0
            }
        }
    
    def load_home_position(self):
        """Load home position from configuration file"""
        try:
            # Try to load from config file
            config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'home_position.yaml')
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    self.home_position = config.get('home_position', {
                        'latitude': 0.0,
                        'longitude': 0.0,
                        'altitude': 0.0
                    })
        except Exception as e:
            rospy.logwarn(f"Could not load home position config: {e}")
            self.home_position = {
                'latitude': 0.0,
                'longitude': 0.0,
                'altitude': 0.0
            }
    
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
        
        # Update position deviation tracking
        self.update_position_deviation()
        
        # Check for off-track condition
        self.check_off_track_condition()
    
    def gps_callback(self, msg):
        """Update GPS position"""
        self.current_gps = msg
        
        # Update home position if not set
        if self.home_position is None:
            self.home_position = {
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude
            }
            rospy.loginfo(f"Home position set: {self.home_position}")
    
    def tracked_targets_callback(self, msg):
        """Handle tracked target updates"""
        # This callback handles TrackedTarget messages
        if msg:
            # Log target information
            distance = math.sqrt(msg.position.x**2 + msg.position.y**2 + msg.position.z**2)
            rospy.logdebug(f"Target detected at distance: {distance:.2f}m")
    
    def mission_callback(self, msg):
        """Handle mission state changes"""
        if msg.data == 'emergency':
            self.status = WaypointStatus.EMERGENCY
            self.return_to_home()
        elif msg.data == 'completed':
            self.status = WaypointStatus.IDLE
            self.waypoint_sequence = []
            self.current_waypoint_index = 0
        elif msg.data == 'failed':
            self.status = WaypointStatus.IDLE
            self.return_to_home()
    
    def navigation_loop(self, event):
        """Main navigation loop"""
        if self.status == WaypointStatus.IDLE:
            return
        
        # Update offtrack timer
        if self.offtrack_detected:
            self.offtrack_timer += 1.0
        else:
            self.offtrack_timer = 0.0
        
        # Check if we need to move to next waypoint
        if self.status == WaypointStatus.NAVIGATING:
            self.check_waypoint_arrival()
    
    def update_waypoints(self, event):
        """Update and publish waypoints"""
        if not self.waypoint_sequence:
            return
        
        # Get current waypoint
        current_waypoint = self.waypoint_sequence[self.current_waypoint_index]
        
        # Calculate navigation commands
        distance_to_waypoint = self.calculate_distance_to_waypoint()
        
        # Check if we're on track
        if distance_to_waypoint > self.max_waypoint_distance:
            self.status = WaypointStatus.OFFTRACK
            rospy.logwarn(f"Off track! Distance to waypoint: {distance_to_waypoint:.2f}m")
        
        # Calculate control command
        control_command = self.calculate_control_command()
        
        # Execute waypoint approach
        if self.status == WaypointStatus.ARRIVED:
            self.execute_waypoint_approach()
        else:
            self.execute_offtrack_recovery()
    
    def check_waypoint_arrival(self):
        """Check if current waypoint has been reached"""
        if not self.waypoint_sequence or self.current_waypoint_index >= len(self.waypoint_sequence):
            return
        
        current_waypoint = self.waypoint_sequence[self.current_waypoint_index]
        distance = self.calculate_distance_to_waypoint()
        
        # Check if waypoint is reached
        if distance < self.approach_threshold:
            # Update waypoint arrival time
            self.last_waypoint_arrival = time.time()
            
            # Mark waypoint as arrived
            self.status = WaypointStatus.ARRIVED
            
            # Log arrival
            rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached!")
            
            # Publish arrival notification
            arrival_msg = String()
            arrival_msg.data = f"waypoint_{self.current_waypoint_index}_reached"
            self.command_pub.publish(arrival_msg)
    
    def calculate_control_command(self):
        """Calculate navigation control command"""
        if self.current_position is None or not self.waypoint_sequence:
            return None
        
        current_waypoint = self.waypoint_sequence[self.current_waypoint_index]
        
        # Calculate desired velocity
        desired_velocity = self.calculate_desired_velocity(current_waypoint)
        
        # Calculate control command
        command = String()
        command.data = json.dumps({
            'type': 'navigation_command',
            'target_position': {
                'x': current_waypoint.pose.position.x,
                'y': current_waypoint.pose.position.y,
                'z': current_waypoint.pose.position.z
            },
            'target_yaw': self.extract_yaw_from_pose(current_waypoint.pose),
            'velocity': desired_velocity,
            'status': self.status.value
        })
        
        return command
    
    def calculate_desired_velocity(self, waypoint):
        """Calculate desired velocity to waypoint"""
        if self.current_position is None:
            return 0.0
        
        # Calculate distance to waypoint
        distance = math.sqrt(
            (waypoint.pose.position.x - self.current_position[0])**2 +
            (waypoint.pose.position.y - self.current_position[1])**2 +
            (waypoint.pose.position.z - self.current_position[2])**2
        )
        
        # Adjust speed based on profile
        if self.speed_profile == 'conservative':
            speed = min(distance * 0.3, 2.0)
        elif self.speed_profile == 'balanced':
            speed = min(distance * 0.5, 5.0)
        elif self.speed_profile == 'aggressive':
            speed = min(distance * 0.7, 8.0)
        else:
            speed = min(distance * 0.5, 5.0)
        
        return speed
    
    def execute_waypoint_approach(self):
        """Execute waypoint approach procedure"""
        if self.current_waypoint_index < len(self.waypoint_sequence) - 1:
            # Move to next waypoint
            self.current_waypoint_index += 1
            self.status = WaypointStatus.NAVIGATING
            rospy.loginfo(f"Moving to waypoint {self.current_waypoint_index}")
        else:
            # All waypoints completed
            self.status = WaypointStatus.IDLE
            rospy.loginfo("All waypoints completed!")
            
            # Send mission completion notification
            completion_msg = String()
            completion_msg.data = "all_waypoints_completed"
            self.command_pub.publish(completion_msg)
    
    def execute_offtrack_recovery(self):
        """Execute off-track recovery procedure"""
        if self.offtrack_timer > 30.0:  # 30 seconds of being offtrack
            rospy.logwarn("Emergency recovery - returning to home")
            self.return_to_home()
        elif self.offtrack_timer > 10.0:
            rospy.logwarn("Warning: Offtrack detected - returning to home")
    
    def return_to_home(self):
        """Return to home position"""
        if self.home_position:
            self.status = WaypointStatus.RETURNING
            
            # Create home waypoint
            home_waypoint = PoseStamped()
            home_waypoint.header.stamp = rospy.Time.now()
            home_waypoint.header.frame_id = "map"
            home_waypoint.pose.position.x = self.home_position.get('longitude', 0.0)
            home_waypoint.pose.position.y = self.home_position.get('latitude', 0.0)
            home_waypoint.pose.position.z = self.home_position.get('altitude', 0.0)
            
            # Set to returning home status
            status_msg = String()
            status_msg.data = f"returning_home_{self.home_position}"
            self.navigator_status_pub.publish(status_msg)
            
            # Add to waypoint sequence
            self.waypoint_sequence = [home_waypoint]
            self.current_waypoint_index = 0
            self.status = WaypointStatus.NAVIGATING
    
    def generate_waypoint_sequence(self, waypoints):
        """Generate and execute waypoint sequence"""
        if not waypoints:
            rospy.logwarn("No waypoints provided")
            return
        
        # Convert waypoints to PoseStamped messages
        waypoint_messages = []
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            
            if isinstance(waypoint, tuple) and len(waypoint) == 3:
                # 3D coordinate tuple
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.position.z = waypoint[2]
                
                # Calculate yaw to next waypoint if available
                if len(waypoint) > 3:
                    next_pos = waypoint[3:6]
                    dyaw = math.atan2(next_pos[1] - waypoint[1], 
                                     next_pos[0] - waypoint[0])
                    self.set_quaternion_from_yaw(pose.pose, dyaw)
            elif isinstance(waypoint, PoseStamped):
                pose = waypoint
            
            waypoint_messages.append(pose)
        
        # Set waypoint sequence
        self.waypoint_sequence = waypoint_messages
        self.current_waypoint_index = 0
        self.status = WaypointStatus.NAVIGATING
        
        # Publish waypoints
        self.publish_waypoints()
        
        rospy.loginfo(f"Generated waypoint sequence with {len(self.waypoint_sequence)} waypoints")
    
    def generate_waypoint_pattern(self, pattern_type, parameters):
        """Generate waypoints for a specific pattern"""
        if pattern_type in self.waypoint_patterns:
            return self.waypoint_patterns[pattern_type](parameters)
        else:
            rospy.logwarn(f"Unknown pattern type: {pattern_type}")
            return []
    
    def generate_spiral_pattern(self, params):
        """Generate spiral pattern waypoints"""
        waypoints = []
        radius_start = params.get('radius_start', 10.0)
        radius_end = params.get('radius_end', 50.0)
        spiral_rings = params.get('spiral_rings', 3)
        height = params.get('height', 10.0)
        
        angle_step = math.pi / 8  # 22.5 degrees
        points_per_ring = 16
        
        for ring in range(spiral_rings):
            radius = radius_start + (radius_end - radius_start) * ring / spiral_rings
            
            for point in range(points_per_ring):
                angle = point * angle_step
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                
                waypoints.append((x, y, height))
        
        return waypoints
    
    def generate_lawnmower_pattern(self, params):
        """Generate lawnmower pattern waypoints"""
        waypoints = []
        grid_size = params.get('grid_size', 5)
        spacing = params.get('spacing', 10.0)
        height = params.get('height', 10.0)
        
        # Generate grid points
        for row in range(grid_size):
            for col in range(grid_size):
                x = col * spacing
                y = row * spacing
                
                if row % 2 == 1:  # Alternate direction
                    x = x + spacing / 2
                
                waypoints.append((x, y, height))
        
        return waypoints
    
    def generate_circle_pattern(self, params):
        """Generate circle pattern waypoints"""
        waypoints = []
        radius = params.get('radius', 20.0)
        height = params.get('height', 10.0)
        points = params.get('points', 32)
        
        angle_step = 2 * math.pi / points
        
        for point in range(points):
            angle = point * angle_step
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            waypoints.append((x, y, height))
        
        return waypoints
    
    def generate_grid_pattern(self, params):
        """Generate grid pattern waypoints"""
        waypoints = []
        grid_size = params.get('grid_size', 5)
        spacing = params.get('spacing', 10.0)
        height = params.get('height', 10.0)
        
        for row in range(grid_size):
            for col in range(grid_size):
                x = col * spacing
                y = row * spacing
                
                waypoints.append((x, y, height))
        
        return waypoints
    
    def generate_target_approach_pattern(self, params):
        """Generate target approach pattern"""
        waypoints = []
        
        if self.target_position is not None:
            # Add approach waypoints to target
            target_x, target_y, target_z = self.target_position
            
            # Add several approach points
            approach_points = [
                (target_x - 20, target_y - 20, target_z + 5),
                (target_x - 10, target_y - 10, target_z + 2),
                (target_x, target_y, target_z)
            ]
            
            for point in approach_points:
                waypoints.append(point)
        
        return waypoints
    
    def publish_waypoints(self):
        """Publish current waypoint sequence"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for waypoint in self.waypoint_sequence:
            path_msg.poses.append(waypoint)
        
        self.path_pub.publish(path_msg)
        
        # Publish individual waypoints
        for i, waypoint in enumerate(self.waypoint_sequence):
            waypoint.pose.header.stamp = rospy.Time.now()
            self.waypoint_pub.publish(waypoint)
    
    def update_position_deviation(self):
        """Update position deviation from planned path"""
        if self.current_position is None or not self.waypoint_sequence:
            return
        
        current_waypoint = self.waypoint_sequence[self.current_waypoint_index]
        
        # Calculate deviation
        deviation = math.sqrt(
            (current_waypoint.pose.position.x - self.current_position[0])**2 +
            (current_waypoint.pose.position.y - self.current_position[1])**2
        )
        
        self.position_deviation = deviation
        self.waypoint_deviation_history.append(deviation)
        
        # Keep history manageable
        if len(self.waypoint_deviation_history) > 100:
            self.waypoint_deviation_history.pop(0)
    
    def check_off_track_condition(self):
        """Check if drone is off track from planned path"""
        if self.current_position is None or not self.waypoint_sequence:
            self.offtrack_detected = False
            return
        
        current_waypoint = self.waypoint_sequence[self.current_waypoint_index]
        distance = self.calculate_distance_to_waypoint()
        
        # Check distance threshold
        distance_threshold = self.max_waypoint_distance * 0.7
        if distance > distance_threshold:
            self.offtrack_detected = True
            return
        
        # Check compass deviation
        if len(self.compass_history) > 0:
            compass_deviation = abs(self.current_yaw - self.compass_history[-1])
            if compass_deviation > self.compass_deviation_threshold:
                self.offtrack_detected = True
                return
        
        # Update compass history
        self.compass_history.append(self.current_yaw)
        if len(self.compass_history) > 10:
            self.compass_history.pop(0)
        
        # Not off track
        self.offtrack_detected = False
    
    def calculate_distance_to_waypoint(self):
        """Calculate distance to current waypoint"""
        if self.current_position is None or not self.waypoint_sequence:
            return float('inf')
        
        current_waypoint = self.waypoint_sequence[self.current_waypoint_index]
        
        return math.sqrt(
            (current_waypoint.pose.position.x - self.current_position[0])**2 +
            (current_waypoint.pose.position.y - self.current_position[1])**2 +
            (current_waypoint.pose.position.z - self.current_position[2])**2
        )
    
    def publish_status(self, event):
        """Publish navigator status"""
        status_msg = String()
        status_msg.data = self.status.value
        
        # Add additional status information
        status_info = {
            'status': self.status.value,
            'current_waypoint': self.current_waypoint_index,
            'total_waypoints': len(self.waypoint_sequence),
            'position_deviation': self.position_deviation,
            'offtrack_detected': self.offtrack_detected,
            'offtrack_timer': self.offtrack_timer
        }
        
        status_msg.data = json.dumps(status_info)
        self.navigator_status_pub.publish(status_msg)
    
    def set_quaternion_from_yaw(self, pose, yaw):
        """Set quaternion orientation from yaw angle"""
        from tf.transformations import quaternion_from_euler
        
        quaternion = quaternion_from_euler(0, 0, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
    
    def extract_yaw_from_pose(self, pose):
        """Extract yaw angle from pose orientation"""
        from tf.transformations import euler_from_quaternion
        
        quaternion = [pose.orientation.x, pose.orientation.y, 
                     pose.orientation.z, pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return yaw

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass