#!/usr/bin/env python3
"""
drone_control/scripts/controllers/trajectory_planner.py
Advanced trajectory planning with A* algorithm and dynamic obstacle avoidance
"""

import rospy
import numpy as np
import heapq
from enum import Enum
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from drone_control.msg import TrackedTarget
from std_msgs.msg import String
import sys
import os
import math
from typing import List, Tuple, Optional, Dict, Set

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

class NodeType(Enum):
    FREE = 0
    OBSTACLE = 1
    TARGET = 2

class TrajectoryPlanner:
    """Advanced trajectory planning with A* algorithm and dynamic obstacle avoidance"""
    
    def __init__(self):
        rospy.init_node('trajectory_planner', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='trajectory_planner')
        self.control_lib = ControlLibrary()
        
        # Configuration parameters
        self.map_resolution = rospy.get_param('~map_resolution', 0.5)
        self.planning_horizon = rospy.get_param('~planning_horizon', 30.0)
        self.replanning_frequency = rospy.get_param('~replanning_frequency', 10.0)
        self.max_velocity = rospy.get_param('~max_velocity', 5.0)
        self.max_acceleration = rospy.get_param('~max_acceleration', 2.0)
        self.safety_margin = rospy.get_param('~safety_margin', 1.0)
        self.simulation_mode = rospy.get_param('/use_simulation', True)
        
        # State variables
        self.current_position = None
        self.current_velocity = None
        self.target_position = None
        self.waypoint_sequence = []
        self.current_waypoint_index = 0
        self.map_grid = None
        self.grid_size = None
        self.last_replan_time = 0.0
        self.emergency_stop = False
        self.obstacle_positions = []
        self.collision_warnings = []
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odometry_callback)
        self.tracked_targets_sub = rospy.Subscriber('/tracked_targets', TrackedTargets, self.tracked_targets_callback)
        self.mission_sub = rospy.Subscriber('/mission_status', String, self.mission_callback)
        
        # Publishers
        self.trajectory_pub = rospy.Publisher('/planned_trajectory', PoseStamped, queue_size=10)
        self.waypoint_pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('/path', PoseStamped, queue_size=10)
        self.planner_status_pub = rospy.Publisher('/planner_status', String, queue_size=10)
        
        # Timers
        self.planning_timer = rospy.Timer(rospy.Duration(1.0 / self.replanning_frequency), self.planning_loop)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        # Initialize map
        self.initialize_map()
        
        rospy.loginfo("Trajectory Planner initialized")
    
    def initialize_map(self):
        """Initialize planning grid based on workspace bounds"""
        # Workspace bounds (meters)
        self.map_bounds = {
            'min_x': -100.0, 'max_x': 100.0,
            'min_y': -100.0, 'max_y': 100.0,
            'min_z': 0.0, 'max_z': 50.0
        }
        
        # Calculate grid dimensions
        self.grid_size = {
            'width': int((self.map_bounds['max_x'] - self.map_bounds['min_x']) / self.map_resolution) + 1,
            'height': int((self.map_bounds['max_y'] - self.map_bounds['min_y']) / self.map_resolution) + 1,
            'depth': int((self.map_bounds['max_z'] - self.map_bounds['min_z']) / self.map_resolution) + 1
        }
        
        # Initialize grid
        self.map_grid = np.zeros((self.grid_size['depth'], 
                                 self.grid_size['height'], 
                                 self.grid_size['width']), dtype=np.int8)
        
        # Mark static obstacles (geofence boundaries)
        self.mark_obstacle_volume(
            self.map_bounds['min_x'], self.map_bounds['max_x'],
            self.map_bounds['min_y'], self.map_bounds['max_y'],
            self.map_bounds['min_z'], self.map_bounds['max_z'],
            1  # obstacle type
        )
    
    def mark_obstacle_volume(self, x_min, x_max, y_min, y_max, z_min, z_max, obstacle_type):
        """Mark a volume as obstacle in the grid"""
        # Convert world coordinates to grid indices
        x_min_idx = int((x_min - self.map_bounds['min_x']) / self.map_resolution)
        x_max_idx = int((x_max - self.map_bounds['min_x']) / self.map_resolution)
        y_min_idx = int((y_min - self.map_bounds['min_y']) / self.map_resolution)
        y_max_idx = int((y_max - self.map_bounds['min_y']) / self.map_resolution)
        z_min_idx = int((z_min - self.map_bounds['min_z']) / self.map_resolution)
        z_max_idx = int((z_max - self.map_bounds['min_z']) / self.map_resolution)
        
        # Clamp to grid bounds
        x_min_idx = max(0, min(x_min_idx, self.grid_size['width'] - 1))
        x_max_idx = max(0, min(x_max_idx, self.grid_size['width'] - 1))
        y_min_idx = max(0, min(y_min_idx, self.grid_size['height'] - 1))
        y_max_idx = max(0, min(y_max_idx, self.grid_size['height'] - 1))
        z_min_idx = max(0, min(z_min_idx, self.grid_size['depth'] - 1))
        z_max_idx = max(0, min(z_max_idx, self.grid_size['depth'] - 1))
        
        # Mark obstacle cells
        for z in range(z_min_idx, z_max_idx + 1):
            for y in range(y_min_idx, y_max_idx + 1):
                for x in range(x_min_idx, x_max_idx + 1):
                    self.map_grid[z, y, x] = obstacle_type
    
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
        
        # Check for emergency stop
        if self.emergency_stop:
            self.trigger_emergency_stop()
    
    def tracked_targets_callback(self, msg):
        """Update target positions from tracked targets"""
        if msg.targets:
            # Get the closest target
            closest_target = None
            min_distance = float('inf')
            
            for target in msg.targets:
                distance = np.linalg.norm([
                    target.position.x - self.current_position[0],
                    target.position.y - self.current_position[1],
                    target.position.z - self.current_position[2]
                ])
                
                if distance < min_distance:
                    min_distance = distance
                    closest_target = target
            
            if closest_target:
                self.target_position = np.array([
                    closest_target.position.x,
                    closest_target.position.y,
                    closest_target.position.z
                ])
                rospy.logdebug(f"New target position: {self.target_position}, distance: {min_distance:.2f}m")
    
    def mission_callback(self, msg):
        """Handle mission state changes"""
        if msg.data == 'emergency':
            self.emergency_stop = True
        elif msg.data in ['completed', 'failed']:
            self.emergency_stop = False
            self.waypoint_sequence = []
            self.current_waypoint_index = 0
    
    def planning_loop(self, event):
        """Main planning loop"""
        current_time = rospy.Time.now().to_sec()
        
        # Check if we need to replan
        if current_time - self.last_replan_time > (1.0 / self.replanning_frequency):
            if self.current_position is not None and self.target_position is not None:
                self.plan_trajectory()
                self.last_replan_time = current_time
    
    def plan_trajectory(self):
        """Plan trajectory using A* algorithm"""
        if self.emergency_stop:
            return
        
        # Convert current and target positions to grid indices
        current_idx = self.world_to_grid(self.current_position)
        target_idx = self.world_to_grid(self.target_position)
        
        if current_idx is None or target_idx is None:
            rospy.logwarn("Cannot plan trajectory - invalid positions")
            return
        
        # Check if target is reachable
        if self.map_grid[target_idx[0], target_idx[1], target_idx[2]] == NodeType.OBSTACLE:
            rospy.logwarn("Target position is in obstacle - using alternative approach")
            # Find alternative target position
            target_idx = self.find_nearest_free_cell(target_idx)
            if target_idx is None:
                rospy.logerr("No free cells found for target")
                return
        
        # Check for collision with current obstacles
        if self.check_collision(current_idx):
            rospy.logwarn("Collision detected with current obstacles")
            # Try to move to a safer position first
            safe_position = self.find_safe_position()
            if safe_position is not None:
                current_idx = self.world_to_grid(safe_position)
        
        # Run A* algorithm
        path = self.a_star_planning(current_idx, target_idx)
        
        if path is not None:
            # Convert path back to world coordinates
            world_path = [self.grid_to_world(idx) for idx in path]
            
            # Filter path for safety
            safe_path = self.filter_path_for_safety(world_path)
            
            if safe_path:
                # Generate waypoints
                self.generate_waypoints(safe_path)
                rospy.loginfo(f"Trajectory planned with {len(safe_path)} waypoints")
            else:
                rospy.logwarn("Generated path failed safety checks")
        else:
            rospy.logwarn("A* planning failed - no path found")
            self.collision_warnings.append("No path found to target")
    
    def a_star_planning(self, start_idx, goal_idx):
        """A* algorithm for path planning"""
        # Initialize open and closed sets
        open_set = []
        closed_set = set()
        
        # Add start node
        heapq.heappush(open_set, (0, start_idx))
        
        # Track costs
        g_score = {start_idx: 0}
        f_score = {start_idx: self.heuristic_cost(start_idx, goal_idx)}
        came_from = {start_idx: None}
        
        while open_set:
            # Get node with lowest f-score
            current_f, current = heapq.heappop(open_idx)
            
            if current == goal_idx:
                # Reconstruct path
                return self.reconstruct_path(came_from, current)
            
            closed_set.add(current)
            
            # Generate neighbors
            neighbors = self.get_neighbors(current)
            
            for neighbor in neighbors:
                if neighbor in closed_set:
                    continue
                
                # Calculate tentative g-score
                tentative_g = g_score[current] + self.distance_cost(current, neighbor)
                
                if tentative_g < g_score.get(neighbor, float('inf')):
                    # Update scores
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic_cost(neighbor, goal_idx)
                    
                    # Add to open set
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        return None
    
    def get_neighbors(self, node_idx):
        """Get valid neighboring cells in the grid"""
        neighbors = []
        z, y, x = node_idx
        
        # Generate 26-connected neighborhood (3D)
        for dz in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dz == 0 and dy == 0 and dx == 0:
                        continue
                    
                    nz, ny, nx = z + dz, y + dy, x + dx
                    
                    # Check bounds
                    if (0 <= nz < self.grid_size['depth'] and
                        0 <= ny < self.grid_size['height'] and
                        0 <= nx < self.grid_size['width']):
                        
                        # Check if cell is traversable
                        if self.map_grid[nz, ny, nx] != NodeType.OBSTACLE:
                            neighbors.append((nz, ny, nx))
        
        return neighbors
    
    def heuristic_cost(self, node1, node2):
        """Calculate heuristic cost (Euclidean distance)"""
        z1, y1, x1 = node1
        z2, y2, x2 = node2
        
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
    
    def distance_cost(self, node1, node2):
        """Calculate actual movement cost between nodes"""
        # 3D Euclidean distance scaled by resolution
        distance = math.sqrt((node1[2] - node2[2])**2 +  # x difference
                           (node1[1] - node2[1])**2 +  # y difference
                           (node1[0] - node2[0])**2)   # z difference
        
        return distance * self.map_resolution
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from A* algorithm"""
        path = [current]
        
        while current in came_from and came_from[current] is not None:
            current = came_from[current]
            path.append(current)
        
        # Reverse to get start-to-goal path
        path.reverse()
        return path
    
    def check_collision(self, position_idx):
        """Check if position is in collision with obstacles"""
        z, y, x = position_idx
        
        # Check if cell is occupied
        if self.map_grid[z, y, x] == NodeType.OBSTACLE:
            return True
        
        # Check surrounding cells for proximity to obstacles
        for dz in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    nz, ny, nx = z + dz, y + dy, x + dx
                    
                    if (0 <= nz < self.grid_size['depth'] and
                        0 <= ny < self.grid_size['height'] and
                        0 <= nx < self.grid_size['width']):
                        
                        if self.map_grid[nz, ny, nx] == NodeType.OBSTACLE:
                            # Check if within safety margin
                            distance = math.sqrt(dz**2 + dy**2 + dx**2)
                            if distance < self.safety_margin:
                                return True
        
        return False
    
    def generate_waypoints(self, path):
        """Generate waypoints from planned path"""
        self.waypoint_sequence = []
        self.current_waypoint_index = 0
        
        # Create PoseStamped messages for each waypoint
        for i, position in enumerate(path):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            pose.pose.position.z = position[2]
            
            # Set yaw to approach direction (except for last waypoint)
            if i < len(path) - 1:
                next_pos = path[i + 1]
                dyaw = math.atan2(next_pos[1] - position[1], 
                                 next_pos[0] - position[0])
                pose.pose.orientation = self.euler_to_quaternion(0, 0, dyaw)
            
            self.waypoint_sequence.append(pose)
        
        # Publish waypoints
        for waypoint in self.waypoint_sequence:
            self.waypoint_pub.publish(waypoint)
        
        # Publish trajectory
        if path:
            trajectory_pose = PoseStamped()
            trajectory_pose.header.stamp = rospy.Time.now()
            trajectory_pose.header.frame_id = "map"
            trajectory_pose.pose.position.x = path[0][0]
            trajectory_pose.pose.position.y = path[0][1]
            trajectory_pose.pose.position.z = path[0][2]
            self.trajectory_pub.publish(trajectory_pose)
        
        rospy.loginfo(f"Generated {len(self.waypoint_sequence)} waypoints")
    
    def filter_path_for_safety(self, path):
        """Filter path to ensure safety"""
        if not path:
            return None
        
        filtered_path = []
        
        for position in path:
            grid_idx = self.world_to_grid(position)
            if grid_idx is None:
                continue
            
            # Check collision
            if self.check_collision(grid_idx):
                rospy.logwarn(f"Waypoint {position} is in collision")
                continue
            
            # Check altitude constraints
            if not self.check_altitude_constraints(position):
                rospy.logwarn(f"Waypoint {position} violates altitude constraints")
                continue
            
            # Add to filtered path
            filtered_path.append(position)
        
        return filtered_path if filtered_path else None
    
    def check_altitude_constraints(self, position):
        """Check if position meets altitude constraints"""
        x, y, z = position
        
        # Check minimum altitude
        if z < self.map_bounds['min_z']:
            return False
        
        # Check maximum altitude
        if z > self.map_bounds['max_z']:
            return False
        
        return True
    
    def find_nearest_free_cell(self, target_idx):
        """Find nearest free cell to target position"""
        if target_idx is None:
            return None
        
        # BFS search for nearest free cell
        queue = [target_idx]
        visited = {target_idx}
        directions = [(dz, dy, dx) for dz in [-1, 0, 1] 
                      for dy in [-1, 0, 1] 
                      for dx in [-1, 0, 1] 
                      if not (dz == 0 and dy == 0 and dx == 0)]
        
        while queue:
            current = queue.pop(0)
            
            # Check if current cell is free
            if self.map_grid[current[0], current[1], current[2]] != NodeType.OBSTACLE:
                return current
            
            # Add neighbors to queue
            for dz, dy, dx in directions:
                neighbor = (current[0] + dz, current[1] + dy, current[2] + dx)
                
                if (0 <= neighbor[0] < self.grid_size['depth'] and
                    0 <= neighbor[1] < self.grid_size['height'] and
                    0 <= neighbor[2] < self.grid_size['width'] and
                    neighbor not in visited):
                    
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        return None
    
    def find_safe_position(self):
        """Find a safe position to move to"""
        # Search for safe position around current position
        if self.current_position is None:
            return None
        
        current_idx = self.world_to_grid(self.current_position)
        if current_idx is None:
            return None
        
        # Try positions around current position
        for dz in [-2, -1, 0, 1, 2]:
            for dy in [-2, -1, 0, 1, 2]:
                for dx in [-2, -1, 0, 1, 2]:
                    if dz == 0 and dy == 0 and dx == 0:
                        continue
                    
                    new_z = self.current_position[2] + dz * self.map_resolution
                    new_y = self.current_position[1] + dy * self.map_resolution
                    new_x = self.current_position[0] + dx * self.map_resolution
                    
                    new_position = (new_x, new_y, new_z)
                    new_idx = self.world_to_grid(new_position)
                    
                    if new_idx is not None and not self.check_collision(new_idx):
                        return new_position
        
        return None
    
    def trigger_emergency_stop(self):
        """Trigger emergency stop procedures"""
        rospy.logerr("Emergency stop triggered!")
        
        # Publish emergency stop signal
        status_msg = String()
        status_msg.data = "emergency_stop"
        self.planner_status_pub.publish(status_msg)
        
        # Clear current trajectory
        self.waypoint_sequence = []
        self.current_waypoint_index = 0
        self.target_position = None
    
    def publish_status(self, event):
        """Publish planner status"""
        status_msg = String()
        
        if self.emergency_stop:
            status_msg.data = "emergency"
        elif self.target_position is None:
            status_msg.data = "searching"
        elif len(self.waypoint_sequence) > 0:
            status_msg.data = f"following_waypoints_{self.current_waypoint_index}/{len(self.waypoint_sequence)}"
        else:
            status_msg.data = "planning"
        
        self.planner_status_pub.publish(status_msg)
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        from tf import transformations
        
        quaternion = transformations.quaternion_from_euler(roll, pitch, yaw)
        
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = quaternion[0]
        q.y = quaternion[1]
        q.z = quaternion[2]
        q.w = quaternion[3]
        
        return q
    
    def world_to_grid(self, position):
        """Convert world coordinates to grid indices"""
        if position is None:
            return None
        
        x, y, z = position
        
        # Check bounds
        if (x < self.map_bounds['min_x'] or x > self.map_bounds['max_x'] or
            y < self.map_bounds['min_y'] or y > self.map_bounds['max_y'] or
            z < self.map_bounds['min_z'] or z > self.map_bounds['max_z']):
            return None
        
        # Convert to grid indices
        idx_x = int((x - self.map_bounds['min_x']) / self.map_resolution)
        idx_y = int((y - self.map_bounds['min_y']) / self.map_resolution)
        idx_z = int((z - self.map_bounds['min_z']) / self.map_resolution)
        
        return (idx_z, idx_y, idx_x)
    
    def grid_to_world(self, grid_idx):
        """Convert grid indices to world coordinates"""
        z, y, x = grid_idx
        
        world_x = x * self.map_resolution + self.map_bounds['min_x']
        world_y = y * self.map_resolution + self.map_bounds['min_y']
        world_z = z * self.map_resolution + self.map_bounds['min_z']
        
        return (world_x, world_y, world_z)

if __name__ == '__main__':
    try:
        planner = TrajectoryPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass