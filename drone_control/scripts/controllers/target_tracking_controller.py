#!/usr/bin/env python3
"""
drone_control/scripts/controllers/target_tracking_controller.py
Enhanced target tracking with mission manager integration
"""

import time

import numpy as np
import psutil
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry

from drone_control.utils import ErrorHandler

from drone_control.msg import (
    Command,
    CommandResponse,
    DetectedObjects,
    MissionStatus,
    NodeHealth,
    TrackedTarget,
    TrackedTargets,
)

try:
    from filterpy.kalman import KalmanFilter
except ImportError:
    # Fallback simple Kalman filter
    class KalmanFilter:
        def __init__(self, dim_x, dim_z):
            self.dim_x = dim_x
            self.dim_z = dim_z
            self.x = np.zeros(dim_x)
            self.P = np.eye(dim_x)
            self.F = np.eye(dim_x)
            self.H = np.eye(dim_z, dim_x)
            self.R = np.eye(dim_z)
            self.Q = np.eye(dim_x)
            
        def predict(self):
            self.x = self.F @ self.x
            self.P = self.F @ self.P @ self.F.T + self.Q
            
        def update(self, z):
            y = z - self.H @ self.x
            S = self.H @ self.P @ self.H.T + self.R
            K = self.P @ self.H.T @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.P = (np.eye(self.dim_x) - K @ self.H) @ self.P
      
      
class TargetTrackingController:
    """Enhanced target tracking with mission integration"""
    
    def __init__(self):
        rospy.init_node('target_tracking_controller', anonymous=False)
        
        # Setup error handling
        self.error_handler = ErrorHandler(node_name='target_tracking_controller')
        
        # State machine
        self.state = "SEARCHING"  # SEARCHING, TRACKING, ENGAGING, ATTACK, EMERGENCY, PAUSED
        self.previous_state = None
        self.state_start_time = time.time()
        
        self.error_count = 0
        
        # Get parameters
        self.engagement_distance = rospy.get_param('tracking/engagement_distance', 10.0)
        self.attack_distance = rospy.get_param('tracking/attack_distance', 2.0)
        self.disengagement_distance = rospy.get_param('tracking/disengagement_distance', 15.0)
        self.max_velocity = rospy.get_param('dynamics/constraints/max_velocity', 5.0)
        self.max_acceleration = rospy.get_param('dynamics/constraints/max_acceleration', 2.0)
        self.simulation_mode = rospy.get_param('/use_simulation', True)
        self.debug_mode = rospy.get_param('/debug_mode', False)
        self.test_mode = rospy.get_param('/test_mode', False)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odometry_callback)
        self.det_sub = rospy.Subscriber('/detected_objects', DetectedObjects, self.detections_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.mavros_state_callback)
        self.mission_sub = rospy.Subscriber('/mission_status', MissionStatus, self._handle_mission_status)
        
        # Publishers
        self.target_pub = rospy.Publisher('/tracked_targets', TrackedTargets, queue_size=10)
        self.cmd_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.position_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.health_pub = rospy.Publisher('/target_tracking_controller/node_health', NodeHealth, queue_size=10)
        
        # Service proxies (may be None if unavailable) – non‑blocking wait
        self.set_mode_srv = None
        self.arm_srv = None

        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=2.0)
            self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            rospy.loginfo("MAVROS set_mode service available")
        except rospy.ROSException as e:
            rospy.logwarn("MAVROS set_mode service not available: %s", e)

        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=2.0)
            self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            rospy.loginfo("MAVROS arming service available")
        except rospy.ROSException as e:
            rospy.logwarn("MAVROS arming service not available: %s", e)
        
        # State variables
        self.drone_pose = None
        self.target_kf = None
        self.mavros_state = None
        self.offboard_enabled = False
        self.tracking_start_time = None
        self.last_control_time = time.time()
        self.target_id = 0
        self.tracked_targets = []
        
        # Health monitoring
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)
        
        self.rate = rospy.Rate(20)
        rospy.loginfo(f"Target Tracking Controller started in {self.state} mode")
        
    def _handle_mission_status(self, msg):
        """Handle mission status updates"""
        rospy.logdebug(f"Mission state: {msg.state}")
        
        if msg.state == 'emergency':
            self._set_state("EMERGENCY")
        elif msg.state == 'paused':
            self._set_state("PAUSED")
        elif msg.state == 'completed' or msg.state == 'failed':
            self._set_state("SEARCHING")
            
    def _set_state(self, new_state):
        """Change state with logging"""
        if new_state != self.state:
            self.previous_state = self.state
            self.state = new_state
            self.state_start_time = time.time()
            rospy.loginfo(f"State changed: {self.previous_state} -> {self.state}")
            
    def mavros_state_callback(self, msg):
        self.mavros_state = msg
        
    def odometry_callback(self, msg):
        self.drone_pose = msg.pose.pose
        
    def detections_callback(self, msg):
        """Process detections from YOLO"""
        if self.state in ["SEARCHING", "TRACKING", "ENGAGING"] and msg.objects:
            # Find best detection
            best = max(msg.objects, key=lambda o: o.confidence)
            
            if self.state == "SEARCHING":
                self.initialize_kalman_filter(best)
                self._set_state("TRACKING")
                self.tracking_start_time = time.time()
                rospy.loginfo("Target acquired, switching to TRACKING")
            elif self.state == "TRACKING":
                # Update Kalman filter
                self._update_target_position(best)
                
    def initialize_kalman_filter(self, detection):
        """Initialize Kalman filter for target tracking"""
        self.target_kf = KalmanFilter(dim_x=6, dim_z=3)
        dt = 0.1
        
        # State transition matrix
        self.target_kf.F = np.array([
            [1,0,0,dt,0,0],
            [0,1,0,0,dt,0],
            [0,0,1,0,0,dt],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]
        ])
        
        # Measurement matrix
        self.target_kf.H = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0]
        ])
        
        # Initial covariance
        self.target_kf.P *= 10.0
        
        # Measurement noise
        self.target_kf.R = np.eye(3) * rospy.get_param('tracking/kalman/measurement_noise', 0.05)
        
        # Process noise
        self.target_kf.Q = np.eye(6) * rospy.get_param('tracking/kalman/process_noise', 0.1)
        
        # Set initial position in front of drone
        if self.drone_pose:
            drone_pos = np.array([
                self.drone_pose.position.x,
                self.drone_pose.position.y,
                self.drone_pose.position.z
            ])
            
            # Calculate direction vector from detection
            init_pos = drone_pos + np.array([0, self.engagement_distance * 0.5, 0])
            self.target_kf.x = np.array([init_pos[0], init_pos[1], init_pos[2], 0, 0, 0])
            rospy.loginfo(f"Target initialized at: {init_pos}")
            
        self.target_id += 1
        
    def _update_target_position(self, detection):
        """Update Kalman filter with new detection"""
        if self.target_kf is None:
            return
            
        # Predict
        self.target_kf.predict()
        
        # Update with measurement
        # For now, use current position (will be improved with actual pixel-to-world conversion)
        z = self.target_kf.x[:3]
        self.target_kf.update(z)
        
    def run(self):
        """Main control loop"""
        while not rospy.is_shutdown():
            try:
                if self.state == "SEARCHING":
                    self._search_mode()
                elif self.state == "TRACKING":
                    self._track_mode()
                elif self.state == "ENGAGING":
                    self._engage_mode()
                elif self.state == "ATTACK":
                    self._attack_mode()
                elif self.state == "EMERGENCY":
                    self._emergency_mode()
                elif self.state == "PAUSED":
                    self._paused_mode()
                    
                # Publish tracked targets
                self._publish_tracked_targets()
                
            except Exception as e:
                self.error_handler.handle_error(e, "Main loop error")
                rospy.logwarn(f"Error in control loop: {e}")
                
            self.rate.sleep()
            
    def _search_mode(self):
        """Search for targets"""
        # Hover in place or follow search pattern
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        self.cmd_pub.publish(cmd)
        
    def _track_mode(self):
        """Track target"""
        if self.target_kf is None:
            self._set_state("SEARCHING")
            return
            
        # Predict target position
        self.target_kf.predict()
        
        # Calculate distance to target
        if self.drone_pose:
            target_pos = self.target_kf.x[:3]
            drone_pos = np.array([
                self.drone_pose.position.x,
                self.drone_pose.position.y,
                self.drone_pose.position.z
            ])
            distance = np.linalg.norm(target_pos - drone_pos)
            
            # Check if target is lost
            if distance > self.disengagement_distance:
                self._set_state("SEARCHING")
                rospy.logwarn("Target lost, returning to search")
                return
                
            # Check if within engagement distance
            if distance < self.attack_distance:
                self._set_state("ENGAGING")
                rospy.loginfo("Target within engagement distance")
                return
                
            # Move towards target
            self._move_towards_target(target_pos)
            
    def _engage_mode(self):
        """Engage target - move closer"""
        if self.target_kf is None or self.drone_pose is None:
            self._set_state("SEARCHING")
            return
            
        # Ensure OFFBOARD mode
        if not self.offboard_enabled:
            self._enable_offboard_mode()
            
        target_pos = self.target_kf.x[:3]
        drone_pos = np.array([
            self.drone_pose.position.x,
            self.drone_pose.position.y,
            self.drone_pose.position.z
        ])
        distance = np.linalg.norm(target_pos - drone_pos)
        
        rospy.loginfo_throttle(1.0, f"Engaging target, distance: {distance:.2f}m")
        
        if distance < self.attack_distance * 0.5:
            self._set_state("ATTACK")
            rospy.loginfo("Attack position reached!")
            return
            
        self._move_towards_target(target_pos)
        
    def _attack_mode(self):
        """Execute attack - full speed towards target"""
        if self.target_kf is None or self.drone_pose is None:
            self._set_state("SEARCHING")
            return
            
        target_pos = self.target_kf.x[:3]
        drone_pos = np.array([
            self.drone_pose.position.x,
            self.drone_pose.position.y,
            self.drone_pose.position.z
        ])
        distance = np.linalg.norm(target_pos - drone_pos)
        
        rospy.loginfo_throttle(0.5, f"ATTACK! Distance: {distance:.2f}m")
        
        if distance < 0.5:
            rospy.loginfo("Target destroyed!")
            self._set_state("SEARCHING")
            # Publish mission complete
            self._send_mission_complete()
            return
            
        # Full speed ahead
        if distance > 0:
            vel = (target_pos - drone_pos) / distance * self.max_velocity * 2.0
        else:
            vel = np.zeros(3)
            
        cmd = Twist()
        cmd.linear.x = vel[0]
        cmd.linear.y = vel[1]
        cmd.linear.z = vel[2]
        self.cmd_pub.publish(cmd)
        
    def _emergency_mode(self):
        """Emergency procedures"""
        rospy.logwarn_throttle(1.0, "EMERGENCY mode active")
        # Hover in place
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        self.cmd_pub.publish(cmd)
        
    def _paused_mode(self):
        """Paused state"""
        # Hover in place
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        self.cmd_pub.publish(cmd)
        
    def _move_towards_target(self, target_pos):
        """Move drone towards target position"""
        if self.drone_pose is None:
            return
            
        drone_pos = np.array([
            self.drone_pose.position.x,
            self.drone_pose.position.y,
            self.drone_pose.position.z
        ])
        
        error = target_pos - drone_pos
        distance = np.linalg.norm(error)
        
        # Publish position setpoint for OFFBOARD
        self._publish_position_setpoint(target_pos)
        
        # Velocity control
        if distance > 0:
            speed = min(self.max_velocity, distance * 0.5)
            vel = error / distance * speed
        else:
            vel = np.zeros(3)
            
        cmd = Twist()
        cmd.linear.x = vel[0]
        cmd.linear.y = vel[1]
        cmd.linear.z = vel[2]
        self.cmd_pub.publish(cmd)
        
    def _publish_position_setpoint(self, target_pos):
        """Publish position setpoint for OFFBOARD mode"""
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = "map"
        setpoint.coordinate_frame = 1  # MAV_FRAME_LOCAL_NED
        setpoint.type_mask = 0b0000111111111000  # Position + velocity + yaw
        setpoint.position.x = target_pos[0]
        setpoint.position.y = target_pos[1]
        setpoint.position.z = target_pos[2]
        setpoint.yaw = 0.0
        self.position_target_pub.publish(setpoint)
        
    def _enable_offboard_mode(self):
        """Switch to OFFBOARD mode and arm"""
        if self.mavros_state is None:
            return False

        if self.mavros_state.mode == "OFFBOARD" and self.mavros_state.armed:
            self.offboard_enabled = True
            return True

        if self.set_mode_srv is None or self.arm_srv is None:
            rospy.logwarn("MAVROS services not available, cannot enable OFFBOARD")
            return False

        try:
            mode_resp = self.set_mode_srv(0, "OFFBOARD")
            if mode_resp.mode_sent:
                rospy.loginfo("OFFBOARD mode enabled")
                self.offboard_enabled = True
                arm_resp = self.arm_srv(True)
                if arm_resp.success:
                    rospy.loginfo("Drone armed")
                return True
            else:
                rospy.logwarn("Failed to enable OFFBOARD mode")
                return False
        except rospy.ServiceException as e:
            self.error_handler.handle_error(e, "OFFBOARD mode failed")
            return False
            
    def _publish_tracked_targets(self):
        """Publish tracked targets"""
        if self.target_kf is None:
            return
            
        targets_msg = TrackedTargets()
        targets_msg.header.stamp = rospy.Time.now()
        targets_msg.header.frame_id = 'map'
        
        target = TrackedTarget()
        target.id = self.target_id
        target.position.x = self.target_kf.x[0]
        target.position.y = self.target_kf.x[1]
        target.position.z = self.target_kf.x[2]
        target.velocity.x = self.target_kf.x[3]
        target.velocity.y = self.target_kf.x[4]
        target.velocity.z = self.target_kf.x[5]
        target.confidence = 0.95  # Will be updated from actual detections
        target.state = self.state
        
        targets_msg.targets.append(target)
        self.target_pub.publish(targets_msg)
        
    def _send_mission_complete(self):
        """Send mission complete notification"""
        status_msg = MissionStatus()
        status_msg.mission_id = "attack_mission"
        status_msg.state = "completed"
        status_msg.elapsed_time = time.time() - self.state_start_time
        # Publish to mission manager
        
    def _publish_health(self, event):
        """Publish node health with all required fields"""
        try:
            health_msg = NodeHealth()
            health_msg.node_name = 'target_tracking_controller'
            health_msg.status = self.state
            health_msg.timestamp = rospy.Time.now()
            
            # Determine health status
            is_healthy = True
            
            # Check for emergency state
            if self.state == "EMERGENCY":
                is_healthy = False
                rospy.logwarn_throttle(5.0, "Node in EMERGENCY state")
                
            # Check if Kalman filter is initialized when tracking
            if self.state in ["TRACKING", "ENGAGING", "ATTACK"]:
                if self.target_kf is None:
                    is_healthy = False
                    rospy.logwarn_throttle(5.0, "Kalman filter not initialized")
                elif self.drone_pose is None:
                    is_healthy = False
                    rospy.logwarn_throttle(5.0, "No drone pose available")
                    
            # Check for MAVROS connection
            if self.mavros_state is not None and not self.mavros_state.connected:
                is_healthy = False
                rospy.logwarn_throttle(5.0, "MAVROS disconnected")
                
            health_msg.is_healthy = is_healthy
            
            # ALL REQUIRED FIELDS
            health_msg.detection_count = len(self.tracked_targets) if hasattr(self, 'tracked_targets') else 0
            health_msg.error_count = getattr(self, 'error_count', 0)
            health_msg.fps = 20.0  # Control loop runs at 20Hz
            health_msg.cpu_usage = psutil.cpu_percent()
            health_msg.memory_usage = psutil.virtual_memory().percent
            
            self.health_pub.publish(health_msg)
            
        except Exception as e:
            rospy.logwarn(f"Error publishing health: {e}")

if __name__ == '__main__':
    tracker = TargetTrackingController()
    tracker.run()