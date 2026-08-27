#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from nav_msgs.msg import Odometry
from drone_control.msg import DetectedObjects, Target
from filterpy.kalman import KalmanFilter

class TargetTracker:
    def __init__(self):
        rospy.init_node('target_tracking_controller', anonymous=True)
        self.state = "SEARCHING"
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odometry_callback)
        self.det_sub = rospy.Subscriber('/detected_objects', DetectedObjects, self.detections_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.mavros_state_callback)
        
        # Publishers
        self.target_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
        self.cmd_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.position_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        # Services
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        # State variables
        self.drone_pose = None
        self.target_kf = None
        self.mavros_state = None
        self.engagement_distance = rospy.get_param('tracking/engagement_distance', 10.0)
        self.attack_distance = rospy.get_param('tracking/attack_distance', 2.0)
        self.offboard_enabled = False
        self.tracking_start_time = None
        
        self.rate = rospy.Rate(20)
        rospy.loginfo("Target Tracking Controller started in SEARCHING mode")
        
    def mavros_state_callback(self, msg):
        self.mavros_state = msg
        
    def odometry_callback(self, msg):
        self.drone_pose = msg.pose.pose
        
    def detections_callback(self, msg):
        if self.state in ["SEARCHING", "TRACKING"] and msg.objects:
            best = max(msg.objects, key=lambda o: o.confidence)
            if self.state == "SEARCHING":
                self.initialize_kalman_filter(best)
                self.state = "TRACKING"
                self.tracking_start_time = rospy.get_time()
                rospy.loginfo("Target acquired, switching to TRACKING")
                
    def enable_offboard_mode(self):
        """Switch to OFFBOARD mode and arm the drone"""
        if self.mavros_state is None:
            return False
            
        # Check if already in OFFBOARD mode
        if self.mavros_state.mode == "OFFBOARD":
            self.offboard_enabled = True
            return True
            
        # Switch to OFFBOARD mode
        rospy.loginfo("Switching to OFFBOARD mode...")
        try:
            mode_resp = self.set_mode_srv(0, "OFFBOARD")  # 0 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            if mode_resp.mode_sent:
                rospy.loginfo("OFFBOARD mode command sent")
                self.offboard_enabled = True
                return True
            else:
                rospy.logwarn("Failed to send OFFBOARD mode command")
                return False
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed: {e}")
            return False
            
    def arm_drone(self):
        """Arm the drone"""
        if self.mavros_state is None:
            return False
            
        if self.mavros_state.armed:
            return True
            
        rospy.loginfo("Arming drone...")
        try:
            arm_resp = self.arm_srv(True)
            if arm_resp.success:
                rospy.loginfo("Drone armed")
                return True
            else:
                rospy.logwarn("Failed to arm drone")
                return False
        except rospy.ServiceException as e:
            rospy.logwarn(f"Arming service call failed: {e}")
            return False
            
    def initialize_kalman_filter(self, detection):
        """Initialize Kalman filter for target tracking"""
        self.target_kf = KalmanFilter(dim_x=6, dim_z=3)
        dt = 0.1
        self.target_kf.F = np.array([[1,0,0,dt,0,0],
                                     [0,1,0,0,dt,0],
                                     [0,0,1,0,0,dt],
                                     [0,0,0,1,0,0],
                                     [0,0,0,0,1,0],
                                     [0,0,0,0,0,1]])
        self.target_kf.H = np.array([[1,0,0,0,0,0],
                                     [0,1,0,0,0,0],
                                     [0,0,1,0,0,0]])
        self.target_kf.P *= 10.0
        self.target_kf.R = np.eye(3) * 0.05
        self.target_kf.Q = np.eye(6) * 0.1
        
        # Set initial position 10m in front of drone
        if self.drone_pose:
            drone_x = self.drone_pose.position.x
            drone_y = self.drone_pose.position.y
            drone_z = self.drone_pose.position.z
            
            # Place target 10m ahead
            init_pos = np.array([drone_x, drone_y + 10.0, drone_z])
            self.target_kf.x = np.array([init_pos[0], init_pos[1], init_pos[2], 0, 0, 0])
            rospy.loginfo(f"Target initialized at: {init_pos}")
            
    def run(self):
        """Main control loop"""
        while not rospy.is_shutdown():
            if self.state == "TRACKING":
                self.track_target()
                # After 3 seconds of tracking, engage
                if self.tracking_start_time and rospy.get_time() - self.tracking_start_time > 3.0:
                    self.state = "ENGAGING"
                    rospy.loginfo("Switching to ENGAGING mode")
                    
            elif self.state == "ENGAGING":
                # Ensure OFFBOARD mode before engaging
                if not self.offboard_enabled:
                    self.enable_offboard_mode()
                    self.arm_drone()
                self.engage_target()
                
            elif self.state == "ATTACK":
                self.execute_attack()
                
            # Publish target pose
            if self.target_kf is not None:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = 'map'
                pose_msg.pose.position.x = self.target_kf.x[0]
                pose_msg.pose.position.y = self.target_kf.x[1]
                pose_msg.pose.position.z = self.target_kf.x[2]
                self.target_pub.publish(pose_msg)
                
            self.rate.sleep()
            
    def track_target(self):
        """Track target while in TRACKING mode"""
        self.target_kf.predict()
        # Simulate target staying at same position
        z = self.target_kf.x[:3]
        self.target_kf.update(z)
        
    def engage_target(self):
        """Move towards target"""
        if self.target_kf is None or self.drone_pose is None:
            return
            
        target_pos = self.target_kf.x[:3]
        drone_pos = np.array([self.drone_pose.position.x,
                              self.drone_pose.position.y,
                              self.drone_pose.position.z])
        error = target_pos - drone_pos
        dist = np.linalg.norm(error)
        
        rospy.loginfo(f"Distance to target: {dist:.2f}m")
        
        if dist < self.attack_distance:
            self.state = "ATTACK"
            rospy.loginfo("Target within attack distance, switching to ATTACK")
            return
            
        # Publish continuous setpoint for OFFBOARD mode
        self.publish_position_setpoint(target_pos)
        
        # Simple proportional velocity control
        max_speed = 5.0
        if dist > 0:
            vel = error / dist * min(max_speed, dist * 2.0)
        else:
            vel = np.zeros(3)
            
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = vel[0]
        cmd.linear.y = vel[1]
        cmd.linear.z = vel[2]
        self.cmd_pub.publish(cmd)
        
    def publish_position_setpoint(self, target_pos):
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
        
    def execute_attack(self):
        """Kamikaze attack - full speed towards target"""
        if self.target_kf is None or self.drone_pose is None:
            return
            
        target_pos = self.target_kf.x[:3]
        drone_pos = np.array([self.drone_pose.position.x,
                              self.drone_pose.position.y,
                              self.drone_pose.position.z])
        error = target_pos - drone_pos
        dist = np.linalg.norm(error)
        
        rospy.loginfo(f"ATTACK! Distance: {dist:.2f}m")
        
        if dist < 0.5:
            rospy.loginfo("Target destroyed!")
            rospy.signal_shutdown("Mission complete")
            return
            
        # Full speed ahead
        if dist > 0:
            vel = error / dist * 10.0
        else:
            vel = np.zeros(3)
            
        cmd = Twist()
        cmd.linear.x = vel[0]
        cmd.linear.y = vel[1]
        cmd.linear.z = vel[2]
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    tracker = TargetTracker()
    tracker.run()
