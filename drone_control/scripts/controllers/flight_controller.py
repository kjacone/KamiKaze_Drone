#!/usr/bin/env python3
"""
drone_control/scripts/controllers/flight_controller.py
Enhanced flight controller with PID and trajectory tracking
"""

import numpy as np
import rospy
from filterpy.common import Q_discrete_white_noise
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode

from drone_control import ControlLibrary
from drone_control import SafetyLibrary
from drone_control.msg  import ControlCommand, NodeHealth, SafetyStatus, TrackedTarget
from drone_control.utils import ErrorHandler


class PIDController:
    """Simple PID controller implementation"""
    
    def __init__(self, kp, ki, kd, integral_clamp=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_clamp = integral_clamp
        self.last_time = rospy.Time.now()
        
    def update(self, setpoint, measurement, dt):
        """Update PID controller"""
        error = setpoint - measurement
        
        # Proportional
        p_term = self.kp * error
        
        # Integral
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_clamp, self.integral_clamp)
        i_term = self.ki * self.integral
        
        # Derivative
        d_term = 0.0
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
            
        self.prev_error = error
        
        return p_term + i_term + d_term

class FlightController:
    """Enhanced flight controller with PID and trajectory tracking"""
    
    def __init__(self):
        rospy.init_node('flight_controller', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='flight_controller')
        self.control_lib = ControlLibrary()
        
        # Get parameters
        self.max_velocity = rospy.get_param('dynamics/constraints/max_velocity', 5.0)
        self.max_acceleration = rospy.get_param('dynamics/constraints/max_acceleration', 2.0)
        self.max_yaw_rate = rospy.get_param('dynamics/constraints/max_yaw_rate', 1.0)
        self.simulation_mode = rospy.get_param('/use_simulation', True)
        
        # Load PID gains
        self.pid_position_x = PIDController(
            rospy.get_param('pid/position/x/p', 1.0),
            rospy.get_param('pid/position/x/i', 0.01),
            rospy.get_param('pid/position/x/d', 0.1)
        )
        self.pid_position_y = PIDController(
            rospy.get_param('pid/position/y/p', 1.0),
            rospy.get_param('pid/position/y/i', 0.01),
            rospy.get_param('pid/position/y/d', 0.1)
        )
        self.pid_position_z = PIDController(
            rospy.get_param('pid/position/z/p', 1.2),
            rospy.get_param('pid/position/z/i', 0.02),
            rospy.get_param('pid/position/z/d', 0.15)
        )
        
        # State
        self.current_pose = None
        self.current_twist = None
        self.target_pose = None
        self.mavros_state = None
        self.offboard_enabled = False
        self.command_timeout = 2.0
        self.last_command_time = rospy.Time.now()
        
        # Subscribers
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_callback)
        rospy.Subscriber('/mavros/local_position/velocity', Twist, self._twist_callback)
        rospy.Subscriber('/mavros/state', State, self._state_callback)
        rospy.Subscriber('/drone_control/control_command', ControlCommand, self._command_callback)
        
        # Publishers
        self.target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.health_pub = rospy.Publisher('/flight_controller/node_health', NodeHealth, queue_size=10)
        
        # Services
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        # Control timer
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self._control_loop)
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)
        
        rospy.loginfo("Flight Controller initialized")
        
    def _pose_callback(self, msg):
        self.current_pose = msg.pose
        
    def _twist_callback(self, msg):
        self.current_twist = msg
        
    def _state_callback(self, msg):
        self.mavros_state = msg
        
    def _command_callback(self, msg):
        """Handle control commands"""
        self.last_command_time = rospy.Time.now()
        
        if msg.command_type == "position":
            self.target_pose = msg.pose
            self._enable_offboard_mode()
            self._arm_drone()
        elif msg.command_type == "velocity":
            self._send_velocity_command(msg.twist)
        elif msg.command_type == "stop":
            self._send_stop_command()
        elif msg.command_type == "land":
            self._send_land_command()
            
    def _enable_offboard_mode(self):
        """Switch to OFFBOARD mode"""
        if self.mavros_state is None:
            return False
            
        if self.mavros_state.mode == "OFFBOARD":
            self.offboard_enabled = True
            return True
            
        try:
            mode_resp = self.set_mode_srv(0, "OFFBOARD")
            if mode_resp.mode_sent:
                rospy.loginfo("OFFBOARD mode enabled")
                self.offboard_enabled = True
                return True
        except rospy.ServiceException as e:
            self.error_handler.handle_error(e, "OFFBOARD mode failed")
            
        return False
        
    def _arm_drone(self):
        """Arm the drone"""
        if self.mavros_state is None or self.mavros_state.armed:
            return True
            
        try:
            arm_resp = self.arm_srv(True)
            if arm_resp.success:
                rospy.loginfo("Drone armed")
                return True
        except rospy.ServiceException as e:
            self.error_handler.handle_error(e, "Arming failed")
            
        return False
        
    def _control_loop(self, event):
        """Main control loop"""
        if self.target_pose is not None and self.current_pose is not None:
            # Calculate control outputs
            setpoint = self.target_pose.position
            measurement = self.current_pose.position
            
            dt = 0.02  # 50Hz
            
            # Position PID control
            x_vel = self.pid_position_x.update(setpoint.x, measurement.x, dt)
            y_vel = self.pid_position_y.update(setpoint.y, measurement.y, dt)
            z_vel = self.pid_position_z.update(setpoint.z, measurement.z, dt)
            
            # Limit velocity
            velocity = np.array([x_vel, y_vel, z_vel])
            speed = np.linalg.norm(velocity)
            if speed > self.max_velocity:
                velocity = velocity / speed * self.max_velocity
                
            # Send position setpoint
            self._send_position_setpoint(
                setpoint.x, setpoint.y, setpoint.z,
                velocity[0], velocity[1], velocity[2]
            )
            
    def _send_position_setpoint(self, x, y, z, vx=0, vy=0, vz=0):
        """Send position setpoint to PX4"""
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = "map"
        setpoint.coordinate_frame = 1  # MAV_FRAME_LOCAL_NED
        setpoint.type_mask = 0b0000111111111000  # Position + velocity + yaw
        setpoint.position.x = x
        setpoint.position.y = y
        setpoint.position.z = z
        setpoint.velocity.x = vx
        setpoint.velocity.y = vy
        setpoint.velocity.z = vz
        setpoint.yaw = 0.0
        self.target_pub.publish(setpoint)
        
    def _send_velocity_command(self, twist):
        """Send velocity command"""
        # Limit velocity
        speed = np.linalg.norm([twist.linear.x, twist.linear.y, twist.linear.z])
        if speed > self.max_velocity:
            twist.linear.x = twist.linear.x / speed * self.max_velocity
            twist.linear.y = twist.linear.y / speed * self.max_velocity
            twist.linear.z = twist.linear.z / speed * self.max_velocity
            
        self.vel_pub.publish(twist)
        
    def _send_stop_command(self):
        """Send stop command"""
        stop_twist = Twist()
        self.vel_pub.publish(stop_twist)
        self.target_pose = None
        
    def _send_land_command(self):
        """Send land command"""
        try:
            self.set_mode_srv(0, "LAND")
            rospy.loginfo("Land command sent")
        except rospy.ServiceException as e:
            self.error_handler.handle_error(e, "Land command failed")
            
    def _publish_health(self, event):
        """Publish node health"""
        health_msg = NodeHealth()
        health_msg.node_name = 'flight_controller'
        health_msg.status = 'running'
        health_msg.timestamp = rospy.Time.now()
        health_msg.is_healthy = self.mavros_state is not None and self.mavros_state.connected
          # Add CPU and memory usage
        import psutil
        health_msg.cpu_usage = psutil.cpu_percent()
        health_msg.memory_usage = psutil.virtual_memory().percent

        self.health_pub.publish(health_msg)

if __name__ == '__main__':
    try:
        controller = FlightController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass