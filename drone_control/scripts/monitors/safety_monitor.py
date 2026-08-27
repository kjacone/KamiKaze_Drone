#!/usr/bin/env python3
"""
drone_control/scripts/monitors/safety_monitor.py
Safety monitoring with geofence and emergency procedures
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from mavros_msgs.msg import State
from drone_control.msg import NodeHealth, Command, CommandResponse, SafetyStatus
from std_srvs.srv import Trigger, TriggerResponse
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from utils.error_handler import ErrorHandler
except ImportError:
    class ErrorHandler:
        def __init__(self, node_name):
            self.node_name = node_name
        def handle_error(self, error, context=None):
            rospy.logerr(f"[{self.node_name}] {error}: {context}")

class SafetyMonitor:
    """Safety monitoring with geofence and emergency procedures"""
    
    def __init__(self):
        rospy.init_node('safety_monitor', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='safety_monitor')
        
        # Safety parameters
        self.geofence_enabled = rospy.get_param('~enable_geofence', True)
        self.geofence_radius = rospy.get_param('~geofence_radius', 100.0)
        self.max_altitude = rospy.get_param('~max_altitude', 50.0)
        self.min_altitude = rospy.get_param('~min_altitude', 2.0)
        self.enable_emergency_landing = rospy.get_param('~enable_emergency_landing', True)
        self.simulation_mode = rospy.get_param('/use_simulation', True)
        self.test_mode = rospy.get_param('/test_mode', False)
        
        # State
        self.current_pose = None
        self.current_velocity = None
        self.is_safe = True
        self.safety_violations = []
        self.emergency_active = False
        
        # Subscribers
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_callback)
        rospy.Subscriber('/mavros/local_position/velocity', Twist, self._velocity_callback)
        rospy.Subscriber('/mavros/state', State, self._state_callback)
        
        # Publishers
        self.safety_pub = rospy.Publisher('/safety_status', SafetyStatus, queue_size=10)
        self.health_pub = rospy.Publisher('/node_health', NodeHealth, queue_size=10)
        self.emergency_pub = rospy.Publisher('/emergency_triggered', Bool, queue_size=10)
        
        # Services
        rospy.Service('/safety/geofence_enable', Trigger, self._enable_geofence)
        rospy.Service('/safety/geofence_disable', Trigger, self._disable_geofence)
        rospy.Service('/safety/check', Trigger, self._check_safety)
        rospy.Service('/safety/status', Trigger, self._get_status)
        
        # Timers
        self.check_timer = rospy.Timer(rospy.Duration(1.0), self._check_safety_timer)
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)
        
        rospy.loginfo("Safety Monitor initialized")
        
    def _pose_callback(self, msg):
        self.current_pose = msg.pose
        
    def _velocity_callback(self, msg):
        self.current_velocity = msg
        
    def _state_callback(self, msg):
        # Monitor for lost connection
        if not msg.connected:
            self._trigger_emergency("LOST_CONNECTION")
            
    def _check_safety_timer(self, event):
        """Periodic safety check"""
        if self.current_pose is None:
            return
            
        violations = []
        pos = self.current_pose.position
        
        # Check geofence
        if self.geofence_enabled:
            distance = np.linalg.norm([pos.x, pos.y])
            if distance > self.geofence_radius:
                violations.append(f"GEOFENCE_BREACH: {distance:.1f}m > {self.geofence_radius}m")
                
        # Check altitude
        if pos.z > self.max_altitude:
            violations.append(f"ALTITUDE_TOO_HIGH: {pos.z:.1f}m > {self.max_altitude}m")
        elif pos.z < self.min_altitude:
            violations.append(f"ALTITUDE_TOO_LOW: {pos.z:.1f}m < {self.min_altitude}m")
            
        # Check velocity
        if self.current_velocity:
            speed = np.linalg.norm([
                self.current_velocity.linear.x,
                self.current_velocity.linear.y,
                self.current_velocity.linear.z
            ])
            max_speed = rospy.get_param('dynamics/constraints/max_velocity', 5.0)
            if speed > max_speed * 1.5:
                violations.append(f"SPEED_EXCEEDED: {speed:.1f}m/s")
                
        # Handle violations
        if violations:
            self.is_safe = False
            for violation in violations:
                if violation not in self.safety_violations:
                    self.safety_violations.append(violation)
                    rospy.logwarn(f"Safety violation: {violation}")
                    
            # Trigger emergency for critical violations
            if any('GEOFENCE' in v or 'ALTITUDE' in v for v in violations):
                self._trigger_emergency(violations[0])
        else:
            self.is_safe = True
            self.safety_violations = []
            
        # Publish safety status
        status_msg = SafetyStatus()
        status_msg.is_safe = self.is_safe
        status_msg.violations = self.safety_violations
        status_msg.timestamp = rospy.Time.now()
        self.safety_pub.publish(status_msg)
        
    def _trigger_emergency(self, reason: str):
        """Trigger emergency procedures"""
        if not self.emergency_active:
            self.emergency_active = True
            rospy.logerr(f"EMERGENCY TRIGGERED: {reason}")
            
            # Publish emergency
            self.emergency_pub.publish(True)
            
            # Execute emergency procedures
            if self.enable_emergency_landing:
                self._emergency_land()
                
    def _emergency_land(self):
        """Execute emergency landing"""
        rospy.logwarn("Executing emergency landing...")
        # Publish emergency land command
        # This would be handled by the flight controller
        
    def _enable_geofence(self, req):
        """Enable geofence"""
        self.geofence_enabled = True
        return TriggerResponse(success=True, message="Geofence enabled")
        
    def _disable_geofence(self, req):
        """Disable geofence"""
        self.geofence_enabled = False
        return TriggerResponse(success=True, message="Geofence disabled")
        
    def _check_safety(self, req):
        """Check safety status"""
        return TriggerResponse(
            success=self.is_safe,
            message=f"Safety status: {'SAFE' if self.is_safe else 'UNSAFE'}"
        )
        
    def _get_status(self, req):
        """Get detailed safety status"""
        status = {
            'is_safe': self.is_safe,
            'violations': self.safety_violations,
            'geofence_enabled': self.geofence_enabled,
            'emergency_active': self.emergency_active,
            'pose': {
                'x': self.current_pose.position.x if self.current_pose else 0,
                'y': self.current_pose.position.y if self.current_pose else 0,
                'z': self.current_pose.position.z if self.current_pose else 0
            } if self.current_pose else {}
        }
        return TriggerResponse(
            success=True,
            message=str(status)
        )
        
    def _publish_health(self, event):
        """Publish node health"""
        health_msg = NodeHealth()
        health_msg.node_name = 'safety_monitor'
        health_msg.status = 'running' if self.is_safe else 'emergency'
        health_msg.timestamp = rospy.Time.now()
        health_msg.is_healthy = self.is_safe
        self.health_pub.publish(health_msg)

if __name__ == '__main__':
    try:
        monitor = SafetyMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass