#!/usr/bin/env python3
"""
drone_control/scripts/monitors/safety_monitor.py
Safety monitoring with geofence and emergency procedures - Enhanced with metrics
"""

import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from drone_control.utils import ErrorHandler
from drone_control.utils.correlation import get_or_create_mission_id
from drone_control.msg import Command, CommandResponse, NodeHealth, SafetyStatus


class SafetyMonitor:
    """Safety monitoring with geofence and emergency procedures - Enhanced with metrics"""
    
    def __init__(self):
        rospy.init_node('safety_monitor', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='safety_monitor')
        
        # ------------------------------------------------------------------
        # 1. Lightweight state that needs no params / no heavy imports
        # ------------------------------------------------------------------
        self.current_pose = None
        self.current_velocity = None
        self.is_safe = True
        self.is_healthy = True
        self.safety_violations = []
        self.emergency_active = False
        self.error_count = 0
        self.last_health_publish = time.time()
        self.current_mission_id = get_or_create_mission_id()
        self.violation_counts = {}
        self.last_check_time = time.time()
        self.total_checks = 0
        
        # Place-holders – filled after pub/sub/svc are up
        self.geofence_enabled = True
        self.geofence_radius = 100.0
        self.max_altitude = 50.0
        self.min_altitude = 2.0
        self.enable_emergency_landing = True
        self.simulation_mode = True
        self.test_mode = False
        
        # Metrics will be created later
        self.metric_violations_total = None
        self.metric_emergencies_total = None
        self.metric_checks_total = None
        self.metric_is_safe = None
        self.metric_emergency_active = None
        self.metric_geofence_enabled = None
        self.metric_health_status = None
        self.metric_altitude = None
        self.metric_distance_from_origin = None
        
        # ------------------------------------------------------------------
        # 2. Create ALL publishers / subscribers / services FIRST
        # ------------------------------------------------------------------
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_callback)
        rospy.Subscriber('/mavros/local_position/velocity', Twist, self._velocity_callback)
        rospy.Subscriber('/mavros/state', State, self._state_callback)
        
        self.safety_pub = rospy.Publisher('/safety_status', SafetyStatus, queue_size=10)
        self.health_pub = rospy.Publisher('/safety_monitor/node_health', NodeHealth, queue_size=10)
        self.emergency_pub = rospy.Publisher('/emergency_triggered', Bool, queue_size=10)
        
        rospy.Service('/safety/geofence_enable', Trigger, self._enable_geofence)
        rospy.Service('/safety/geofence_disable', Trigger, self._disable_geofence)
        rospy.Service('/safety/check', Trigger, self._check_safety)
        rospy.Service('/safety/status', Trigger, self._get_status)
        
        # ------------------------------------------------------------------
        # 3. NOW it is safe to touch the parameter server and heavy imports
        # ------------------------------------------------------------------
        self._load_parameters()
        self._init_metrics()
        
        # Timers (they only start after the node is fully wired)
        self.check_timer = rospy.Timer(rospy.Duration(1.0), self._check_safety_timer)
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)
        
        rospy.loginfo("Safety Monitor initialized with metrics")
        rospy.loginfo("Health publisher: /safety_monitor/node_health")

    # ----------------------------------------------------------------------
    # Deferred parameter loading – never blocks rospy.init_node
    # ----------------------------------------------------------------------
    def _load_parameters(self):
        """Load all parameters AFTER publishers/subscribers exist."""
        try:
            self.geofence_enabled = rospy.get_param('~enable_geofence', True)
            self.geofence_radius = rospy.get_param('~geofence_radius', 100.0)
            self.max_altitude = rospy.get_param('~max_altitude', 50.0)
            self.min_altitude = rospy.get_param('~min_altitude', 2.0)
            self.enable_emergency_landing = rospy.get_param('~enable_emergency_landing', True)
            
            # These two were previously called too early and could block
            self.simulation_mode = rospy.get_param('/use_simulation', True)
            self.test_mode = rospy.get_param('/test_mode', False)
            
            rospy.loginfo("Safety parameters loaded")
        except Exception as e:
            rospy.logwarn(f"Parameter load failed (using defaults): {e}")

    # ----------------------------------------------------------------------
    # Lazy + guarded metric initialisation
    # ----------------------------------------------------------------------
    def _init_metrics(self):
        """Initialize Prometheus metrics – import is deferred and guarded."""
        try:
            # Heavy imports only after the node is up
            from prometheus_client import Counter, Gauge, Histogram
            
            prefix = "safety_monitor"
            
            self.metric_violations_total = Counter(
                f"{prefix}_violations_total",
                "Total number of safety violations",
                ["violation_type", "mission_id"]
            )
            self.metric_emergencies_total = Counter(
                f"{prefix}_emergencies_total",
                "Total number of emergency events",
                ["reason", "mission_id"]
            )
            self.metric_checks_total = Counter(
                f"{prefix}_checks_total",
                "Total number of safety checks performed",
                ["mission_id"]
            )
            
            self.metric_is_safe = Gauge(
                f"{prefix}_is_safe",
                "Current safety status (1=safe, 0=unsafe)",
                ["mission_id"]
            )
            self.metric_emergency_active = Gauge(
                f"{prefix}_emergency_active",
                "Emergency active status (1=active, 0=inactive)",
                ["mission_id"]
            )
            self.metric_geofence_enabled = Gauge(
                f"{prefix}_geofence_enabled",
                "Geofence enabled status (1=enabled, 0=disabled)",
                ["mission_id"]
            )
            self.metric_health_status = Gauge(
                f"{prefix}_health_status",
                "Health status (1=healthy, 0=unhealthy)",
                ["mission_id"]
            )
            
            self.metric_altitude = Histogram(
                f"{prefix}_altitude_meters",
                "Current altitude in meters",
                ["mission_id"],
                buckets=[0, 2, 5, 10, 15, 20, 25, 30, 40, 50, 75, 100]
            )
            self.metric_distance_from_origin = Histogram(
                f"{prefix}_distance_from_origin_meters",
                "Distance from origin in meters",
                ["mission_id"],
                buckets=[0, 10, 25, 50, 75, 100, 150, 200, 300, 500]
            )
            
            self.metric_geofence_enabled.labels(
                mission_id=self.current_mission_id
            ).set(1 if self.geofence_enabled else 0)
            
            rospy.loginfo("Safety Monitor metrics initialized")
            
        except ImportError:
            rospy.logwarn("prometheus_client not available, metrics disabled")
        except Exception as e:
            rospy.logwarn(f"Metrics init failed: {e}")

    # ----------------------------------------------------------------------
    # Health publisher – lazy-import psutil so a missing dep does not cascade
    # ----------------------------------------------------------------------
    def _publish_health(self, event):
        """Publish node health to namespaced topic with all required fields"""
        try:
            is_healthy = self.is_healthy and not self.emergency_active
            
            if self.metric_health_status:
                try:
                    self.metric_health_status.labels(
                        mission_id=self.current_mission_id
                    ).set(1 if is_healthy else 0)
                except Exception:
                    pass
            
            # Lazy import – one missing dep must not kill the node
            try:
                import psutil
                cpu = psutil.cpu_percent()
                mem = psutil.virtual_memory().percent
            except ImportError:
                cpu = 0.0
                mem = 0.0
            
            msg = NodeHealth()
            msg.node_name = 'safety_monitor'
            msg.status = 'running' if is_healthy else 'error'
            msg.timestamp = rospy.Time.now()
            msg.is_healthy = is_healthy
            msg.detection_count = 0
            msg.error_count = self.error_count
            msg.fps = 0.0
            msg.cpu_usage = cpu
            msg.memory_usage = mem
            
            self.health_pub.publish(msg)
            self.last_health_publish = time.time()
            
        except Exception as e:
            rospy.logwarn(f"Error publishing health: {e}")
        
    def _pose_callback(self, msg):
        self.current_pose = msg.pose
        
        if self.current_pose and not self.emergency_active:
            try:
                pos = self.current_pose.position
                
                if self.metric_altitude:
                    self.metric_altitude.labels(
                        mission_id=self.current_mission_id
                    ).observe(pos.z)
                
                if self.metric_distance_from_origin:
                    distance = np.linalg.norm([pos.x, pos.y])
                    self.metric_distance_from_origin.labels(
                        mission_id=self.current_mission_id
                    ).observe(distance)
            except Exception:
                pass
        
    def _velocity_callback(self, msg):
        self.current_velocity = msg
        
    def _state_callback(self, msg):
        if not msg.connected:
            self._trigger_emergency("LOST_CONNECTION")
            
    def _check_safety_timer(self, event):
        """Periodic safety check"""
        self.total_checks += 1
        
        if self.metric_checks_total:
            try:
                self.metric_checks_total.labels(
                    mission_id=self.current_mission_id
                ).inc()
            except Exception:
                pass
        
        if self.current_pose is None:
            return
            
        violations = []
        pos = self.current_pose.position
        
        try:
            if self.geofence_enabled:
                distance = np.linalg.norm([pos.x, pos.y])
                if distance > self.geofence_radius:
                    violations.append(("GEOFENCE_BREACH", f"{distance:.1f}m > {self.geofence_radius}m"))
                    
            if pos.z > self.max_altitude:
                violations.append(("ALTITUDE_TOO_HIGH", f"{pos.z:.1f}m > {self.max_altitude}m"))
            elif pos.z < self.min_altitude:
                violations.append(("ALTITUDE_TOO_LOW", f"{pos.z:.1f}m < {self.min_altitude}m"))
                
            if self.current_velocity:
                speed = np.linalg.norm([
                    self.current_velocity.linear.x,
                    self.current_velocity.linear.y,
                    self.current_velocity.linear.z
                ])
                # Param read is now safe – node is fully initialised
                max_speed = rospy.get_param('dynamics/constraints/max_velocity', 5.0)
                if speed > max_speed * 1.5:
                    violations.append(("SPEED_EXCEEDED", f"{speed:.1f}m/s"))
                    
            if violations:
                self.is_safe = False
                for violation_type, violation_msg in violations:
                    if violation_type not in self.violation_counts:
                        self.violation_counts[violation_type] = 0
                    self.violation_counts[violation_type] += 1
                    
                    if violation_msg not in self.safety_violations:
                        self.safety_violations.append(violation_msg)
                        rospy.logwarn(f"Safety violation: {violation_msg}")
                        self.error_count += 1
                        
                        if self.metric_violations_total:
                            try:
                                self.metric_violations_total.labels(
                                    violation_type=violation_type,
                                    mission_id=self.current_mission_id
                                ).inc()
                            except Exception:
                                pass
                        
                if any(v[0] in ['GEOFENCE_BREACH', 'ALTITUDE_TOO_HIGH', 'ALTITUDE_TOO_LOW'] for v in violations):
                    self._trigger_emergency(violations[0][1])
            else:
                self.is_safe = True
                self.safety_violations = []
                
            if self.metric_is_safe:
                try:
                    self.metric_is_safe.labels(
                        mission_id=self.current_mission_id
                    ).set(1 if self.is_safe else 0)
                except Exception:
                    pass
                
        except Exception as e:
            self.error_handler.handle_error(e, "Safety check failed")
            self.error_count += 1
            
        status_msg = SafetyStatus()
        status_msg.is_safe = self.is_safe
        status_msg.violations = self.safety_violations
        status_msg.timestamp = rospy.Time.now()
        status_msg.emergency_active = self.emergency_active
        status_msg.emergency_reason = self.safety_violations[0] if self.safety_violations else ""
        self.safety_pub.publish(status_msg)
        
        self.last_check_time = time.time()
        
    def _trigger_emergency(self, reason: str):
        """Trigger emergency procedures"""
        if not self.emergency_active:
            self.emergency_active = True
            self.is_healthy = False
            rospy.logerr(f"EMERGENCY TRIGGERED: {reason}")
            
            if self.metric_emergency_active:
                try:
                    self.metric_emergency_active.labels(
                        mission_id=self.current_mission_id
                    ).set(1)
                except Exception:
                    pass
            
            if self.metric_emergencies_total:
                try:
                    short_reason = reason.split(':')[0] if ':' in reason else reason[:20]
                    self.metric_emergencies_total.labels(
                        reason=short_reason,
                        mission_id=self.current_mission_id
                    ).inc()
                except Exception:
                    pass
            
            self.emergency_pub.publish(True)
            
            if self.enable_emergency_landing:
                self._emergency_land()
                
    def _emergency_land(self):
        """Execute emergency landing"""
        rospy.logwarn("Executing emergency landing...")
        
    def _enable_geofence(self, req):
        self.geofence_enabled = True
        if self.metric_geofence_enabled:
            try:
                self.metric_geofence_enabled.labels(
                    mission_id=self.current_mission_id
                ).set(1)
            except Exception:
                pass
        return TriggerResponse(success=True, message="Geofence enabled")
        
    def _disable_geofence(self, req):
        self.geofence_enabled = False
        if self.metric_geofence_enabled:
            try:
                self.metric_geofence_enabled.labels(
                    mission_id=self.current_mission_id
                ).set(0)
            except Exception:
                pass
        return TriggerResponse(success=True, message="Geofence disabled")
        
    def _check_safety(self, req):
        return TriggerResponse(
            success=self.is_safe,
            message=f"Safety status: {'SAFE' if self.is_safe else 'UNSAFE'}"
        )
        
    def _get_status(self, req):
        status = {
            'is_safe': self.is_safe,
            'violations': self.safety_violations,
            'geofence_enabled': self.geofence_enabled,
            'emergency_active': self.emergency_active,
            'violation_counts': self.violation_counts,
            'total_checks': self.total_checks,
            'error_count': self.error_count,
            'pose': {
                'x': self.current_pose.position.x if self.current_pose else 0,
                'y': self.current_pose.position.y if self.current_pose else 0,
                'z': self.current_pose.position.z if self.current_pose else 0
            } if self.current_pose else {}
        }
        return TriggerResponse(success=True, message=str(status))


if __name__ == '__main__':
    try:
        monitor = SafetyMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass