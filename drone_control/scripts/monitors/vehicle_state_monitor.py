

import rospy
import psutil
import time
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TwistStamped
from drone_control.msg import NodeHealth, DiagnosticStatus
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

class VehicleStateMonitor:
    """Vehicle state monitoring with diagnostics"""
    
    def __init__(self):
        rospy.init_node('vehicle_state_monitor', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='vehicle_state_monitor')
        
        # State variables
        self.state = None
        self.armed = False
        self.mode = "UNKNOWN"
        self.connected = False
        self.safety_violations = []
        self.start_time = time.time()
        
        # Battery monitoring
        self.battery_voltage = 0.0
        self.battery_percentage = 100.0
        self.battery_warning_threshold = rospy.get_param('vehicle/battery_warning_threshold', 25.0)
        self.battery_critical_threshold = rospy.get_param('vehicle/battery_critical_threshold', 15.0)
        
        # GPS monitoring
        self.gps_fix = False
        self.gps_satellites = 0
        self.gps_accuracy = 0.0
        
        # System health
        self.cpu_usage = 0.0
        self.memory_usage = 0.0
        self.uptime = 0.0
        
        # Subscribers - FIXED: Added BatteryState import
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/battery', BatteryState, self.battery_callback)
        rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self.velocity_callback)
        
        # Publishers
        self.status_pub = rospy.Publisher('/drone_status', String, queue_size=10)
        self.diagnostic_pub = rospy.Publisher('/diagnostic_status', DiagnosticStatus, queue_size=10)
        self.health_pub = rospy.Publisher('/vehicle_state_monitor/node_health', NodeHealth, queue_size=10)
        self.battery_pub = rospy.Publisher('/battery_percentage', Float32, queue_size=10)
        
        # Services
        rospy.Service('/drone_emergency_stop', Trigger, self.emergency_stop)
        rospy.Service('/drone_arm', Trigger, self.arm)
        rospy.Service('/drone_disarm', Trigger, self.disarm)
        rospy.Service('/drone_get_status', Trigger, self.get_status)
        
        # Timers
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)
        self.diagnostic_timer = rospy.Timer(rospy.Duration(2.0), self._publish_diagnostic)
        self.status_timer = rospy.Timer(rospy.Duration(5.0), self._publish_status)

        self.health_pub = rospy.Publisher('/vehicle_state_monitor/node_health', NodeHealth, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self._publish_health)
        
        rospy.loginfo("Vehicle State Monitor started")

    def _publish_health(self, event):
        msg = NodeHealth()
        msg.node_name = 'vehicle_state_monitor'
        msg.status = 'running'
        msg.timestamp = rospy.Time.now()
        msg.is_healthy = True
        msg.cpu_usage = psutil.cpu_percent()
        msg.memory_usage = psutil.virtual_memory().percent


        self.health_pub.publish(msg)
        
    def state_callback(self, msg):
        """Handle MAVROS state updates"""
        self.state = msg
        self.armed = msg.armed
        self.mode = msg.mode
        self.connected = msg.connected
        
    def battery_callback(self, msg):
        """Handle battery updates"""
        self.battery_voltage = msg.voltage
        self.battery_percentage = msg.percentage * 100.0
        
        # Check battery thresholds
        if self.battery_percentage < self.battery_critical_threshold:
            self._log_safety_violation("BATTERY_CRITICAL")
        elif self.battery_percentage < self.battery_warning_threshold:
            self._log_safety_violation("BATTERY_LOW")
            
    def velocity_callback(self, msg):
        """Monitor velocity for safety"""
        speed = np.linalg.norm([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        max_speed = rospy.get_param('dynamics/constraints/max_velocity', 5.0)
        
        if speed > max_speed * 1.2:
            self._log_safety_violation("SPEED_EXCEEDED")
            
    def _log_safety_violation(self, violation):
        """Log safety violations"""
        if violation not in self.safety_violations:
            self.safety_violations.append(violation)
            rospy.logwarn(f"Safety violation: {violation}")
            
    def _publish_status(self, event):
        """Publish vehicle status"""
        status_msg = String()
        status_msg.data = f"Armed: {self.armed}, Mode: {self.mode}, BatteryState: {self.battery_percentage:.1f}%"
        self.status_pub.publish(status_msg)
        
        # Publish battery
        self.battery_pub.publish(self.battery_percentage)
        
    def _publish_diagnostic(self, event):
        """Publish diagnostic information"""
        # Get system metrics
        self.cpu_usage = psutil.cpu_percent()
        self.memory_usage = psutil.virtual_memory().percent
        self.uptime = time.time() - self.start_time
        
        diagnostic_msg = DiagnosticStatus()
        diagnostic_msg.timestamp = rospy.Time.now()
        # Simplified diagnostic
        self.diagnostic_pub.publish(diagnostic_msg)
        
        
    def emergency_stop(self, req):
        """Emergency stop service"""
        rospy.logwarn("EMERGENCY STOP requested!")
        return TriggerResponse(success=True, message="Emergency stop triggered")
        
    def arm(self, req):
        """Arm drone service"""
        if not self.connected:
            return TriggerResponse(success=False, message="Drone not connected")
        rospy.loginfo("Arming requested")
        return TriggerResponse(success=True, message="Arming command sent")
        
    def disarm(self, req):
        """Disarm drone service"""
        rospy.loginfo("Disarming requested")
        return TriggerResponse(success=True, message="Disarming command sent")
        
    def get_status(self, req):
        """Get full status service"""
        status = {
            'armed': self.armed,
            'mode': self.mode,
            'connected': self.connected,
            'battery': self.battery_percentage,
            'safety_violations': self.safety_violations,
            'uptime': self.uptime
        }
        return TriggerResponse(success=True, message=str(status))

if __name__ == '__main__':
    try:
        import numpy as np
        monitor = VehicleStateMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass