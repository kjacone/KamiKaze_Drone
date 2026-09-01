#!/usr/bin/env python3
"""
drone_control/scripts/diagnostic_reporter.py
System-wide diagnostic reporting
"""

import json
import time

import psutil
import rospy
from std_msgs.msg import String

from drone_control import DiagnosticStatus, MissionStatus, NodeHealth, SafetyStatus


class DiagnosticReporter:
    """System-wide diagnostic status reporting"""
    
    def __init__(self):
        rospy.init_node('diagnostic_reporter', anonymous=False)
        
        self.report_rate = rospy.get_param('~report_rate', 5.0)
        self.output_file = rospy.get_param('~output_file', 'diagnostic.log')
        
        # State
        self.node_health = {}
        self.safety_status = None
        self.mission_status = None
        self.start_time = time.time()
        
        # Subscribers
        rospy.Subscriber('/node_health', NodeHealth, self._node_health_callback)
        rospy.Subscriber('/safety_status', SafetyStatus, self._safety_callback)
        rospy.Subscriber('/mission_status', MissionStatus, self._mission_callback)
        
        # Publishers
        self.diagnostic_pub = rospy.Publisher('/diagnostic_status', DiagnosticStatus, queue_size=10)
        self.status_pub = rospy.Publisher('/system_status', String, queue_size=10)
        
        # Timer
        self.report_timer = rospy.Timer(rospy.Duration(self.report_rate), self._report_diagnostics)
        
        rospy.loginfo("Diagnostic Reporter initialized")
        
    def _node_health_callback(self, msg):
        """Handle node health messages"""
        self.node_health[msg.node_name] = msg
        
    def _safety_callback(self, msg):
        """Handle safety status messages"""
        self.safety_status = msg
        
    def _mission_callback(self, msg):
        """Handle mission status messages"""
        self.mission_status = msg
        
    def _report_diagnostics(self, event):
        """Generate and publish diagnostic report"""
        # System metrics
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent()
        disk_percent = psutil.disk_usage('/').percent
        
        # Build diagnostic message
        diag_msg = DiagnosticStatus()
        diag_msg.timestamp = rospy.Time.now()
        
        # System health
        diag_msg.system_health = json.dumps({
            'cpu_usage': cpu_percent,
            'memory_usage': memory_percent,
            'disk_usage': disk_percent,
            'uptime': time.time() - self.start_time,
            'node_count': len(self.node_health)
        })
        
        # Node status
        node_status = {}
        for name, health in self.node_health.items():
            node_status[name] = {
                'status': health.status,
                'is_healthy': health.is_healthy,
                'last_seen': health.timestamp.to_sec() if health.timestamp else 0
            }
        diag_msg.node_status = json.dumps(node_status)
        
        # Safety status
        if self.safety_status:
            diag_msg.safety_status = json.dumps({
                'is_safe': self.safety_status.is_safe,
                'violations': list(self.safety_status.violations)
            })
            
        # Mission status
        if self.mission_status:
            diag_msg.mission_status = json.dumps({
                'state': self.mission_status.state,
                'elapsed_time': self.mission_status.elapsed_time
            })
            
        # Publish diagnostic
        self.diagnostic_pub.publish(diag_msg)
        
        # Also publish as string for logging
        status_msg = String()
        status_msg.data = f"CPU: {cpu_percent}%, Memory: {memory_percent}%, Nodes: {len(self.node_health)}"
        self.status_pub.publish(status_msg)
        
        # Log to file if specified
        if self.output_file:
            try:
                with open(self.output_file, 'a') as f:
                    f.write(f"{time.time()}: {diag_msg.system_health}\n")
            except Exception:
                pass

if __name__ == '__main__':
    try:
        reporter = DiagnosticReporter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass