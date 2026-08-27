#!/usr/bin/env python3
"""
drone_control/scripts/monitors/system_health_monitor.py
Comprehensive system health monitoring
"""

import rospy
import psutil
import time
import subprocess
from drone_control.msg import NodeHealth, DiagnosticStatus
from std_msgs.msg import String
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

class SystemHealthMonitor:
    """Comprehensive system health monitoring"""
    
    def __init__(self):
        rospy.init_node('system_health_monitor', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='system_health_monitor')
        
        # State
        self.node_health = {}
        self.last_update = {}
        self.start_time = time.time()
        
        # Subscribers
        rospy.Subscriber('/node_health', NodeHealth, self._node_health_callback)
        
        # Publishers
        self.health_pub = rospy.Publisher('/system_health', DiagnosticStatus, queue_size=10)
        self.status_pub = rospy.Publisher('/system_status', String, queue_size=10)
        
        # Timers
        self.health_timer = rospy.Timer(rospy.Duration(2.0), self._publish_health)
        self.status_timer = rospy.Timer(rospy.Duration(5.0), self._publish_status)
        
        rospy.loginfo("System Health Monitor initialized")
        
    def _node_health_callback(self, msg):
        """Handle node health messages"""
        self.node_health[msg.node_name] = msg
        self.last_update[msg.node_name] = time.time()
        
    def _publish_health(self, event):
        """Publish system health"""
        current_time = time.time()
        
        # Check for stale nodes
        stale_nodes = []
        for node, last_time in self.last_update.items():
            if current_time - last_time > 10.0:
                stale_nodes.append(node)
                
        # Get system metrics
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent
        disk_percent = psutil.disk_usage('/').percent
        
        # Check ROS nodes
        try:
            ros_nodes = subprocess.check_output(['rosnode', 'list'], text=True).strip().split('\n')
        except:
            ros_nodes = []
            
        # Build health status
        health_status = DiagnosticStatus()
        health_status.timestamp = rospy.Time.now()
        health_status.system_health = {
            'cpu_usage': cpu_percent,
            'memory_usage': memory_percent,
            'disk_usage': disk_percent,
            'uptime': time.time() - self.start_time,
            'node_count': len(self.node_health),
            'stale_nodes': stale_nodes,
            'healthy_nodes': len([n for n, h in self.node_health.items() if h.is_healthy]),
            'total_nodes': len(self.node_health)
        }
        
        self.health_pub.publish(health_status)
        
        # Log warnings
        if cpu_percent > 80:
            rospy.logwarn(f"High CPU usage: {cpu_percent}%")
        if memory_percent > 80:
            rospy.logwarn(f"High memory usage: {memory_percent}%")
        if stale_nodes:
            rospy.logwarn(f"Stale nodes: {stale_nodes}")
            
    def _publish_status(self, event):
        """Publish system status"""
        status = f"System: {len(self.node_health)} nodes, "
        status += f"CPU: {psutil.cpu_percent()}%, "
        status += f"Memory: {psutil.virtual_memory().percent}%"
        
        self.status_pub.publish(status)

if __name__ == '__main__':
    try:
        monitor = SystemHealthMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass