#!/usr/bin/env python3
"""
drone_control/scripts/monitors/health_checker.py
Node health monitoring and auto-restart
"""

import rospy
import time
import psutil
from drone_control.msg import NodeHealth
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import subprocess
import sys
import os

class HealthChecker:
    """Comprehensive node health monitoring"""
    
    def __init__(self):
        rospy.init_node('health_checker', anonymous=False)
        
        # Configuration
        self.check_interval = rospy.get_param('~check_interval', 5.0)
        self.timeout_threshold = rospy.get_param('~timeout_threshold', 10.0)
        self.restart_on_failure = rospy.get_param('~restart_on_failure', True)
        self.max_restarts = rospy.get_param('~max_restarts', 3)
        
        # Monitored nodes
        self.monitored_nodes = rospy.get_param('~monitored_nodes', [
            'yolo_detector',
            'target_tracking_controller',
            'vehicle_state_monitor',
            'mission_manager',
            'safety_monitor'
        ])
        
        # State tracking
        self.node_health = {}
        self.node_restart_counts = {}
        self.last_health_time = {}
        
        # Subscribers
        for node in self.monitored_nodes:
            rospy.Subscriber(f'/{node}/node_health', NodeHealth, self._health_callback, callback_args=node)
            
        # Publishers
        self.health_pub = rospy.Publisher('/system_health', NodeHealth, queue_size=10)
        
        # Services
        rospy.Service('/health/check_all', Trigger, self._check_all)
        rospy.Service('/health/restart_node', Trigger, self._restart_node)
        
        # Timer
        self.check_timer = rospy.Timer(rospy.Duration(self.check_interval), self._check_health)
        
        rospy.loginfo("Health Checker initialized")
        
    def _health_callback(self, msg, node_name):
        """Handle health messages from nodes"""
        self.node_health[node_name] = msg
        self.last_health_time[node_name] = time.time()
        
    def _check_health(self, event):
        """Periodic health check"""
        current_time = time.time()
        unhealthy_nodes = []
        
        for node in self.monitored_nodes:
            # Check if node reported health
            if node not in self.node_health:
                unhealthy_nodes.append((node, "No health report"))
                continue
                
            # Check health status
            if not self.node_health[node].is_healthy:
                unhealthy_nodes.append((node, "Reported unhealthy"))
                continue
                
            # Check timeout
            if node in self.last_health_time:
                elapsed = current_time - self.last_health_time[node]
                if elapsed > self.timeout_threshold:
                    unhealthy_nodes.append((node, f"Timeout ({elapsed:.1f}s)"))
                    
        # Handle unhealthy nodes
        if unhealthy_nodes:
            rospy.logwarn(f"Unhealthy nodes: {len(unhealthy_nodes)}")
            for node, reason in unhealthy_nodes:
                rospy.logwarn(f"  {node}: {reason}")
                
                if self.restart_on_failure:
                    self._restart_node_attempt(node)
        else:
            rospy.logdebug("All nodes healthy")
            
        # Publish system health
        health_msg = NodeHealth()
        health_msg.node_name = 'system'
        health_msg.status = 'healthy' if not unhealthy_nodes else 'degraded'
        health_msg.timestamp = rospy.Time.now()
        health_msg.is_healthy = len(unhealthy_nodes) == 0
        self.health_pub.publish(health_msg)
        
    def _restart_node_attempt(self, node_name):
        """Attempt to restart a node"""
        # Track restart attempts
        if node_name not in self.node_restart_counts:
            self.node_restart_counts[node_name] = 0
            
        self.node_restart_counts[node_name] += 1
        
        if self.node_restart_counts[node_name] > self.max_restarts:
            rospy.logerr(f"Node {node_name} exceeded max restarts ({self.max_restarts})")
            return
            
        rospy.loginfo(f"Attempting to restart {node_name} (attempt {self.node_restart_counts[node_name]})")
        
        # Attempt restart using roslaunch or rosnode kill
        try:
            # Kill existing node
            subprocess.run(['rosnode', 'kill', node_name], timeout=5)
            rospy.sleep(1)
            
            # Relaunch node (implementation depends on launch system)
            # This would need to be implemented based on how nodes are launched
            rospy.loginfo(f"Node {node_name} restart initiated")
            
        except Exception as e:
            rospy.logerr(f"Failed to restart {node_name}: {e}")
            
    def _check_all(self, req):
        """Check all nodes health service"""
        results = []
        for node in self.monitored_nodes:
            if node in self.node_health:
                health = self.node_health[node]
                results.append(f"{node}: {health.status} (healthy: {health.is_healthy})")
            else:
                results.append(f"{node}: No health report")
                
        return TriggerResponse(
            success=True,
            message="\n".join(results)
        )
        
    def _restart_node(self, req):
        """Restart a specific node service"""
        # This would need to be implemented
        return TriggerResponse(
            success=False,
            message="Restart functionality not fully implemented"
        )

if __name__ == '__main__':
    try:
        checker = HealthChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass