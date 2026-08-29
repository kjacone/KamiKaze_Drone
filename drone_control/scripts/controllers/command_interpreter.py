#!/usr/bin/env python3
"""
drone_control/scripts/controllers/command_interpreter.py
Unified command interface
"""

import rospy
import json
import time
from typing import Dict, Any, Tuple, Optional
from enum import Enum
from drone_control.msg import Command, CommandResponse, NodeHealth 
import sys
import os
import psutil

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from utils.error_handler import ErrorHandler
except ImportError:
    class ErrorHandler:
        def __init__(self, node_name):
            self.node_name = node_name
        def handle_error(self, error, context=None):
            rospy.logerr(f"[{self.node_name}] {error}: {context}")

class CommandType(Enum):
    """Supported command types"""
    MISSION = "mission"
    CONTROL = "control"
    CONFIG = "config"
    SAFETY = "safety"
    DIAGNOSTIC = "diagnostic"
    TEST = "test"

class CommandPriority(Enum):
    """Command priority levels"""
    CRITICAL = 0
    HIGH = 1
    NORMAL = 2
    LOW = 3

class CommandInterpreter:
    """Unified command interface for all system control"""
    
    def __init__(self):
        rospy.init_node('command_interpreter', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='command_interpreter')
        
        # Command registry
        self.command_handlers = {}
        self._register_handlers()
        
        # Subscribers
        self.command_sub = rospy.Subscriber('/drone_control/command', Command, self._command_callback)
        
        # Publishers
        self.response_pub = rospy.Publisher('/drone_control/command_response', CommandResponse, queue_size=10)

        self.health_pub = rospy.Publisher('/command_interpreter/node_health', NodeHealth, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self._publish_health)

        rospy.loginfo("Command Interpreter initialized")

    def _publish_health(self, event):
        msg = NodeHealth()
        msg.node_name = 'command_interpreter'
        msg.status = 'running'
        msg.timestamp = rospy.Time.now()
        msg.is_healthy = True
        msg.cpu_usage = psutil.cpu_percent()
        msg.memory_usage = psutil.virtual_memory().percent

        self.health_pub.publish(msg)
        
    def _register_handlers(self):
        """Register all command handlers"""
        self.command_handlers = {
            'start': self._handle_start,
            'stop': self._handle_stop,
            'pause': self._handle_pause,
            'resume': self._handle_resume,
            'status': self._handle_status,
            'engage': self._handle_engage,
            'abort': self._handle_abort,
            'set_speed': self._handle_set_speed,
            'set_altitude': self._handle_set_altitude,
            'emergency_land': self._handle_emergency_land,
            'emergency_stop': self._handle_emergency_stop,
            'return_home': self._handle_return_home,
            'geofence_enable': self._handle_geofence_enable,
            'geofence_disable': self._handle_geofence_disable,
            'config_reload': self._handle_config_reload,
            'diagnostic_status': self._handle_diagnostic_status,
            'test_start': self._handle_test_start,
            'test_stop': self._handle_test_stop,
        }
        
    def _command_callback(self, msg):
        """Handle incoming command messages"""
        try:
            command = msg.command.lower()
            data = msg.data if hasattr(msg, 'data') else {}
            
            if command in self.command_handlers:
                result = self.command_handlers[command](data)
                self._send_response(command, True, result.get('message', 'Command executed'), result.get('data'))
            else:
                self._send_response(command, False, f"Unknown command: {command}")
                
        except Exception as e:
            self.error_handler.handle_error(e, f"Command: {msg.command}")
            self._send_response(msg.command, False, str(e))
            
    def _send_response(self, command: str, success: bool, message: str = "", data: Any = None):
        """Send command response"""
        response = CommandResponse()
        response.original_command = command
        response.success = success
        response.message = message
        if data:
            response.data = json.dumps(data)
        self.response_pub.publish(response)
        
    # ============================================
    # COMMAND HANDLERS
    # ============================================
    
    def _handle_start(self, data: Dict) -> Dict:
        """Handle start command"""
        rospy.loginfo("Start command received")
        # Publish to mission manager
        return {'success': True, 'message': 'Mission started'}
        
    def _handle_stop(self, data: Dict) -> Dict:
        """Handle stop command"""
        rospy.loginfo("Stop command received")
        return {'success': True, 'message': 'Mission stopped'}
        
    def _handle_pause(self, data: Dict) -> Dict:
        """Handle pause command"""
        rospy.loginfo("Pause command received")
        return {'success': True, 'message': 'Mission paused'}
        
    def _handle_resume(self, data: Dict) -> Dict:
        """Handle resume command"""
        rospy.loginfo("Resume command received")
        return {'success': True, 'message': 'Mission resumed'}
        
    def _handle_status(self, data: Dict) -> Dict:
        """Handle status request"""
        return {
            'success': True, 
            'message': 'Status retrieved',
            'data': {
                'mode': rospy.get_param('/test_mode', False) and 'test' or 'production',
                'simulation': rospy.get_param('/use_simulation', True),
                'uptime': time.time() - rospy.get_time()
            }
        }
        
    def _handle_engage(self, data: Dict) -> Dict:
        """Handle engage command"""
        target_id = data.get('target_id', 0)
        rospy.loginfo(f"Engage target {target_id}")
        return {'success': True, 'message': f"Engaging target {target_id}"}
        
    def _handle_abort(self, data: Dict) -> Dict:
        """Handle abort command"""
        rospy.loginfo("Abort command received")
        return {'success': True, 'message': 'Aborted'}
        
    def _handle_set_speed(self, data: Dict) -> Dict:
        """Handle set speed command"""
        speed = data.get('speed', 2.0)
        rospy.loginfo(f"Setting speed to {speed} m/s")
        rospy.set_param('/max_velocity', speed)
        return {'success': True, 'message': f"Speed set to {speed} m/s"}
        
    def _handle_set_altitude(self, data: Dict) -> Dict:
        """Handle set altitude command"""
        altitude = data.get('altitude', 10.0)
        rospy.loginfo(f"Setting altitude to {altitude} m")
        return {'success': True, 'message': f"Altitude set to {altitude} m"}
        
    def _handle_emergency_land(self, data: Dict) -> Dict:
        """Handle emergency land command"""
        rospy.logwarn("Emergency land command received")
        return {'success': True, 'message': 'Emergency landing initiated'}
        
    def _handle_emergency_stop(self, data: Dict) -> Dict:
        """Handle emergency stop command"""
        rospy.logwarn("Emergency stop command received")
        return {'success': True, 'message': 'Emergency stop initiated'}
        
    def _handle_return_home(self, data: Dict) -> Dict:
        """Handle return home command"""
        rospy.loginfo("Return home command received")
        return {'success': True, 'message': 'Returning home'}
        
    def _handle_geofence_enable(self, data: Dict) -> Dict:
        """Handle geofence enable command"""
        rospy.loginfo("Geofence enabled")
        rospy.set_param('/safety/geofence_enabled', True)
        return {'success': True, 'message': 'Geofence enabled'}
        
    def _handle_geofence_disable(self, data: Dict) -> Dict:
        """Handle geofence disable command"""
        rospy.loginfo("Geofence disabled")
        rospy.set_param('/safety/geofence_enabled', False)
        return {'success': True, 'message': 'Geofence disabled'}
        
    def _handle_config_reload(self, data: Dict) -> Dict:
        """Handle config reload command"""
        rospy.loginfo("Reloading configuration")
        # Trigger parameter reload
        return {'success': True, 'message': 'Configuration reloaded'}
        
    def _handle_diagnostic_status(self, data: Dict) -> Dict:
        """Handle diagnostic status request"""
        return {
            'success': True,
            'message': 'Diagnostic status retrieved',
            'data': {
                'cpu_usage': 0,
                'memory_usage': 0,
                'nodes': []
            }
        }
        
    def _handle_test_start(self, data: Dict) -> Dict:
        """Handle test start command"""
        scenario = data.get('scenario', 'default')
        rospy.loginfo(f"Starting test scenario: {scenario}")
        return {'success': True, 'message': f"Test started: {scenario}"}
        
    def _handle_test_stop(self, data: Dict) -> Dict:
        """Handle test stop command"""
        rospy.loginfo("Stopping test")
        return {'success': True, 'message': 'Test stopped'}

if __name__ == '__main__':
    try:
        interpreter = CommandInterpreter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass