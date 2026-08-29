#!/usr/bin/env python3
"""
drone_control/scripts/controllers/mission_manager.py
Centralized mission state machine
"""

import rospy
import time
import json
from enum import Enum
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass, field
from std_msgs.msg import String
from drone_control.msg import MissionStatus, Command, CommandResponse, NodeHealth, TrackedTargets, SafetyStatus
import sys
import os
import yaml

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from utils.error_handler import ErrorHandler
except ImportError:
    class ErrorHandler:
        def __init__(self, node_name):
            self.node_name = node_name
        def handle_error(self, error, context=None):
            rospy.logerr(f"[{self.node_name}] {error}: {context}")

class MissionState(Enum):
    """Mission states"""
    NONE = "none"  # Added for emergency transitions
    IDLE = "idle"
    INITIALIZING = "initializing"
    SEARCHING = "searching"
    TRACKING = "tracking"
    ENGAGING = "engaging"
    ATTACKING = "attacking"
    RETURNING = "returning"
    LANDING = "landing"
    EMERGENCY = "emergency"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"

class MissionEvent(Enum):
    """Mission events"""
    START = "start"
    STOP = "stop"
    PAUSE = "pause"
    RESUME = "resume"
    TARGET_DETECTED = "target_detected"
    TARGET_LOST = "target_lost"
    TARGET_ENGAGED = "target_engaged"
    TARGET_DESTROYED = "target_destroyed"
    EMERGENCY = "emergency"
    SAFETY_VIOLATION = "safety_violation"
    LOW_BATTERY = "low_battery"
    MISSION_COMPLETE = "mission_complete"
    ERROR = "error"

@dataclass
class Transition:
    """State transition definition"""
    from_state: MissionState
    to_state: MissionState
    event: MissionEvent
    condition: Optional[Callable] = None
    action: Optional[Callable] = None
    priority: int = 0

class MissionManager:
    """Centralized mission state machine"""
    
    def __init__(self):
        rospy.init_node('mission_manager', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='mission_manager')
        
        # Load configuration
        self.config = self._load_config()
        
        # State management
        self.current_state = MissionState.IDLE
        self.previous_state = MissionState.IDLE
        self.state_history = []
        self.state_start_time = time.time()
        
        # Mission data
        self.mission_id = None
        self.mission_data = {}
        self.target_data = {}
        self.waypoints = []
        self.current_waypoint_index = 0
        
        # Publishers
        self.status_pub = rospy.Publisher('/mission_status', MissionStatus, queue_size=10)
        self.health_pub = rospy.Publisher('/mission_manager/node_health', NodeHealth, queue_size=10)
        
        # Subscribers - FIXED: using correct message types
        self.command_sub = rospy.Subscriber('/drone_control/command', Command, self._command_callback)
        self.safety_sub = rospy.Subscriber('/safety_status', SafetyStatus, self._safety_callback)
        self.target_sub = rospy.Subscriber('/tracked_targets', TrackedTargets, self._target_callback)
        
        # Setup state machine
        self._setup_transitions()
        
        # State timer
        self.state_timer = rospy.Timer(rospy.Duration(0.1), self._update)
        
        # Health timer
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)
        
        rospy.loginfo("Mission Manager initialized")
        self._set_state(MissionState.IDLE)
        
    def _load_config(self) -> Dict:
        """Load mission configuration"""
        config_path = rospy.get_param('~mission_config', 'config/mission_config.yaml')
        
        # Try to find config file
        possible_paths = [
            config_path,
            os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'mission_config.yaml'),
            os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config', 'mission_config.yaml'),
        ]
        
        for path in possible_paths:
            try:
                if os.path.exists(path):
                    with open(path, 'r') as f:
                        return yaml.safe_load(f)
            except Exception:
                continue
                
        rospy.logwarn("Failed to load mission config, using defaults")
        return {
            'timeout': 300,
            'max_retries': 3,
            'search_altitude': 10.0,
            'approach_speed': 2.0
        }
        
    def _setup_transitions(self):
        """Configure state machine transitions"""
        self.transitions = [
            # IDLE -> INITIALIZING
            Transition(
                MissionState.IDLE, MissionState.INITIALIZING,
                MissionEvent.START,
                action=self._on_initialize
            ),
            
            # INITIALIZING -> SEARCHING
            Transition(
                MissionState.INITIALIZING, MissionState.SEARCHING,
                MissionEvent.TARGET_DETECTED,
                condition=self._is_initialized,
                action=self._on_start_search
            ),
            
            # SEARCHING -> TRACKING
            Transition(
                MissionState.SEARCHING, MissionState.TRACKING,
                MissionEvent.TARGET_DETECTED,
                condition=self._is_valid_target,
                action=self._on_start_tracking
            ),
            
            # TRACKING -> ENGAGING
            Transition(
                MissionState.TRACKING, MissionState.ENGAGING,
                MissionEvent.TARGET_ENGAGED,
                condition=self._is_within_engagement_distance,
                action=self._on_engage
            ),
            
            # ENGAGING -> ATTACKING
            Transition(
                MissionState.ENGAGING, MissionState.ATTACKING,
                MissionEvent.TARGET_ENGAGED,
                condition=self._is_within_attack_distance,
                action=self._on_attack
            ),
            
            # ATTACKING -> COMPLETED
            Transition(
                MissionState.ATTACKING, MissionState.COMPLETED,
                MissionEvent.TARGET_DESTROYED,
                action=self._on_complete
            ),
            
            # NONE state -> EMERGENCY (highest priority)
            Transition(
                MissionState.NONE, MissionState.EMERGENCY,
                MissionEvent.EMERGENCY,
                priority=100,
                action=self._on_emergency
            ),
            
            # NONE state -> PAUSED
            Transition(
                MissionState.NONE, MissionState.PAUSED,
                MissionEvent.PAUSE,
                priority=50,
                action=self._on_pause
            ),
            
            # PAUSED -> SEARCHING
            Transition(
                MissionState.PAUSED, MissionState.SEARCHING,
                MissionEvent.RESUME,
                action=self._on_resume
            ),
            
            # TRACKING -> SEARCHING (lost target)
            Transition(
                MissionState.TRACKING, MissionState.SEARCHING,
                MissionEvent.TARGET_LOST,
                action=self._on_target_lost
            ),
        ]
        
    def _set_state(self, new_state: MissionState):
        """Change state with logging"""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = time.time()
            
            rospy.loginfo(f"Mission state: {self.previous_state.value} -> {new_state.value}")
            self._publish_status()
            
    def _process_event(self, event: MissionEvent, data: Optional[Dict] = None) -> bool:
        """Process an event and handle transitions"""
        # Check for emergency events first (highest priority)
        if event in [MissionEvent.EMERGENCY, MissionEvent.SAFETY_VIOLATION]:
            for transition in self.transitions:
                if transition.event == event and transition.priority >= 50:
                    if transition.from_state == MissionState.NONE or transition.from_state == self.current_state:
                        if not transition.condition or transition.condition(data):
                            self._set_state(transition.to_state)
                            if transition.action:
                                transition.action(data)
                            return True
        
        # Process normal events
        for transition in self.transitions:
            if transition.event == event:
                if transition.from_state == MissionState.NONE or transition.from_state == self.current_state:
                    if not transition.condition or transition.condition(data):
                        self._set_state(transition.to_state)
                        if transition.action:
                            transition.action(data)
                        return True
                        
        rospy.logdebug(f"Unhandled event: {event.value} in state {self.current_state.value}")
        return False
        
    def _update(self, event):
        """Periodic update"""
        # Check for state timeouts
        elapsed = time.time() - self.state_start_time
        
        if self.current_state == MissionState.SEARCHING and elapsed > self.config.get('timeout', 300):
            self._process_event(MissionEvent.ERROR, {"message": "Search timeout"})
            
        if self.current_state == MissionState.TRACKING and elapsed > self.config.get('timeout', 300):
            self._process_event(MissionEvent.ERROR, {"message": "Tracking timeout"})
            
    def _publish_status(self):
        """Publish current mission status"""
        status_msg = MissionStatus()
        status_msg.mission_id = self.mission_id or "unknown"
        status_msg.state = self.current_state.value
        status_msg.previous_state = self.previous_state.value
        status_msg.elapsed_time = time.time() - self.state_start_time
        status_msg.current_waypoint = self.current_waypoint_index
        status_msg.total_waypoints = len(self.waypoints)
        status_msg.target_count = 1 if self.target_data else 0
        self.status_pub.publish(status_msg)
        
    def _publish_health(self, event=None):
        """Publish node health"""
        health_msg = NodeHealth()
        health_msg.node_name = 'mission_manager'
        health_msg.status = self.current_state.value
        health_msg.timestamp = rospy.Time.now()
        health_msg.is_healthy = self.current_state != MissionState.EMERGENCY

          # Add CPU and memory usage
        import psutil
        health_msg.cpu_usage = psutil.cpu_percent()
        health_msg.memory_usage = psutil.virtual_memory().percent

        self.health_pub.publish(health_msg)
        
    # ============================================
    # ACTION METHODS
    # ============================================
    
    def _on_initialize(self, data: Optional[Dict] = None):
        """Initialize mission"""
        self.mission_id = f"mission_{int(time.time())}"
        self.mission_data = data or {}
        rospy.loginfo(f"Mission {self.mission_id} initialized")
        
    def _on_start_search(self, data: Optional[Dict] = None):
        """Start search pattern"""
        rospy.loginfo("Starting search pattern")
        
    def _on_start_tracking(self, data: Optional[Dict] = None):
        """Start tracking target"""
        if data and 'target' in data:
            self.target_data = data['target']
            rospy.loginfo(f"Tracking target: {self.target_data.get('id', 'unknown')}")
            
    def _on_engage(self, data: Optional[Dict] = None):
        """Engage target"""
        rospy.loginfo("Engaging target")
        
    def _on_attack(self, data: Optional[Dict] = None):
        """Attack target"""
        rospy.loginfo("ATTACKING target!")
        
    def _on_complete(self, data: Optional[Dict] = None):
        """Mission complete"""
        rospy.loginfo("Mission complete!")
        
    def _on_emergency(self, data: Optional[Dict] = None):
        """Emergency handling"""
        rospy.logwarn(f"EMERGENCY: {data.get('message', 'Unknown emergency')}")
        
    def _on_pause(self, data: Optional[Dict] = None):
        """Pause mission"""
        rospy.loginfo("Mission paused")
        
    def _on_resume(self, data: Optional[Dict] = None):
        """Resume mission"""
        rospy.loginfo("Mission resumed")
        
    def _on_target_lost(self, data: Optional[Dict] = None):
        """Handle lost target"""
        rospy.logwarn("Target lost, resuming search")
        self.target_data = {}
        
    # ============================================
    # CONDITION METHODS
    # ============================================
    
    def _is_initialized(self, data: Optional[Dict] = None) -> bool:
        """Check if system is initialized"""
        return rospy.has_param('/system_initialized')
        
    def _is_valid_target(self, data: Optional[Dict] = None) -> bool:
        """Check if target is valid"""
        if not data or 'target' not in data:
            return False
        target = data['target']
        return target.get('confidence', 0) > 0.5
        
    def _is_within_engagement_distance(self, data: Optional[Dict] = None) -> bool:
        """Check if within engagement distance"""
        if not self.target_data:
            return False
        return self.target_data.get('distance', 100) < rospy.get_param('/tracking/attack_distance', 2.0)
        
    def _is_within_attack_distance(self, data: Optional[Dict] = None) -> bool:
        """Check if within attack distance"""
        if not self.target_data:
            return False
        return self.target_data.get('distance', 100) < rospy.get_param('/tracking/attack_distance', 2.0) * 0.5
        
    # ============================================
    # CALLBACKS - FIXED with correct message types
    # ============================================
    
    def _command_callback(self, msg):
        """Handle commands"""
        rospy.logdebug(f"Command received: {msg.command}")
        
        command_map = {
            'start': MissionEvent.START,
            'stop': MissionEvent.MISSION_COMPLETE,
            'pause': MissionEvent.PAUSE,
            'resume': MissionEvent.RESUME,
            'emergency': MissionEvent.EMERGENCY,
            'land': MissionEvent.MISSION_COMPLETE,
        }
        
        event = command_map.get(msg.command.lower())
        if event:
            self._process_event(event, {})
            
    def _safety_callback(self, msg):
        """Handle safety events - FIXED for SafetyStatus message"""
        # Check if emergency is active
        if msg.emergency_active:
            self._process_event(MissionEvent.EMERGENCY, {'message': msg.emergency_reason or 'Emergency active'})
        # Check for safety violations
        elif msg.violations and len(msg.violations) > 0:
            self._process_event(MissionEvent.SAFETY_VIOLATION, {'message': ', '.join(msg.violations)})
        # Check if not safe
        elif not msg.is_safe:
            self._process_event(MissionEvent.SAFETY_VIOLATION, {'message': 'Safety status unsafe'})
            
    def _target_callback(self, msg):
        """Handle target updates - FIXED for TrackedTargets message"""
        if msg.targets and len(msg.targets) > 0:
            # Convert to dict for internal use
            target = msg.targets[0]
            self.target_data = {
                'id': target.id,
                'confidence': target.confidence,
                'distance': target.distance,
                'position': [target.position.x, target.position.y, target.position.z],
                'velocity': [target.velocity.x, target.velocity.y, target.velocity.z],
                'state': target.state
            }
            
            # Trigger events based on current state
            if self.current_state == MissionState.SEARCHING:
                self._process_event(MissionEvent.TARGET_DETECTED, {'target': self.target_data})
            elif self.current_state == MissionState.TRACKING:
                # Check if within engagement distance
                if self._is_within_engagement_distance():
                    self._process_event(MissionEvent.TARGET_ENGAGED, {'target': self.target_data})

if __name__ == '__main__':
    try:
        manager = MissionManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass