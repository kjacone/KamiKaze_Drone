#!/usr/bin/env python3
"""
drone_control/scripts/controllers/mission_manager.py
Centralized mission state machine with structured logging and metrics
"""

import os
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, Optional

import rospy
from std_msgs.msg import String

from drone_control.utils import ErrorHandler
from drone_control.utils.correlation import get_or_create_mission_id
from drone_control.utils.logging_framework import get_logger_with_ros_level
from drone_control.lib.control_lib import ControlLibrary
from drone_control.lib.safety_lib import SafetyLibrary
from drone_control.msg import (
    Command,
    CommandResponse,
    ControlCommand,
    MissionStatus,
    NodeHealth,
    SafetyStatus,
    TrackedTarget,
    TrackedTargets,
)


class MissionState(Enum):
    """Mission states"""
    NONE = "none"
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
    """Centralized mission state machine with structured logging and metrics"""
    
    def __init__(self):
        rospy.init_node('mission_manager', anonymous=False)
        
        # Structured logger (lightweight)
        self.logger = get_logger_with_ros_level("mission_manager")
        self.logger.info("node_initializing")
        
        self.error_handler = ErrorHandler(node_name='mission_manager')
        
        # ------------------------------------------------------------------
        # 1. Lightweight state – no params, no heavy imports yet
        # ------------------------------------------------------------------
        self.current_state = MissionState.IDLE
        self.previous_state = MissionState.IDLE
        self.state_history = []
        self.state_start_time = time.time()
        
        self.mission_id = None
        self.mission_data = {}
        self.target_data = {}
        self.waypoints = []
        self.current_waypoint_index = 0
        self.mission_start_time = None
        self._acquisition_start_time = None
        self.correlation_id = get_or_create_mission_id()
        
        # Place-holders for deferred objects
        self.metrics = None
        self.config = {}
        self.metric_state = None
        self.metric_mission_started = None
        self.metric_mission_completed = None
        self.metric_state_transitions = None
        self.metric_targets_tracked = None
        self.metric_mission_duration = None
        self.metric_target_acquisition_time = None
        self.metric_state_duration = None
        
        # ------------------------------------------------------------------
        # 2. Create ALL publishers / subscribers FIRST
        # ------------------------------------------------------------------
        self.status_pub = rospy.Publisher('/mission_status', MissionStatus, queue_size=10)
        self.health_pub = rospy.Publisher('/mission_manager/node_health', NodeHealth, queue_size=10)
        
        self.command_sub = rospy.Subscriber('/drone_control/command', Command, self._command_callback)
        self.safety_sub = rospy.Subscriber('/safety_status', SafetyStatus, self._safety_callback)
        self.target_sub = rospy.Subscriber('/tracked_targets', TrackedTargets, self._target_callback)
        
        # ------------------------------------------------------------------
        # 3. NOW safe to load config, create MetricsCollector, etc.
        # ------------------------------------------------------------------
        self._load_config()
        self._init_metrics()
        self._setup_transitions()
        
        # Timers after everything else is ready
        self.state_timer = rospy.Timer(rospy.Duration(0.1), self._update)
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)
        
        self.logger.info("node_initialized", extra={
            "correlation_id": self.correlation_id,
            "initial_state": self.current_state.value
        })
        self._set_state(MissionState.IDLE)
        
    # ----------------------------------------------------------------------
    # Deferred config load (yaml + rospy.get_param)
    # ----------------------------------------------------------------------
    def _load_config(self) -> None:
        """Load mission configuration AFTER the node is fully wired."""
        # Lazy-import yaml so a missing dep does not cascade
        try:
            import yaml
        except ImportError:
            self.logger.warning("yaml not available – using defaults")
            self.config = {
                'timeout': 300,
                'max_retries': 3,
                'search_altitude': 10.0,
                'approach_speed': 2.0
            }
            return
        
        try:
            config_path = rospy.get_param('~mission_config', 'config/mission_config.yaml')
        except Exception:
            config_path = 'config/mission_config.yaml'
        
        possible_paths = [
            config_path,
            os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'mission_config.yaml'),
            os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config', 'mission_config.yaml'),
        ]
        
        for path in possible_paths:
            try:
                if os.path.exists(path):
                    with open(path, 'r') as f:
                        self.config = yaml.safe_load(f) or {}
                        self.logger.info("config_loaded", extra={"path": path})
                        return
            except Exception as e:
                self.logger.warning("config_load_failed", extra={
                    "path": path,
                    "error": str(e)
                })
        
        self.logger.warning("using_default_config", extra={
            "reason": "Failed to load mission config"
        })
        self.config = {
            'timeout': 300,
            'max_retries': 3,
            'search_altitude': 10.0,
            'approach_speed': 2.0
        }
        
    # ----------------------------------------------------------------------
    # Deferred MetricsCollector + metric creation
    # ----------------------------------------------------------------------
    def _init_metrics(self):
        """Create MetricsCollector and all metrics AFTER pub/sub exist."""
        self.logger.debug("initializing_metrics")
        
        try:
            from drone_control.utils.metrics_collector import MetricsCollector
            self.metrics = MetricsCollector("mission_manager", port=8001)
        except Exception as e:
            self.logger.warning("metrics_collector_unavailable", extra={"error": str(e)})
            self.metrics = None
            return
        
        try:
            self.metric_state = self.metrics.gauge(
                "current_state",
                "Current mission state (0=IDLE, 1=SEARCHING, 2=TRACKING, 3=ENGAGING, 4=ATTACKING, 5=COMPLETED, 6=FAILED, 7=EMERGENCY)",
                labels=["state", "mission_id"]
            )
            
            self.metric_mission_started = self.metrics.counter(
                "missions_started_total",
                "Total number of missions started",
                labels=["mission_id"]
            )
            self.metric_mission_completed = self.metrics.counter(
                "missions_completed_total",
                "Total number of missions completed",
                labels=["mission_id", "success"]
            )
            self.metric_state_transitions = self.metrics.counter(
                "state_transitions_total",
                "Total number of state transitions",
                labels=["from_state", "to_state", "mission_id"]
            )
            self.metric_targets_tracked = self.metrics.counter(
                "targets_tracked_total",
                "Total number of targets tracked",
                labels=["mission_id"]
            )
            
            self.metric_mission_duration = self.metrics.histogram(
                "mission_duration_seconds",
                "Duration of missions in seconds",
                labels=["mission_id", "outcome"],
                buckets=[10, 30, 60, 120, 180, 300, 600]
            )
            self.metric_target_acquisition_time = self.metrics.histogram(
                "target_acquisition_seconds",
                "Time to acquire target in seconds",
                labels=["mission_id"],
                buckets=[1, 2, 5, 10, 15, 30, 60]
            )
            self.metric_state_duration = self.metrics.histogram(
                "state_duration_seconds",
                "Duration spent in each state",
                labels=["state", "mission_id"],
                buckets=[1, 5, 10, 30, 60, 120, 300]
            )
            
            self.logger.info("metrics_initialized")
        except Exception as e:
            self.logger.warning("metric_creation_failed", extra={"error": str(e)})
        
    def _setup_transitions(self):
        """Configure state machine transitions"""
        self.logger.debug("setting_up_transitions")
        
        self.transitions = [
            Transition(MissionState.IDLE, MissionState.INITIALIZING,
                      MissionEvent.START, action=self._on_initialize),
            Transition(MissionState.INITIALIZING, MissionState.SEARCHING,
                      MissionEvent.TARGET_DETECTED, condition=self._is_initialized,
                      action=self._on_start_search),
            Transition(MissionState.SEARCHING, MissionState.TRACKING,
                      MissionEvent.TARGET_DETECTED, condition=self._is_valid_target,
                      action=self._on_start_tracking),
            Transition(MissionState.TRACKING, MissionState.ENGAGING,
                      MissionEvent.TARGET_ENGAGED, condition=self._is_within_engagement_distance,
                      action=self._on_engage),
            Transition(MissionState.ENGAGING, MissionState.ATTACKING,
                      MissionEvent.TARGET_ENGAGED, condition=self._is_within_attack_distance,
                      action=self._on_attack),
            Transition(MissionState.ATTACKING, MissionState.COMPLETED,
                      MissionEvent.TARGET_DESTROYED, action=self._on_complete),
            Transition(MissionState.NONE, MissionState.EMERGENCY,
                      MissionEvent.EMERGENCY, priority=100, action=self._on_emergency),
            Transition(MissionState.NONE, MissionState.PAUSED,
                      MissionEvent.PAUSE, priority=50, action=self._on_pause),
            Transition(MissionState.PAUSED, MissionState.SEARCHING,
                      MissionEvent.RESUME, action=self._on_resume),
            Transition(MissionState.TRACKING, MissionState.SEARCHING,
                      MissionEvent.TARGET_LOST, action=self._on_target_lost),
        ]
        
        self.logger.debug("transitions_configured", extra={
            "count": len(self.transitions)
        })
        
    def _set_state(self, new_state: MissionState):
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = time.time()
            
            self.logger.info("state_changed", extra={
                "correlation_id": self.correlation_id,
                "from_state": self.previous_state.value,
                "to_state": self.current_state.value,
                "elapsed": time.time() - self.state_start_time
            })
            
            self._record_state_transition(self.previous_state, new_state)
            self._publish_status()
            self._update_state_metric(new_state)
            
    def _record_state_transition(self, from_state: MissionState, to_state: MissionState):
        if not self.metric_state_transitions:
            return
        mission_id = self.mission_id or self.correlation_id
        try:
            self.metric_state_transitions.labels(
                from_state=from_state.value,
                to_state=to_state.value,
                mission_id=mission_id
            ).inc()
        except Exception as e:
            self.logger.debug("state_transition_metric_failed", extra={"error": str(e)})
        
        if self.metric_state_duration:
            try:
                duration = time.time() - self.state_start_time
                self.metric_state_duration.labels(
                    state=from_state.value,
                    mission_id=mission_id
                ).observe(duration)
            except Exception as e:
                self.logger.debug("state_duration_metric_failed", extra={"error": str(e)})
        
    def _update_state_metric(self, state: MissionState):
        if not self.metric_state:
            return
        mission_id = self.mission_id or self.correlation_id
        
        state_map = {
            MissionState.IDLE: 0,
            MissionState.INITIALIZING: 0,
            MissionState.SEARCHING: 1,
            MissionState.TRACKING: 2,
            MissionState.ENGAGING: 3,
            MissionState.ATTACKING: 4,
            MissionState.COMPLETED: 5,
            MissionState.FAILED: 6,
            MissionState.EMERGENCY: 7,
            MissionState.PAUSED: 0,
            MissionState.RETURNING: 0,
            MissionState.LANDING: 0,
            MissionState.NONE: -1,
        }
        
        try:
            self.metric_state.labels(
                state=state.value,
                mission_id=mission_id
            ).set(state_map.get(state, -1))
        except Exception as e:
            self.logger.debug("state_metric_update_failed", extra={"error": str(e)})
        
    def _process_event(self, event: MissionEvent, data: Optional[Dict] = None) -> bool:
        self.logger.debug("processing_event", extra={
            "event": event.value,
            "current_state": self.current_state.value,
            "correlation_id": self.correlation_id
        })
        
        if event in [MissionEvent.EMERGENCY, MissionEvent.SAFETY_VIOLATION]:
            for transition in self.transitions:
                if transition.event == event and transition.priority >= 50:
                    if transition.from_state == MissionState.NONE or transition.from_state == self.current_state:
                        if not transition.condition or transition.condition(data):
                            self.logger.warning("emergency_event_processed", extra={
                                "event": event.value,
                                "to_state": transition.to_state.value
                            })
                            self._set_state(transition.to_state)
                            if transition.action:
                                transition.action(data)
                            return True
        
        for transition in self.transitions:
            if transition.event == event:
                if transition.from_state == MissionState.NONE or transition.from_state == self.current_state:
                    if not transition.condition or transition.condition(data):
                        self._set_state(transition.to_state)
                        if transition.action:
                            transition.action(data)
                        return True
                        
        self.logger.debug("unhandled_event", extra={
            "event": event.value,
            "state": self.current_state.value
        })
        return False
        
    def _update(self, event):
        elapsed = time.time() - self.state_start_time
        
        if self.current_state == MissionState.SEARCHING and elapsed > self.config.get('timeout', 300):
            self.logger.warning("search_timeout", extra={
                "elapsed": elapsed,
                "timeout": self.config.get('timeout', 300)
            })
            self._process_event(MissionEvent.ERROR, {"message": "Search timeout"})
            
        if self.current_state == MissionState.TRACKING and elapsed > self.config.get('timeout', 300):
            self.logger.warning("tracking_timeout", extra={
                "elapsed": elapsed,
                "timeout": self.config.get('timeout', 300)
            })
            self._process_event(MissionEvent.ERROR, {"message": "Tracking timeout"})
            
    def _publish_status(self):
        status_msg = MissionStatus()
        status_msg.mission_id = self.mission_id or "unknown"
        status_msg.state = self.current_state.value
        status_msg.previous_state = self.previous_state.value
        status_msg.elapsed_time = time.time() - self.state_start_time
        status_msg.current_waypoint = self.current_waypoint_index
        status_msg.total_waypoints = len(self.waypoints)
        status_msg.target_count = 1 if self.target_data else 0
        self.status_pub.publish(status_msg)
        
        self.logger.debug("status_published", extra={
            "state": status_msg.state,
            "elapsed": status_msg.elapsed_time
        })
        
    def _publish_health(self, event=None):
        is_healthy = self.current_state != MissionState.EMERGENCY
        
        # Lazy import of psutil
        try:
            import psutil
            cpu = psutil.cpu_percent()
            mem = psutil.virtual_memory().percent
        except ImportError:
            cpu = 0.0
            mem = 0.0
        
        health_msg = NodeHealth()
        health_msg.node_name = 'mission_manager'
        health_msg.status = self.current_state.value
        health_msg.timestamp = rospy.Time.now()
        health_msg.is_healthy = is_healthy
        health_msg.cpu_usage = cpu
        health_msg.memory_usage = mem

        self.health_pub.publish(health_msg)
        
        if not is_healthy:
            self.logger.warning("node_unhealthy", extra={
                "state": self.current_state.value
            })
        
    # ------------------------------------------------------------------
    # ACTION METHODS (unchanged logic, just guarded metric access)
    # ------------------------------------------------------------------
    
    def _on_initialize(self, data: Optional[Dict] = None):
        self.mission_id = get_or_create_mission_id()
        self.correlation_id = self.mission_id
        self.mission_data = data or {}
        self.mission_start_time = time.time()
        
        if self.metric_mission_started:
            try:
                self.metric_mission_started.labels(mission_id=self.mission_id).inc()
            except Exception:
                pass
        
        self.logger.info("mission_initialized", extra={
            "mission_id": self.mission_id,
            "correlation_id": self.correlation_id,
            "mission_data": self.mission_data
        })
        
    def _on_start_search(self, data: Optional[Dict] = None):
        self.logger.info("search_started", extra={
            "correlation_id": self.correlation_id,
            "altitude": self.config.get('search_altitude', 10.0)
        })
        self._acquisition_start_time = time.time()
        
    def _on_start_tracking(self, data: Optional[Dict] = None):
        if data and 'target' in data:
            self.target_data = data['target']
            self.logger.info("tracking_started", extra={
                "correlation_id": self.correlation_id,
                "target_id": self.target_data.get('id', 'unknown'),
                "confidence": self.target_data.get('confidence', 0),
                "distance": self.target_data.get('distance', 0)
            })
            
            if self.metric_targets_tracked:
                try:
                    self.metric_targets_tracked.labels(
                        mission_id=self.mission_id or self.correlation_id
                    ).inc()
                except Exception:
                    pass
            
            if self._acquisition_start_time is not None and self.metric_target_acquisition_time:
                try:
                    acquisition_time = time.time() - self._acquisition_start_time
                    self.metric_target_acquisition_time.labels(
                        mission_id=self.mission_id or self.correlation_id
                    ).observe(acquisition_time)
                    self.logger.info("target_acquired", extra={
                        "correlation_id": self.correlation_id,
                        "acquisition_time": acquisition_time
                    })
                except Exception:
                    pass
                self._acquisition_start_time = None
        
    def _on_engage(self, data: Optional[Dict] = None):
        self.logger.info("engaging_target", extra={
            "correlation_id": self.correlation_id,
            "target_id": self.target_data.get('id', 'unknown')
        })
        
    def _on_attack(self, data: Optional[Dict] = None):
        self.logger.warning("ATTACKING_TARGET", extra={
            "correlation_id": self.correlation_id,
            "target_id": self.target_data.get('id', 'unknown')
        })
        
    def _on_complete(self, data: Optional[Dict] = None):
        duration = time.time() - getattr(self, 'mission_start_time', time.time())
        
        self.logger.info("mission_completed", extra={
            "correlation_id": self.correlation_id,
            "mission_id": self.mission_id,
            "duration": duration
        })
        
        if self.metric_mission_duration:
            try:
                self.metric_mission_duration.labels(
                    mission_id=self.mission_id or self.correlation_id,
                    outcome="success"
                ).observe(duration)
            except Exception:
                pass
        
        if self.metric_mission_completed:
            try:
                self.metric_mission_completed.labels(
                    mission_id=self.mission_id or self.correlation_id,
                    success="true"
                ).inc()
            except Exception:
                pass
        
    def _on_emergency(self, data: Optional[Dict] = None):
        duration = time.time() - getattr(self, 'mission_start_time', time.time())
        
        self.logger.critical("EMERGENCY_TRIGGERED", extra={
            "correlation_id": self.correlation_id,
            "mission_id": self.mission_id,
            "reason": data.get('message', 'Unknown emergency') if data else 'Unknown emergency',
            "duration": duration
        })
        
        if self.metric_mission_duration:
            try:
                self.metric_mission_duration.labels(
                    mission_id=self.mission_id or self.correlation_id,
                    outcome="emergency"
                ).observe(duration)
            except Exception:
                pass
        
        if self.metric_mission_completed:
            try:
                self.metric_mission_completed.labels(
                    mission_id=self.mission_id or self.correlation_id,
                    success="false"
                ).inc()
            except Exception:
                pass
        
    def _on_pause(self, data: Optional[Dict] = None):
        self.logger.info("mission_paused", extra={
            "correlation_id": self.correlation_id
        })
        
    def _on_resume(self, data: Optional[Dict] = None):
        self.logger.info("mission_resumed", extra={
            "correlation_id": self.correlation_id
        })
        
    def _on_target_lost(self, data: Optional[Dict] = None):
        self.logger.warning("target_lost", extra={
            "correlation_id": self.correlation_id,
            "target_id": self.target_data.get('id', 'unknown')
        })
        self.target_data = {}
        
    # ------------------------------------------------------------------
    # CONDITION METHODS
    # ------------------------------------------------------------------
    
    def _is_initialized(self, data: Optional[Dict] = None) -> bool:
        is_initialized = rospy.has_param('/system_initialized')
        if not is_initialized:
            self.logger.debug("system_not_initialized")
        return is_initialized
        
    def _is_valid_target(self, data: Optional[Dict] = None) -> bool:
        if not data or 'target' not in data:
            return False
        target = data['target']
        is_valid = target.get('confidence', 0) > 0.5
        if not is_valid:
            self.logger.debug("invalid_target", extra={
                "confidence": target.get('confidence', 0)
            })
        return is_valid
        
    def _is_within_engagement_distance(self, data: Optional[Dict] = None) -> bool:
        if not self.target_data:
            return False
        distance = self.target_data.get('distance', 100)
        # Param read is safe – node is fully initialised
        attack_distance = rospy.get_param('/tracking/attack_distance', 2.0)
        is_within = distance < attack_distance
        if is_within:
            self.logger.debug("within_engagement_distance", extra={
                "distance": distance,
                "attack_distance": attack_distance
            })
        return is_within
        
    def _is_within_attack_distance(self, data: Optional[Dict] = None) -> bool:
        if not self.target_data:
            return False
        distance = self.target_data.get('distance', 100)
        attack_distance = rospy.get_param('/tracking/attack_distance', 2.0) * 0.5
        is_within = distance < attack_distance
        if is_within:
            self.logger.debug("within_attack_distance", extra={
                "distance": distance,
                "attack_distance": attack_distance
            })
        return is_within
        
    # ------------------------------------------------------------------
    # CALLBACKS
    # ------------------------------------------------------------------
    
    def _command_callback(self, msg):
        self.logger.debug("command_received", extra={
            "correlation_id": self.correlation_id,
            "command": msg.command
        })
        
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
        else:
            self.logger.warning("unknown_command", extra={
                "command": msg.command
            })
            
    def _safety_callback(self, msg):
        self.logger.debug("safety_event_received", extra={
            "correlation_id": self.correlation_id,
            "is_safe": msg.is_safe,
            "emergency_active": msg.emergency_active
        })
        
        if msg.emergency_active:
            self._process_event(MissionEvent.EMERGENCY, {'message': msg.emergency_reason or 'Emergency active'})
        elif msg.violations and len(msg.violations) > 0:
            self.logger.warning("safety_violations_detected", extra={
                "violations": msg.violations
            })
            self._process_event(MissionEvent.SAFETY_VIOLATION, {'message': ', '.join(msg.violations)})
        elif not msg.is_safe:
            self.logger.warning("unsafe_status", extra={
                "reason": "Safety status unsafe"
            })
            self._process_event(MissionEvent.SAFETY_VIOLATION, {'message': 'Safety status unsafe'})
            
    def _target_callback(self, msg):
        if msg.targets and len(msg.targets) > 0:
            target = msg.targets[0]
            self.target_data = {
                'id': target.id,
                'confidence': target.confidence,
                'distance': target.distance,
                'position': [target.position.x, target.position.y, target.position.z],
                'velocity': [target.velocity.x, target.velocity.y, target.velocity.z],
                'state': target.state
            }
            
            self.logger.debug("target_data_received", extra={
                "correlation_id": self.correlation_id,
                "target_id": self.target_data['id'],
                "confidence": self.target_data['confidence'],
                "distance": self.target_data['distance']
            })
            
            if self.current_state == MissionState.SEARCHING:
                if self._acquisition_start_time is None:
                    self._acquisition_start_time = time.time()
                    self.logger.debug("acquisition_timer_started", extra={
                        "correlation_id": self.correlation_id
                    })
                
                self._process_event(MissionEvent.TARGET_DETECTED, {'target': self.target_data})
            elif self.current_state == MissionState.TRACKING:
                if self._is_within_engagement_distance():
                    self._process_event(MissionEvent.TARGET_ENGAGED, {'target': self.target_data})


if __name__ == '__main__':
    try:
        manager = MissionManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass