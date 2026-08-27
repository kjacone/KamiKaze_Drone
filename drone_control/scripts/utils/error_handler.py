#!/usr/bin/env python3
"""
drone_control/scripts/utils/error_handler.py
Consistent error handling across all nodes
"""

import rospy
import traceback
from enum import Enum
from typing import Optional

class ErrorSeverity(Enum):
    """Error severity levels"""
    DEBUG = "debug"
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"
    FATAL = "fatal"

class ErrorCategory(Enum):
    """Error categories"""
    SYSTEM = "system"
    NETWORK = "network"
    SENSOR = "sensor"
    PROCESSING = "processing"
    CONTROLLER = "controller"
    SAFETY = "safety"
    CONFIGURATION = "configuration"
    RUNTIME = "runtime"

class ErrorHandler:
    """Consistent error handling with logging and recovery"""
    
    def __init__(self, node_name: str, node_instance=None):
        self.node_name = node_name
        self.node_instance = node_instance
        self.error_count = 0
        self.critical_error_count = 0
        self.last_error_time = 0
        self.error_history = []
        
    def handle_error(self, error: Exception, context: Optional[str] = None,
                    severity: ErrorSeverity = ErrorSeverity.ERROR,
                    category: ErrorCategory = ErrorCategory.RUNTIME,
                    recoverable: bool = True):
        """Handle an error with appropriate logging and recovery"""
        
        self.error_count += 1
        self.last_error_time = rospy.Time.now()
        self.error_history.append({
            'timestamp': self.last_error_time,
            'error': str(error),
            'context': context,
            'severity': severity.value,
            'category': category.value
        })
        
        # Log the error
        log_message = f"[{self.node_name}] {severity.value.upper()}: {error}"
        if context:
            log_message += f" | Context: {context}"
            
        if severity in [ErrorSeverity.DEBUG, ErrorSeverity.INFO]:
            rospy.loginfo(log_message)
        elif severity == ErrorSeverity.WARNING:
            rospy.logwarn(log_message)
        elif severity in [ErrorSeverity.ERROR, ErrorSeverity.CRITICAL]:
            rospy.logerr(log_message)
        elif severity == ErrorSeverity.FATAL:
            rospy.logfatal(log_message)
            
        # Log traceback for debugging
        if severity in [ErrorSeverity.ERROR, ErrorSeverity.CRITICAL, ErrorSeverity.FATAL]:
            rospy.logdebug(f"Traceback for {self.node_name}:\n{traceback.format_exc()}")
            
        # Handle critical errors
        if severity in [ErrorSeverity.CRITICAL, ErrorSeverity.FATAL]:
            self.critical_error_count += 1
            if self.critical_error_count > 3:
                rospy.logerr(f"[{self.node_name}] Too many critical errors ({self.critical_error_count})")
                
        # Attempt recovery if recoverable
        if recoverable and severity not in [ErrorSeverity.FATAL]:
            self._attempt_recovery(error, context)
            
        return self.critical_error_count
        
    def _attempt_recovery(self, error: Exception, context: Optional[str] = None):
        """Attempt to recover from error"""
        rospy.logdebug(f"[{self.node_name}] Attempting recovery from: {error}")
        
        # Generic recovery strategies
        if isinstance(error, (ConnectionError, TimeoutError)):
            rospy.logwarn(f"[{self.node_name}] Network error, attempting reconnection")
            
        elif isinstance(error, (ValueError, TypeError)):
            rospy.logwarn(f"[{self.node_name}] Invalid value, attempting to reset")
            
    def get_error_stats(self) -> dict:
        """Get error statistics"""
        return {
            'total_errors': self.error_count,
            'critical_errors': self.critical_error_count,
            'last_error_time': self.last_error_time,
            'recent_errors': self.error_history[-5:] if self.error_history else []
        }
        
    def reset(self):
        """Reset error counters"""
        self.error_count = 0
        self.critical_error_count = 0
        self.error_history = []