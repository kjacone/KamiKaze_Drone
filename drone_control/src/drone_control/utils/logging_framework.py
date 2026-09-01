#!/usr/bin/env python3
"""
drone_control/scripts/utils/logging_framework.py
Structured JSON logging framework with correlation ID support
"""

import json
import logging
import os
import sys
import time
import traceback
from datetime import datetime
from typing import Any, Dict, Optional, Union

import rospy


class StructuredJSONFormatter(logging.Formatter):
    """
    Custom JSON formatter for structured logging.
    Outputs logs as JSON with consistent fields.
    """
    
    def __init__(self, component_name: str = "unknown"):
        super().__init__()
        self.component_name = component_name
        self._node_name = None
        
    def get_node_name(self) -> str:
        """Get the ROS node name if available"""
        if self._node_name is None:
            try:
                self._node_name = rospy.get_name() if rospy.core.is_initialized() else "unknown"
            except:
                self._node_name = "unknown"
        return self._node_name
    
    def get_mission_id(self) -> str:
        """Get mission_id from ROS parameter if available"""
        try:
            if rospy.core.is_initialized() and rospy.has_param("/mission_id"):
                return rospy.get_param("/mission_id")
        except:
            pass
        return "unknown"
    
    def get_correlation_id(self) -> str:
        """Get correlation_id from ROS parameter if available"""
        try:
            if rospy.core.is_initialized() and rospy.has_param("/correlation_id"):
                return rospy.get_param("/correlation_id")
        except:
            pass
        return "unknown"
    
    def format(self, record: logging.LogRecord) -> str:
        """
        Format the log record as JSON with standard fields.
        
        Args:
            record: The log record to format
            
        Returns:
            JSON string representation of the log
        """
        # Get the exception info if present
        exc_info = None
        if record.exc_info:
            exc_info = {
                "type": record.exc_info[0].__name__ if record.exc_info[0] else None,
                "message": str(record.exc_info[1]) if record.exc_info[1] else None,
                "traceback": ''.join(traceback.format_tb(record.exc_info[2])) if record.exc_info[2] else None
            }
        
        # Get extra fields from the record
        extra = getattr(record, 'extra', {})
        if not isinstance(extra, dict):
            extra = {}
        
        # Build the log entry
        log_entry = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "timestamp_unix": time.time(),
            "level": record.levelname,
            "component": self.component_name,
            "node": self.get_node_name(),
            "mission_id": self.get_mission_id(),
            "correlation_id": self.get_correlation_id(),
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
            "extra": extra,
        }
        
        # Add exception info if present
        if exc_info:
            log_entry["exception"] = exc_info
        
        # Add any custom attributes from the record
        for key, value in record.__dict__.items():
            if key not in ["args", "asctime", "created", "exc_info", "exc_text", 
                          "filename", "funcName", "levelname", "levelno", "lineno",
                          "module", "msecs", "message", "msg", "name", "pathname",
                          "process", "processName", "relativeCreated", "stack_info",
                          "thread", "threadName", "extra"]:
                if key not in log_entry:
                    log_entry[key] = value
        
        return json.dumps(log_entry, separators=(',', ':')) + "\n"


class StructuredLogger:
    """
    Structured logger wrapper that provides convenience methods for
    logging with consistent JSON formatting.
    """
    
    def __init__(self, component_name: str = "unknown", level: int = logging.INFO):
        """
        Initialize a structured logger.
        
        Args:
            component_name: Name of the component (e.g., "yolo_detector")
            level: Logging level (default: logging.INFO)
        """
        self.component_name = component_name
        self.logger = logging.getLogger(f"drone_control.{component_name}")
        self.logger.setLevel(level)
        self.logger.propagate = False
        
        # Remove existing handlers to avoid duplicates
        if self.logger.handlers:
            self.logger.handlers.clear()
        
        # Create console handler with JSON formatter
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        json_formatter = StructuredJSONFormatter(component_name)
        console_handler.setFormatter(json_formatter)
        self.logger.addHandler(console_handler)
        
        # Also add a simple console handler for development (optional)
        # This can be enabled by setting LOG_SIMPLE_CONSOLE=1
        if os.environ.get('LOG_SIMPLE_CONSOLE', '').lower() in ['1', 'true', 'yes']:
            simple_handler = logging.StreamHandler(sys.stdout)
            simple_handler.setLevel(level)
            simple_formatter = logging.Formatter(
                '%(asctime)s [%(levelname)s] %(name)s: %(message)s'
            )
            simple_handler.setFormatter(simple_formatter)
            self.logger.addHandler(simple_handler)
    
    def _log(self, level: int, message: str, extra: Optional[Dict[str, Any]] = None, 
             **kwargs) -> None:
        """
        Internal logging method that handles extra fields.
        
        Args:
            level: Logging level
            message: Log message
            extra: Additional context dictionary
            **kwargs: Additional keyword arguments to add to the log
        """
        # Combine extra and kwargs
        context = {}
        if extra:
            context.update(extra)
        if kwargs:
            context.update(kwargs)
        
        # Create log record with extra fields
        self.logger.log(level, message, extra={'extra': context})
    
    def debug(self, message: str, extra: Optional[Dict[str, Any]] = None, **kwargs) -> None:
        """Log a debug message"""
        self._log(logging.DEBUG, message, extra, **kwargs)
    
    def info(self, message: str, extra: Optional[Dict[str, Any]] = None, **kwargs) -> None:
        """Log an info message"""
        self._log(logging.INFO, message, extra, **kwargs)
    
    def warning(self, message: str, extra: Optional[Dict[str, Any]] = None, **kwargs) -> None:
        """Log a warning message"""
        self._log(logging.WARNING, message, extra, **kwargs)
    
    def warn(self, message: str, extra: Optional[Dict[str, Any]] = None, **kwargs) -> None:
        """Log a warning message (alias for warning)"""
        self.warning(message, extra, **kwargs)
    
    def error(self, message: str, extra: Optional[Dict[str, Any]] = None, **kwargs) -> None:
        """Log an error message"""
        self._log(logging.ERROR, message, extra, **kwargs)
    
    def critical(self, message: str, extra: Optional[Dict[str, Any]] = None, **kwargs) -> None:
        """Log a critical message"""
        self._log(logging.CRITICAL, message, extra, **kwargs)
    
    def exception(self, message: str, extra: Optional[Dict[str, Any]] = None, **kwargs) -> None:
        """
        Log an exception with stack trace.
        
        Args:
            message: Log message
            extra: Additional context dictionary
            **kwargs: Additional keyword arguments
        """
        context = {}
        if extra:
            context.update(extra)
        if kwargs:
            context.update(kwargs)
        
        self.logger.error(message, exc_info=True, extra={'extra': context})
    
    def log_with_mission_id(self, level: int, message: str, 
                           mission_id: Optional[str] = None, 
                           extra: Optional[Dict[str, Any]] = None) -> None:
        """
        Log a message with a specific mission_id.
        
        Args:
            level: Logging level
            message: Log message
            mission_id: Mission ID (if None, will try to get from ROS param)
            extra: Additional context dictionary
        """
        context = extra or {}
        if mission_id:
            context['mission_id'] = mission_id
        self._log(level, message, context)
    
    def info_with_mission(self, message: str, mission_id: Optional[str] = None,
                         extra: Optional[Dict[str, Any]] = None, **kwargs) -> None:
        """Log an info message with mission_id"""
        context = extra or {}
        if mission_id:
            context['mission_id'] = mission_id
        self._log(logging.INFO, message, context, **kwargs)
    
    def error_with_mission(self, message: str, mission_id: Optional[str] = None,
                          extra: Optional[Dict[str, Any]] = None, **kwargs) -> None:
        """Log an error message with mission_id"""
        context = extra or {}
        if mission_id:
            context['mission_id'] = mission_id
        self._log(logging.ERROR, message, context, **kwargs)
    
    def set_level(self, level: Union[str, int]) -> None:
        """
        Set the logging level.
        
        Args:
            level: Logging level (e.g., "DEBUG", logging.DEBUG)
        """
        if isinstance(level, str):
            level = getattr(logging, level.upper(), logging.INFO)
        self.logger.setLevel(level)
        for handler in self.logger.handlers:
            handler.setLevel(level)


# Module-level logger cache
_logger_cache = {}


def get_logger(component_name: str = "unknown", level: int = logging.INFO) -> StructuredLogger:
    """
    Get or create a structured logger instance.
    
    Args:
        component_name: Name of the component (e.g., "yolo_detector")
        level: Logging level (default: logging.INFO)
        
    Returns:
        StructuredLogger instance
    """
    cache_key = f"{component_name}_{level}"
    if cache_key not in _logger_cache:
        _logger_cache[cache_key] = StructuredLogger(component_name, level)
    return _logger_cache[cache_key]


def get_logger_with_ros_level(component_name: str = "unknown") -> StructuredLogger:
    """
    Get a logger with level configured from ROS parameter.
    Expects parameter: /logging/level or /<component>/log_level
    
    Args:
        component_name: Name of the component
        
    Returns:
        StructuredLogger instance with configured level
    """
    # Try to get level from ROS parameters
    default_level = logging.INFO
    level = default_level
    
    try:
        if rospy.core.is_initialized():
            # Check global logging level
            if rospy.has_param('/logging/level'):
                level_name = rospy.get_param('/logging/level', 'INFO').upper()
                level = getattr(logging, level_name, logging.INFO)
            
            # Check component-specific level
            if rospy.has_param(f'/{component_name}/log_level'):
                level_name = rospy.get_param(f'/{component_name}/log_level', 'INFO').upper()
                level = getattr(logging, level_name, level)
            
            # Check debug mode
            if rospy.get_param('/debug_mode', False):
                level = logging.DEBUG
    except:
        pass
    
    return get_logger(component_name, level)


# Convenience function to replace rospy.log* calls
def log_info(message: str, **kwargs) -> None:
    """Log info using the root logger"""
    logger = get_logger("root")
    logger.info(message, **kwargs)


def log_warn(message: str, **kwargs) -> None:
    """Log warning using the root logger"""
    logger = get_logger("root")
    logger.warning(message, **kwargs)


def log_error(message: str, **kwargs) -> None:
    """Log error using the root logger"""
    logger = get_logger("root")
    logger.error(message, **kwargs)


def log_debug(message: str, **kwargs) -> None:
    """Log debug using the root logger"""
    logger = get_logger("root")
    logger.debug(message, **kwargs)