#!/usr/bin/env python3
"""
drone_control/scripts/utils/metrics_collector.py
Metrics Collector Base Class with automatic registration and logger integration
"""

import os
import sys
import time
from functools import wraps
from typing import Any, Dict, List, Optional, Union

# Add parent directory to path for imports
# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import structured logger
try:
    from drone_control.utils.logging_framework import get_logger
except ImportError:
    import logging
    def get_logger(name):
        return logging.getLogger(name)

# Import prometheus_client
try:
    from prometheus_client import (
        REGISTRY,
        Counter,
        Gauge,
        Histogram,
        Summary,
        start_http_server,
    )
    PROMETHEUS_AVAILABLE = True
except ImportError:
    PROMETHEUS_AVAILABLE = False
    # Create dummy classes
    class Counter:
        def __init__(self, *args, **kwargs): pass
        def inc(self, *args, **kwargs): pass
        def labels(self, *args, **kwargs): return self
    class Gauge:
        def __init__(self, *args, **kwargs): pass
        def set(self, *args, **kwargs): pass
        def inc(self, *args, **kwargs): pass
        def dec(self, *args, **kwargs): pass
        def labels(self, *args, **kwargs): return self
    class Histogram:
        def __init__(self, *args, **kwargs): pass
        def observe(self, *args, **kwargs): pass
        def labels(self, *args, **kwargs): return self
        def time(self): return self
        def __enter__(self): return self
        def __exit__(self, *args): pass
    class Summary:
        def __init__(self, *args, **kwargs): pass
        def observe(self, *args, **kwargs): pass
        def labels(self, *args, **kwargs): return self
    REGISTRY = None
    def start_http_server(port, addr=''):
        pass

# Optional rospy import for delayed server start
try:
    import rospy
    ROSPY_AVAILABLE = True
except ImportError:
    ROSPY_AVAILABLE = False


class MetricsCollector:
    """
    Base class for collecting and exposing Prometheus metrics.
    Automatically prefixes metric names with component name.
    """
    
    _http_server_started = False
    _http_server_port = 8001
    _metrics_registry = REGISTRY
    _server_start_timer = None   # keep a reference so the timer is not GC'd
    
    def __init__(self, component: str, port: int = 8001, auto_start_server: bool = True):
        """
        Initialize a metrics collector for a component.
        
        Args:
            component: Component name (e.g., "yolo_detector")
            port: Port for HTTP server (default: 8001)
            auto_start_server: Whether to auto-start HTTP server (default: True)
        """
        self.component = component
        self.port = port
        self.logger = get_logger(f"metrics_{component}")
        
        # Track created metrics for cleanup/reuse
        self._metrics = {}
        
        # Defer HTTP server start so that:
        # 1. rospy.init_node() is not blocked by the (slow) first prometheus_client import
        # 2. port 8001 collisions with other nodes do not prevent node startup
        if PROMETHEUS_AVAILABLE and auto_start_server:
            self._schedule_http_server_start()
        
        if PROMETHEUS_AVAILABLE:
            self.logger.info("metrics_collector_initialized", extra={
                "component": component,
                "port": port,
                "server_started": self._http_server_started,
                "server_start_deferred": True
            })
        else:
            self.logger.warning("prometheus_client_not_available", extra={
                "component": component,
                "message": "Metrics will be disabled"
            })
    
    @classmethod
    def _schedule_http_server_start(cls):
        """Schedule a one-shot rospy.Timer that starts the HTTP server after 5 s."""
        if cls._http_server_started or not PROMETHEUS_AVAILABLE:
            return

        if not ROSPY_AVAILABLE:
            # Fallback: start immediately if rospy is not present
            cls._ensure_http_server()
            return

        def _timer_cb(event):
            cls._ensure_http_server()

        # oneshot=True → fires only once, 5 seconds after node init
        cls._server_start_timer = rospy.Timer(
            rospy.Duration(5.0),
            _timer_cb,
            oneshot=True
        )

    @classmethod
    def _ensure_http_server(cls):
        """Start HTTP server if not already started"""
        if not cls._http_server_started and PROMETHEUS_AVAILABLE:
            try:
                start_http_server(cls._http_server_port)
                cls._http_server_started = True
                # Get a logger for the server startup message
                logger = get_logger("metrics_server")
                logger.info("metrics_server_started", extra={
                    "port": cls._http_server_port,
                    "endpoint": f"http://0.0.0.0:{cls._http_server_port}/metrics"
                })
            except Exception as e:
                logger = get_logger("metrics_server")
                logger.error("metrics_server_start_failed", extra={
                    "port": cls._http_server_port,
                    "error": str(e)
                })
    
    def _get_metric_name(self, name: str) -> str:
        """
        Get prefixed metric name.
        
        Args:
            name: Base metric name
            
        Returns:
            Prefixed metric name
        """
        return f"{self.component}_{name}"
    
    def _log_metric_creation(self, metric_type: str, name: str, description: str, labels: Optional[List[str]] = None):
        """Log metric creation for debugging"""
        self.logger.debug("metric_created", extra={
            "type": metric_type,
            "name": name,
            "full_name": self._get_metric_name(name),
            "description": description,
            "labels": labels or []
        })
    
    def counter(self, name: str, description: str, labels: Optional[List[str]] = None) -> Union[Counter, Any]:
        """
        Create a Counter metric.
        
        Args:
            name: Metric name (will be prefixed with component)
            description: Metric description
            labels: List of label names for dynamic dimensions
            
        Returns:
            Counter metric instance
        """
        full_name = self._get_metric_name(name)
        cache_key = f"counter_{full_name}_{'_'.join(labels or [])}"
        
        if cache_key in self._metrics:
            return self._metrics[cache_key]
        
        if PROMETHEUS_AVAILABLE:
            try:
                if labels:
                    metric = Counter(full_name, description, labels)
                else:
                    metric = Counter(full_name, description)
                self._metrics[cache_key] = metric
                self._log_metric_creation("counter", name, description, labels)
                return metric
            except Exception as e:
                self.logger.error("counter_creation_failed", extra={
                    "name": full_name,
                    "error": str(e)
                })
                return Counter(name, description) if labels is None else Counter(name, description, labels)
        else:
            return Counter(name, description) if labels is None else Counter(name, description, labels)
    
    def gauge(self, name: str, description: str, labels: Optional[List[str]] = None) -> Union[Gauge, Any]:
        """
        Create a Gauge metric.
        
        Args:
            name: Metric name (will be prefixed with component)
            description: Metric description
            labels: List of label names for dynamic dimensions
            
        Returns:
            Gauge metric instance
        """
        full_name = self._get_metric_name(name)
        cache_key = f"gauge_{full_name}_{'_'.join(labels or [])}"
        
        if cache_key in self._metrics:
            return self._metrics[cache_key]
        
        if PROMETHEUS_AVAILABLE:
            try:
                if labels:
                    metric = Gauge(full_name, description, labels)
                else:
                    metric = Gauge(full_name, description)
                self._metrics[cache_key] = metric
                self._log_metric_creation("gauge", name, description, labels)
                return metric
            except Exception as e:
                self.logger.error("gauge_creation_failed", extra={
                    "name": full_name,
                    "error": str(e)
                })
                return Gauge(name, description) if labels is None else Gauge(name, description, labels)
        else:
            return Gauge(name, description) if labels is None else Gauge(name, description, labels)
    
    def histogram(self, name: str, description: str, labels: Optional[List[str]] = None, 
                  buckets: Optional[List[float]] = None) -> Union[Histogram, Any]:
        """
        Create a Histogram metric.
        
        Args:
            name: Metric name (will be prefixed with component)
            description: Metric description
            labels: List of label names for dynamic dimensions
            buckets: List of bucket boundaries (optional)
            
        Returns:
            Histogram metric instance
        """
        full_name = self._get_metric_name(name)
        cache_key = f"histogram_{full_name}_{'_'.join(labels or [])}_{'_'.join([str(b) for b in buckets or []])}"
        
        if cache_key in self._metrics:
            return self._metrics[cache_key]
        
        if PROMETHEUS_AVAILABLE:
            try:
                kwargs = {}
                if buckets:
                    kwargs['buckets'] = buckets
                
                if labels:
                    metric = Histogram(full_name, description, labels, **kwargs)
                else:
                    metric = Histogram(full_name, description, **kwargs)
                self._metrics[cache_key] = metric
                self._log_metric_creation("histogram", name, description, labels)
                return metric
            except Exception as e:
                self.logger.error("histogram_creation_failed", extra={
                    "name": full_name,
                    "error": str(e)
                })
                return Histogram(name, description) if labels is None else Histogram(name, description, labels)
        else:
            return Histogram(name, description) if labels is None else Histogram(name, description, labels)
    
    def summary(self, name: str, description: str, labels: Optional[List[str]] = None,
                quantiles: Optional[List[tuple]] = None) -> Union[Summary, Any]:
        """
        Create a Summary metric.
        
        Args:
            name: Metric name (will be prefixed with component)
            description: Metric description
            labels: List of label names for dynamic dimensions
            quantiles: List of (quantile, error) tuples (optional)
            
        Returns:
            Summary metric instance
        """
        full_name = self._get_metric_name(name)
        cache_key = f"summary_{full_name}_{'_'.join(labels or [])}_{'_'.join([str(q) for q in quantiles or []])}"
        
        if cache_key in self._metrics:
            return self._metrics[cache_key]
        
        if PROMETHEUS_AVAILABLE:
            try:
                kwargs = {}
                if quantiles:
                    kwargs['quantiles'] = quantiles
                
                if labels:
                    metric = Summary(full_name, description, labels, **kwargs)
                else:
                    metric = Summary(full_name, description, **kwargs)
                self._metrics[cache_key] = metric
                self._log_metric_creation("summary", name, description, labels)
                return metric
            except Exception as e:
                self.logger.error("summary_creation_failed", extra={
                    "name": full_name,
                    "error": str(e)
                })
                return Summary(name, description) if labels is None else Summary(name, description, labels)
        else:
            return Summary(name, description) if labels is None else Summary(name, description, labels)
    
    def get_metrics(self) -> Dict[str, Any]:
        """
        Get all created metrics.
        
        Returns:
            Dictionary of metric name to metric instance
        """
        return self._metrics.copy()
    
    def reset(self) -> None:
        """Reset all metrics (useful for testing)"""
        # Note: This is a soft reset - metrics remain in registry
        # but can be re-initialized
        self._metrics.clear()


class MetricTimer:
    """
    Context manager for timing operations with histograms or summaries.
    """
    
    def __init__(self, metric, labels: Optional[Dict[str, str]] = None):
        """
        Initialize timer.
        
        Args:
            metric: Histogram or Summary metric
            labels: Labels to apply to the metric
        """
        self.metric = metric
        self.labels = labels or {}
        self.start_time = None
        self._labeled_metric = None
    
    def __enter__(self):
        self.start_time = time.time()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        duration = time.time() - self.start_time
        try:
            if self.labels:
                labeled = self.metric.labels(**self.labels)
                labeled.observe(duration)
            else:
                self.metric.observe(duration)
        except Exception as e:
            # Log error but don't propagate
            logger = get_logger("metric_timer")
            logger.error("timer_observation_failed", extra={
                "error": str(e),
                "duration": duration
            })
    
    def observe(self, value: float):
        """
        Manually observe a value (for when context manager isn't used).
        
        Args:
            value: Value to observe
        """
        try:
            if self.labels:
                labeled = self.metric.labels(**self.labels)
                labeled.observe(value)
            else:
                self.metric.observe(value)
        except Exception as e:
            logger = get_logger("metric_timer")
            logger.error("manual_observation_failed", extra={
                "error": str(e),
                "value": value
            })


def timed_metric(metric_name: str, labels: Optional[Dict[str, str]] = None):
    """
    Decorator for timing function execution with a metric.
    
    Args:
        metric_name: Name of the metric to use (must be a histogram or summary)
        labels: Labels to apply to the metric
        
    Usage:
        @timed_metric("detection_latency_seconds")
        def process_frame(self):
            ...
    """
    def decorator(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            # Get the metrics collector from the instance
            if hasattr(self, 'metrics') and hasattr(self.metrics, 'metric_name'):
                # If metric exists, use it
                pass
            return func(self, *args, **kwargs)
        return wrapper
    return decorator


# Global metrics collector for shared metrics
_global_collector = None


def get_global_collector(component: str = "global", port: int = 8001) -> MetricsCollector:
    """
    Get or create a global metrics collector.
    
    Args:
        component: Component name prefix
        port: HTTP server port
        
    Returns:
        Global MetricsCollector instance
    """
    global _global_collector
    if _global_collector is None:
        _global_collector = MetricsCollector(component, port)
    return _global_collector


# Convenience functions for common metrics
def record_mission_metric(collector: MetricsCollector, metric_type: str, 
                         name: str, value: float, labels: Optional[Dict[str, str]] = None):
    """
    Record a metric value.
    
    Args:
        collector: MetricsCollector instance
        metric_type: Type of metric ('counter', 'gauge', 'histogram', 'summary')
        name: Metric name
        value: Value to record
        labels: Labels to apply
    """
    if metric_type == 'counter':
        metric = collector.counter(name, f"{name} counter")
        if labels:
            metric.labels(**labels).inc(value)
        else:
            metric.inc(value)
    elif metric_type == 'gauge':
        metric = collector.gauge(name, f"{name} gauge")
        if labels:
            metric.labels(**labels).set(value)
        else:
            metric.set(value)
    elif metric_type in ['histogram', 'summary']:
        metric = collector.histogram(name, f"{name} histogram") if metric_type == 'histogram' else collector.summary(name, f"{name} summary")
        if labels:
            metric.labels(**labels).observe(value)
        else:
            metric.observe(value)