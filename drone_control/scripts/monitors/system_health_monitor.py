#!/usr/bin/env python3
"""
drone_control/scripts/monitors/system_health_monitor.py
Comprehensive system health monitoring and resource management,
with structured logging and metrics (mirrors yolo_detector.py's pattern).
"""

import json
import socket
import subprocess
import time
from enum import Enum

import psutil
import rospy
from std_msgs.msg import String

from drone_control.utils.logging_framework import get_logger_with_ros_level
from drone_control.utils.metrics_collector import MetricTimer, MetricsCollector
from drone_control.msg import DiagnosticStatus, NodeHealth
from drone_control.utils import ErrorHandler


# Import distributed tracing utilities. Tried package-qualified first
# (matches how ros_tracing.py itself imports tracing.py, and how other
# nodes such as health_checker.py import these), then bare module names
# (matches this file's own sys.path.append(.../utils) convention above),
# then falls back to no-op stand-ins so this node still runs if the
# tracing package isn't installed -- same defensive pattern used for
# ErrorHandler/get_logger/MetricsCollector above.
try:
    from drone_control import create_traced_publisher, traced_ros_callback
    from drone_control import get_component_tracer, init_tracing, span_context, traced
except ImportError:
    try:
        from ros_tracing import create_traced_publisher, traced_ros_callback
        from tracing import get_component_tracer, init_tracing, span_context, traced
    except ImportError:
        def init_tracing(*args, **kwargs):
            return False

        class _DummySpan:
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def set_attribute(self, *args): pass
            def set_status(self, *args): pass
            def record_exception(self, *args): pass

        def get_component_tracer(name):
            return None

        def span_context(*args, **kwargs):
            return _DummySpan()

        def traced(tracer_name, span_name=None, attributes=None):
            def decorator(func):
                return func
            return decorator

        def traced_ros_callback(tracer_name, span_name=None):
            def decorator(func):
                return func
            return decorator

        def create_traced_publisher(topic, msg_type, queue_size=10, tracer_name=None):
            return rospy.Publisher(topic, msg_type, queue_size=queue_size)


class SystemHealthStatus(Enum):
    HEALTHY = "healthy"
    WARNING = "warning"
    CRITICAL = "critical"
    UNKNOWN = "unknown"


class SystemHealthMonitor:
    """Comprehensive system health monitoring and resource management"""

    def __init__(self):
        rospy.init_node('system_health_monitor', anonymous=False)

        # Initialize structured logger
        self.logger = get_logger_with_ros_level("system_health_monitor")
        self.logger.info("node_initializing", extra={
            "version": "1.0.0",
            "simulation_mode": rospy.get_param('/use_simulation', True)
        })

        self.error_handler = ErrorHandler(node_name='system_health_monitor')

        # Initialize distributed tracing before anything else registers
        # publishers/subscribers/timers so every downstream call has a
        # tracer available.
        init_tracing(component='system_health_monitor')
        self.tracer = get_component_tracer('system_health_monitor')

        # Configuration parameters
        self.monitoring_interval = rospy.get_param('~monitoring_interval', 1.0)
        self.critical_threshold_cpu = rospy.get_param('~critical_threshold_cpu', 90.0)
        self.critical_threshold_memory = rospy.get_param('~critical_threshold_memory', 90.0)
        self.warning_threshold_cpu = rospy.get_param('~warning_threshold_cpu', 70.0)
        self.warning_threshold_memory = rospy.get_param('~warning_threshold_memory', 70.0)
        self.enable_network_monitoring = rospy.get_param('~enable_network_monitoring', True)
        self.enable_disk_monitoring = rospy.get_param('~enable_disk_monitoring', True)
        self.enable_process_monitoring = rospy.get_param('~enable_process_monitoring', True)
        self.simulation_mode = rospy.get_param('/use_simulation', True)

        # Initialize metrics collector (parameters above must be set first,
        # since _init_metrics() reads self.monitoring_interval etc.)
        self.metrics = MetricsCollector("system_health_monitor", port=8002)
        self._init_metrics()

        # State variables
        self.system_health = SystemHealthStatus.UNKNOWN
        self.cpu_info = {}
        self.memory_info = {}
        self.network_info = {}
        self.disk_info = {}
        self.process_info = {}
        self.last_check_time = 0.0
        self.health_history = []
        self.critical_issues = []
        self.warning_issues = []
        self.service_status = {}
        self.service_warning_issues = []
        self.performance_metrics = {}
        self.check_count = 0
        self.error_count = 0

        # Subscribers
        self.health_check_sub = rospy.Subscriber('/health/check_all', String, self.check_all_callback)

        # Publishers (traced so publishes show up as spans alongside the
        # health-check cycle that produced them)
        self.health_pub = create_traced_publisher(
            '/system_health', NodeHealth, queue_size=10, tracer_name='system_health_monitor'
        )
        self.diagnostic_pub = create_traced_publisher(
            '/diagnostic_status', DiagnosticStatus, queue_size=10, tracer_name='system_health_monitor'
        )
        self.performance_pub = create_traced_publisher(
            '/performance_metrics', String, queue_size=10, tracer_name='system_health_monitor'
        )
        self.alert_pub = create_traced_publisher(
            '/system_alert', String, queue_size=10, tracer_name='system_health_monitor'
        )

        # Timers
        self.health_check_timer = rospy.Timer(rospy.Duration(self.monitoring_interval), self.health_check_loop)
        self.background_monitor_timer = rospy.Timer(rospy.Duration(30.0), self.background_monitor)

        self.logger.info("node_initialized", extra={
            "monitoring_interval": self.monitoring_interval,
            "critical_threshold_cpu": self.critical_threshold_cpu,
            "critical_threshold_memory": self.critical_threshold_memory,
            "simulation_mode": self.simulation_mode
        })
        rospy.loginfo("System Health Monitor initialized")

    def _init_metrics(self):
        """Initialize Prometheus metrics for this node"""
        self.logger.debug("initializing_metrics")

        # Counters
        self.metric_health_checks_total = self.metrics.counter(
            "health_checks_total",
            "Total number of health check cycles run"
        )
        self.metric_critical_issues_total = self.metrics.counter(
            "critical_issues_total",
            "Total number of critical issues detected",
            labels=["issue_type"]
        )
        self.metric_warning_issues_total = self.metrics.counter(
            "warning_issues_total",
            "Total number of warning issues detected",
            labels=["issue_type"]
        )
        self.metric_errors_total = self.metrics.counter(
            "errors_total",
            "Total number of errors encountered",
            labels=["error_type"]
        )
        self.metric_service_check_failures_total = self.metrics.counter(
            "service_check_failures_total",
            "Total number of failed service status checks",
            labels=["service"]
        )
        self.metric_network_check_failures_total = self.metrics.counter(
            "network_check_failures_total",
            "Total number of failed network connectivity checks",
            labels=["endpoint"]
        )

        # Gauges
        self.metric_cpu_usage = self.metrics.gauge(
            "cpu_usage_percent",
            "Current CPU usage percentage"
        )
        self.metric_memory_usage = self.metrics.gauge(
            "memory_usage_percent",
            "Current memory usage percentage"
        )
        self.metric_swap_usage = self.metrics.gauge(
            "swap_usage_percent",
            "Current swap usage percentage"
        )
        self.metric_cpu_temperature = self.metrics.gauge(
            "cpu_temperature_celsius",
            "Current CPU temperature in Celsius"
        )
        self.metric_disk_usage = self.metrics.gauge(
            "disk_usage_percent",
            "Current disk usage percentage",
            labels=["mountpoint"]
        )
        self.metric_process_count = self.metrics.gauge(
            "process_count",
            "Total number of running processes"
        )
        self.metric_thread_count = self.metrics.gauge(
            "thread_count",
            "Total number of running threads"
        )
        self.metric_service_up = self.metrics.gauge(
            "service_up",
            "Whether a monitored service is running (1=running, 0=stopped/unknown)",
            labels=["service"]
        )
        self.metric_health_status = self.metrics.gauge(
            "health_status",
            "Overall health status (1=healthy, 0=warning/critical)"
        )

        # Histograms
        self.metric_health_check_duration = self.metrics.histogram(
            "health_check_duration_seconds",
            "Time taken to complete a health check cycle in seconds",
            buckets=[0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5]
        )

        self.logger.info("metrics_initialized")

    def health_check_loop(self, event):
        """Main health check loop"""
        current_time = time.time()
        self.check_count += 1

        self.logger.debug("health_check_started", extra={
            "check_id": self.check_count
        })

        with span_context(self.tracer, "health_check_loop", {
            "check_id": self.check_count
        }) as span:
            with MetricTimer(self.metric_health_check_duration):
                # Collect system health metrics
                self.collect_system_metrics()

                # Check for issues
                self.check_for_issues()

                # Update overall status and publish it (this was previously never
                # triggered on a timer, so /system_health only updated on-demand)
                self.publish_status(event)

            span.set_attribute("status", self.system_health.value)
            span.set_attribute("critical_issue_count", len(self.critical_issues))
            span.set_attribute("warning_issue_count", len(self.warning_issues))

        self.metric_health_checks_total.inc()

        # Update health history (store the plain string value so this is
        # always JSON-serializable, not the Enum member itself)
        self.health_history.append({
            'timestamp': current_time,
            'health': self.system_health.value,
            'cpu_usage': self.cpu_info.get('percent', 0.0),
            'memory_usage': self.memory_info.get('percent', 0.0),
            'critical_issues': len(self.critical_issues),
            'warning_issues': len(self.warning_issues)
        })

        # Keep history manageable
        if len(self.health_history) > 1000:
            self.health_history.pop(0)

        self.logger.debug("health_check_completed", extra={
            "check_id": self.check_count,
            "status": self.system_health.value,
            "critical_issues": len(self.critical_issues),
            "warning_issues": len(self.warning_issues)
        })

    @traced('system_health_monitor', 'collect_system_metrics')
    def collect_system_metrics(self):
        """Collect system performance metrics"""
        current_time = time.time()

        # CPU metrics
        self.cpu_info = {
            'percent': psutil.cpu_percent(interval=None),
            'frequency': psutil.cpu_freq().current if psutil.cpu_freq() else 0.0,
            'core_count': psutil.cpu_count(logical=False),
            'logical_core_count': psutil.cpu_count(logical=True),
            'temperature': self.get_cpu_temperature(),
            'load_average': psutil.getloadavg() if hasattr(psutil, 'getloadavg') else [0.0, 0.0, 0.0]
        }
        self.metric_cpu_usage.set(self.cpu_info['percent'])
        if self.cpu_info['temperature'] is not None:
            self.metric_cpu_temperature.set(self.cpu_info['temperature'])

        # Memory metrics
        self.memory_info = {
            'percent': psutil.virtual_memory().percent,
            'total': psutil.virtual_memory().total,
            'available': psutil.virtual_memory().available,
            'used': psutil.virtual_memory().used,
            'swap_percent': psutil.swap_memory().percent,
            'swap_total': psutil.swap_memory().total,
            'swap_used': psutil.swap_memory().used
        }
        self.metric_memory_usage.set(self.memory_info['percent'])
        self.metric_swap_usage.set(self.memory_info['swap_percent'])

        # Disk metrics
        if self.enable_disk_monitoring:
            self.disk_info = {}
            for partition in psutil.disk_partitions():
                try:
                    usage = psutil.disk_usage(partition.mountpoint)
                    self.disk_info[partition.mountpoint] = {
                        'percent': usage.percent,
                        'total': usage.total,
                        'free': usage.free,
                        'used': usage.used
                    }
                    self.metric_disk_usage.labels(
                        mountpoint=partition.mountpoint
                    ).set(usage.percent)
                except Exception as e:
                    self.error_count += 1
                    self.metric_errors_total.labels(error_type="disk_usage_check").inc()
                    self.logger.warning("disk_usage_check_failed", extra={
                        "mountpoint": partition.mountpoint,
                        "error": str(e)
                    })
                    self.error_handler.handle_error(e, f"Disk usage check: {partition.mountpoint}")

        # Network metrics
        if self.enable_network_monitoring:
            net_io = psutil.net_io_counters()
            self.network_info = {
                'bytes_sent': net_io.bytes_sent,
                'bytes_recv': net_io.bytes_recv,
                'packets_sent': net_io.packets_sent,
                'packets_recv': net_io.packets_recv,
                'drop_in': net_io.dropin,
                'drop_out': net_io.dropout
            }

        # Process metrics
        if self.enable_process_monitoring:
            self.process_info = {
                'running_processes': len(list(psutil.process_iter())),
                'thread_count': self.get_total_thread_count(),
                'cpu_time': list(psutil.cpu_times()),
                'io_counters': psutil.disk_io_counters()._asdict() if psutil.disk_io_counters() else {}
            }
            self.metric_process_count.set(self.process_info['running_processes'])
            self.metric_thread_count.set(self.process_info['thread_count'])

        # Service status check (populates self.service_warning_issues,
        # which check_for_issues() merges in afterwards)
        self.check_service_status()

        self.last_check_time = current_time

        self.logger.debug("system_metrics_collected", extra={
            "cpu_percent": self.cpu_info.get('percent'),
            "memory_percent": self.memory_info.get('percent'),
            "process_count": self.process_info.get('running_processes')
        })

    def get_total_thread_count(self):
        """Count total threads across all running processes.

        `psutil._psplatform.util_processes()` does not exist in the public
        (or private) psutil API and would raise AttributeError. Use the
        documented Process.num_threads() instead, tolerating processes that
        disappear or are inaccessible mid-iteration.
        """
        total_threads = 0
        for proc in psutil.process_iter(['num_threads']):
            try:
                total_threads += proc.info.get('num_threads') or proc.num_threads()
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
        return total_threads

    @traced('system_health_monitor', 'check_for_issues')
    def check_for_issues(self):
        """Check for critical and warning issues"""
        self.critical_issues = []
        self.warning_issues = []

        # Check CPU
        if self.cpu_info.get('percent', 0.0) >= self.critical_threshold_cpu:
            msg = f"CPU usage critical: {self.cpu_info.get('percent', 0.0):.1f}%"
            self.critical_issues.append(msg)
            self.metric_critical_issues_total.labels(issue_type="cpu").inc()
            self.logger.warning("critical_issue_detected", extra={"issue": msg})
        elif self.cpu_info.get('percent', 0.0) >= self.warning_threshold_cpu:
            msg = f"CPU usage warning: {self.cpu_info.get('percent', 0.0):.1f}%"
            self.warning_issues.append(msg)
            self.metric_warning_issues_total.labels(issue_type="cpu").inc()
            self.logger.debug("warning_issue_detected", extra={"issue": msg})

        # Check memory
        if self.memory_info.get('percent', 0.0) >= self.critical_threshold_memory:
            msg = f"Memory usage critical: {self.memory_info.get('percent', 0.0):.1f}%"
            self.critical_issues.append(msg)
            self.metric_critical_issues_total.labels(issue_type="memory").inc()
            self.logger.warning("critical_issue_detected", extra={"issue": msg})
        elif self.memory_info.get('percent', 0.0) >= self.warning_threshold_memory:
            msg = f"Memory usage warning: {self.memory_info.get('percent', 0.0):.1f}%"
            self.warning_issues.append(msg)
            self.metric_warning_issues_total.labels(issue_type="memory").inc()
            self.logger.debug("warning_issue_detected", extra={"issue": msg})

        # Check disk space
        for mountpoint, info in self.disk_info.items():
            if info['percent'] >= 90:
                msg = f"Disk usage critical ({mountpoint}): {info['percent']:.1f}%"
                self.critical_issues.append(msg)
                self.metric_critical_issues_total.labels(issue_type="disk").inc()
                self.logger.warning("critical_issue_detected", extra={"issue": msg, "mountpoint": mountpoint})
            elif info['percent'] >= 80:
                msg = f"Disk usage warning ({mountpoint}): {info['percent']:.1f}%"
                self.warning_issues.append(msg)
                self.metric_warning_issues_total.labels(issue_type="disk").inc()
                self.logger.debug("warning_issue_detected", extra={"issue": msg, "mountpoint": mountpoint})

        # Check temperature (reuse the value already collected instead of
        # calling get_cpu_temperature() a second time)
        cpu_temp = self.cpu_info.get('temperature')
        if cpu_temp and cpu_temp >= 80.0:
            msg = f"CPU temperature warning: {cpu_temp:.1f}\u00b0C"
            self.warning_issues.append(msg)
            self.metric_warning_issues_total.labels(issue_type="temperature").inc()
            self.logger.debug("warning_issue_detected", extra={"issue": msg})

        # Check network connectivity
        if self.enable_network_monitoring:
            self.check_network_connectivity()

        # Merge in service-status warnings collected during
        # collect_system_metrics(); previously these were wiped out because
        # this method reset warning_issues *after* check_service_status()
        # had already appended to it.
        self.warning_issues.extend(self.service_warning_issues)

        self.metric_health_status.set(1 if len(self.critical_issues) == 0 else 0)

    @traced('system_health_monitor', 'check_service_status')
    def check_service_status(self):
        """Check status of critical services"""
        critical_services = [
            'ros', 'docker', 'px4', 'mavros', 'gazebo'
        ]

        self.service_warning_issues = []
        for service in critical_services:
            with span_context(self.tracer, f"check_service.{service}", {
                "service": service
            }) as span:
                try:
                    # Check if service is running
                    result = subprocess.run(['systemctl', 'is-active', service],
                                          capture_output=True, text=True, timeout=5)
                    if result.returncode == 0:
                        self.service_status[service] = 'running'
                        self.metric_service_up.labels(service=service).set(1)
                        span.set_attribute("status", "running")
                    else:
                        self.service_status[service] = 'stopped'
                        self.service_warning_issues.append(f"Service {service} is not running")
                        self.metric_service_up.labels(service=service).set(0)
                        self.metric_service_check_failures_total.labels(service=service).inc()
                        self.logger.warning("service_not_running", extra={"service": service})
                        span.set_attribute("status", "stopped")
                except Exception as e:
                    self.service_status[service] = 'unknown'
                    self.service_warning_issues.append(f"Could not check service {service}: {e}")
                    self.metric_service_up.labels(service=service).set(0)
                    self.metric_service_check_failures_total.labels(service=service).inc()
                    self.error_count += 1
                    self.metric_errors_total.labels(error_type="service_check").inc()
                    self.logger.exception("service_check_failed", extra={
                        "service": service,
                        "error": str(e)
                    })
                    self.error_handler.handle_error(e, f"Service check: {service}")
                    span.set_attribute("status", "unknown")
                    span.record_exception(e)

    @traced('system_health_monitor', 'check_network_connectivity')
    def check_network_connectivity(self):
        """Check network connectivity and endpoints"""
        # Check localhost (ROS master)
        self._check_tcp_port("127.0.0.1", 11311, "ROS master (localhost:11311)")

        # Check PX4
        self._check_tcp_port("px4_sitl", 14540, "PX4 UDP port (14540)")

        # Check MAVROS
        self._check_tcp_port("px4_sitl", 14550, "MAVROS UDP port (14550)")

    def _check_tcp_port(self, host, port, label):
        """Attempt a TCP connection and always close the socket afterwards.

        The original code called socket.create_connection() and discarded
        the returned socket object without closing it, leaking a file
        descriptor on every successful check.
        """
        with span_context(self.tracer, f"check_tcp_port.{label}", {
            "host": host,
            "port": port,
            "label": label
        }) as span:
            try:
                with socket.create_connection((host, port), timeout=2):
                    pass
                span.set_attribute("reachable", True)
            except OSError as e:
                self.warning_issues.append(f"Cannot connect to {label}")
                self.metric_network_check_failures_total.labels(endpoint=label).inc()
                self.logger.warning("network_check_failed", extra={
                    "endpoint": label,
                    "host": host,
                    "port": port,
                    "error": str(e)
                })
                span.set_attribute("reachable", False)
                span.record_exception(e)

    def get_cpu_temperature(self):
        """Get CPU temperature if available"""
        try:
            # Try to get temperature from psutil
            if hasattr(psutil, 'sensors_temperatures'):
                temps = psutil.sensors_temperatures()
                for name, entries in temps.items():
                    for entry in entries:
                        if 'core' in entry.label.lower() or name.lower() in ['cpu', 'thermal']:
                            return entry.current
        except Exception as e:
            self.logger.debug("cpu_temperature_read_failed", extra={
                "method": "psutil",
                "error": str(e)
            })

        # Try alternative method
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read()) / 1000.0
                return temp
        except Exception as e:
            self.logger.debug("cpu_temperature_read_failed", extra={
                "method": "sysfs",
                "error": str(e)
            })

        return None

    @traced('system_health_monitor', 'collect_performance_metrics')
    def collect_performance_metrics(self):
        """Collect additional performance metrics"""
        self.performance_metrics = {
            'cpu_load': list(self.cpu_info.get('load_average', [0.0, 0.0, 0.0])),
            'disk_io': self.process_info.get('io_counters', {}),
            'network_io': self.network_info,
            'process_count': self.process_info.get('running_processes', 0),
            'thread_count': self.process_info.get('thread_count', 0),
            'memory_usage_ratio': self.memory_info.get('used', 0) / self.memory_info.get('total', 1) if self.memory_info.get('total', 0) > 0 else 0.0
        }

    def background_monitor(self, event):
        """Background monitoring tasks"""
        with span_context(self.tracer, "background_monitor") as span:
            self.logger.debug("background_monitor_started")

            # Collect performance metrics
            self.collect_performance_metrics()

            # Publish performance metrics
            if self.performance_metrics:
                perf_msg = String()
                perf_msg.data = json.dumps(self.performance_metrics)
                self.performance_pub.publish(perf_msg)

            # Check for critical issues
            if self.critical_issues:
                # Publish critical alert
                alert_msg = String()
                alert_msg.data = json.dumps({
                    'type': 'critical',
                    'timestamp': time.time(),
                    'issues': self.critical_issues
                })
                self.alert_pub.publish(alert_msg)
                self.logger.warning("critical_alert_published", extra={
                    "issues": self.critical_issues
                })

            # Check for warning issues
            if self.warning_issues:
                # Publish warning alert
                alert_msg = String()
                alert_msg.data = json.dumps({
                    'type': 'warning',
                    'timestamp': time.time(),
                    'issues': self.warning_issues
                })
                self.alert_pub.publish(alert_msg)
                self.logger.debug("warning_alert_published", extra={
                    "issues": self.warning_issues
                })

            span.set_attribute("critical_issue_count", len(self.critical_issues))
            span.set_attribute("warning_issue_count", len(self.warning_issues))

            self.logger.debug("background_monitor_completed")

    @traced_ros_callback('system_health_monitor', 'check_all_callback')
    def check_all_callback(self, msg):
        """Handle comprehensive health check request"""
        self.logger.info("on_demand_health_check_requested")

        # Trigger immediate health check
        self.collect_system_metrics()
        self.check_for_issues()

        # Publish comprehensive health status
        self.publish_status(None)

    @traced('system_health_monitor', 'publish_status')
    def publish_status(self, event):
        """Publish current system health status"""
        # Determine overall system health
        if len(self.critical_issues) > 0:
            self.system_health = SystemHealthStatus.CRITICAL
        elif len(self.warning_issues) > 0:
            self.system_health = SystemHealthStatus.WARNING
        else:
            self.system_health = SystemHealthStatus.HEALTHY

        # Publish the structured NodeHealth message. The original code built
        # a std_msgs/String here and published it on self.health_pub, but
        # that publisher was declared with type NodeHealth -- publishing a
        # mismatched message type raises an error at runtime. Build the
        # correct message type instead, matching check_all_callback().
        health_msg = NodeHealth()
        health_msg.node_name = 'system_health_monitor'
        health_msg.status = self.system_health.value
        health_msg.timestamp = rospy.Time.now()
        health_msg.is_healthy = len(self.critical_issues) == 0
        self.health_pub.publish(health_msg)

        # Detailed status (issues, per-service, per-disk breakdown) doesn't
        # fit into NodeHealth's fields, so publish it separately as JSON.
        status_info = {
            'status': self.system_health.value,
            'cpu_usage': self.cpu_info.get('percent', 0.0),
            'memory_usage': self.memory_info.get('percent', 0.0),
            'critical_issues': self.critical_issues,
            'warning_issues': self.warning_issues,
            'service_status': self.service_status,
            'last_check_time': self.last_check_time,
            'disk_status': {mp: {'critical': info['percent'] >= 90,
                               'warning': info['percent'] >= 80,
                               'usage': info['percent']}
                           for mp, info in self.disk_info.items()}
        }
        detail_msg = String()
        detail_msg.data = json.dumps(status_info)
        self.performance_pub.publish(detail_msg)

        self.logger.info("status_published", extra={
            "status": self.system_health.value,
            "is_healthy": health_msg.is_healthy,
            "critical_issue_count": len(self.critical_issues),
            "warning_issue_count": len(self.warning_issues)
        })

    def get_health_summary(self):
        """Get current health summary"""
        return {
            'status': self.system_health.value,
            'cpu_usage': self.cpu_info.get('percent', 0.0),
            'memory_usage': self.memory_info.get('percent', 0.0),
            'critical_issues_count': len(self.critical_issues),
            'warning_issues_count': len(self.warning_issues),
            'service_count': len(self.service_status),
            'last_check_time': self.last_check_time,
            'is_healthy': len(self.critical_issues) == 0
        }


if __name__ == '__main__':
    try:
        monitor = SystemHealthMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass