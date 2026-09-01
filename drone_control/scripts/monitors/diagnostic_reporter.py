#!/usr/bin/env python3
"""
drone_control/scripts/monitors/diagnostic_reporter.py
System-wide diagnostic reporting
"""

import json
import os
import time

import psutil
import rospy
from std_msgs.msg import String

from drone_control.utils.ros_tracing import create_traced_publisher, traced_ros_callback
from drone_control.utils.tracing import init_tracing, get_component_tracer, span_context
from drone_control.msg import DiagnosticStatus, MissionStatus, NodeHealth, SafetyStatus
from drone_control.utils import ErrorHandler
from drone_control.utils import ErrorHandler, MessageValidator
from drone_control.utils.correlation import get_or_create_mission_id
from drone_control.utils.logging_framework import get_logger_with_ros_level
from drone_control.utils.metrics_collector import MetricsCollector, MetricTimer

class DiagnosticReporter:
    """System-wide diagnostic status reporting"""

    def __init__(self):
        rospy.init_node('diagnostic_reporter', anonymous=False)

        # Structured logger
        self.logger = get_logger_with_ros_level("diagnostic_reporter")
        self.logger.info("node_initializing", extra={"version": "1.0.0"})

        self.error_handler = ErrorHandler(node_name='diagnostic_reporter')

        # Distributed tracing, set up before any publishers/subscribers/
        # timers are registered.
        init_tracing(component='diagnostic_reporter')
        self.tracer = get_component_tracer('diagnostic_reporter')

        self.report_rate = rospy.get_param('~report_rate', 5.0)
        self.output_file = rospy.get_param('~output_file', 'diagnostic.log')

        # Metrics
        self.metrics = MetricsCollector("diagnostic_reporter", port=8003)
        self._init_metrics()

        # State
        self.node_health = {}
        self.safety_status = None
        self.mission_status = None
        self.start_time = time.time()
        self.report_count = 0
        self.error_count = 0

        # Subscribers
        rospy.Subscriber('/node_health', NodeHealth, self._node_health_callback)
        rospy.Subscriber('/safety_status', SafetyStatus, self._safety_callback)
        rospy.Subscriber('/mission_status', MissionStatus, self._mission_callback)

        # Publishers (traced so each publish shows up as a span)
        self.diagnostic_pub = create_traced_publisher(
            '/diagnostic_status', DiagnosticStatus, queue_size=10, tracer_name='diagnostic_reporter'
        )
        self.status_pub = create_traced_publisher(
            '/system_status', String, queue_size=10, tracer_name='diagnostic_reporter'
        )

        # Timer
        self.report_timer = rospy.Timer(rospy.Duration(self.report_rate), self._report_diagnostics)

        self.logger.info("node_initialized", extra={
            "report_rate": self.report_rate,
            "output_file": self.output_file
        })
        rospy.loginfo("Diagnostic Reporter initialized")

    def _init_metrics(self):
        """Initialize Prometheus metrics for this node"""
        self.logger.debug("initializing_metrics")

        self.metric_reports_total = self.metrics.counter(
            "diagnostic_reports_total",
            "Total number of diagnostic reports generated"
        )
        self.metric_errors_total = self.metrics.counter(
            "errors_total",
            "Total number of errors encountered",
            labels=["error_type"]
        )
        self.metric_cpu_usage = self.metrics.gauge(
            "cpu_usage_percent",
            "Current CPU usage percentage"
        )
        self.metric_memory_usage = self.metrics.gauge(
            "memory_usage_percent",
            "Current memory usage percentage"
        )
        self.metric_disk_usage = self.metrics.gauge(
            "disk_usage_percent",
            "Current disk usage percentage for /"
        )
        self.metric_node_count = self.metrics.gauge(
            "known_node_count",
            "Number of nodes currently reporting health"
        )
        self.metric_uptime = self.metrics.gauge(
            "uptime_seconds",
            "Time since diagnostic reporter started"
        )
        self.metric_report_duration = self.metrics.histogram(
            "report_duration_seconds",
            "Time taken to build and publish a diagnostic report",
            buckets=[0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0]
        )

        self.logger.info("metrics_initialized")

    @traced_ros_callback('diagnostic_reporter', 'node_health_callback')
    def _node_health_callback(self, msg):
        """Handle node health messages"""
        self.node_health[msg.node_name] = msg

    @traced_ros_callback('diagnostic_reporter', 'safety_callback')
    def _safety_callback(self, msg):
        """Handle safety status messages"""
        self.safety_status = msg

    @traced_ros_callback('diagnostic_reporter', 'mission_callback')
    def _mission_callback(self, msg):
        """Handle mission status messages"""
        self.mission_status = msg

    def _report_diagnostics(self, event):
        """Generate and publish diagnostic report"""
        self.report_count += 1

        with span_context(self.tracer, "report_diagnostics", {
            "report_id": self.report_count
        }) as span:
            with MetricTimer(self.metric_report_duration):
                # System metrics
                cpu_percent = psutil.cpu_percent()
                # `.percent` is a property on the named-tuple returned by
                # virtual_memory(), not a method -- calling it as
                # `.percent()` raised TypeError on every report cycle.
                memory_percent = psutil.virtual_memory().percent
                disk_percent = psutil.disk_usage('/').percent

                self.metric_cpu_usage.set(cpu_percent)
                self.metric_memory_usage.set(memory_percent)
                self.metric_disk_usage.set(disk_percent)
                self.metric_node_count.set(len(self.node_health))
                uptime = time.time() - self.start_time
                self.metric_uptime.set(uptime)

                # Build diagnostic message
                diag_msg = DiagnosticStatus()
                diag_msg.timestamp = rospy.Time.now()

                # System health
                diag_msg.system_health = json.dumps({
                    'cpu_usage': cpu_percent,
                    'memory_usage': memory_percent,
                    'disk_usage': disk_percent,
                    'uptime': uptime,
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

                # Log to file if specified. Only create parent directories
                # when output_file actually names one -- os.path.dirname()
                # returns '' for a bare filename like 'diagnostic.log', and
                # os.makedirs('', exist_ok=True) raises FileNotFoundError,
                # which the previous bare `except: pass` silently swallowed
                # on every single report cycle.
                if self.output_file:
                    try:
                        out_dir = os.path.dirname(self.output_file)
                        if out_dir:
                            os.makedirs(out_dir, exist_ok=True)
                        with open(self.output_file, 'a') as f:
                            f.write(f"{time.time()}: {diag_msg.system_health}\n")
                    except Exception as e:
                        self.error_count += 1
                        self.metric_errors_total.labels(error_type="file_write").inc()
                        self.logger.warning("diagnostic_file_write_failed", extra={
                            "output_file": self.output_file,
                            "error": str(e)
                        })
                        self.error_handler.handle_error(e, f"Writing diagnostic log: {self.output_file}")

            span.set_attribute("cpu_usage", cpu_percent)
            span.set_attribute("memory_usage", memory_percent)
            span.set_attribute("disk_usage", disk_percent)
            span.set_attribute("node_count", len(self.node_health))

        self.metric_reports_total.inc()

        self.logger.debug("report_published", extra={
            "report_id": self.report_count,
            "cpu_usage": cpu_percent,
            "memory_usage": memory_percent,
            "node_count": len(self.node_health)
        })

if __name__ == '__main__':
    try:
        reporter = DiagnosticReporter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass