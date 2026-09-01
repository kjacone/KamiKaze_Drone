#!/usr/bin/env python3
"""
drone_control/scripts/monitors/health_checker.py
Node health monitoring and auto-restart
"""

import subprocess
import time

import rospy
from std_srvs.srv import Trigger, TriggerResponse

from drone_control.utils.ros_tracing import create_traced_publisher, TracedService, traced_ros_callback
from drone_control.utils.tracing import init_tracing, get_component_tracer, span_context, traced
from drone_control.msg import NodeHealth
from drone_control.utils import ErrorHandler, MessageValidator
from drone_control.utils.correlation import get_or_create_mission_id
from drone_control.utils.logging_framework import get_logger_with_ros_level
from drone_control.utils.metrics_collector import MetricsCollector, MetricTimer


class HealthChecker:
    """Comprehensive node health monitoring"""

    def __init__(self):
        rospy.init_node('health_checker', anonymous=False)

        # Initialize structured logger
        self.logger = get_logger_with_ros_level("health_checker")
        self.logger.info("node_initializing", extra={"version": "1.0.0"})

        # Tracing setup. Do this before anything else registers
        # publishers/services/timers so every downstream call has a
        # tracer available.
        init_tracing(component='health_checker')
        self.tracer = get_component_tracer('health_checker')

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
            'safety_monitor',
            'command_interpreter'
        ])

        # Initialize metrics collector (parameters above must be set first,
        # since _init_metrics() reads self.monitored_nodes)
        self.metrics = MetricsCollector("health_checker", port=8003)
        self._init_metrics()

        # State tracking
        self.node_health = {}
        self.node_restart_counts = {}
        self.last_health_time = {}

        # Subscribers
        for node in self.monitored_nodes:
            rospy.Subscriber(f'/{node}/node_health', NodeHealth, self._health_callback, callback_args=node)

        # Publishers (traced so system-health fan-out shows up in the
        # same trace as the health check that produced it)
        self.health_pub = create_traced_publisher(
            '/system_health', NodeHealth, queue_size=10, tracer_name='health_checker'
        )

        # Services (TracedService wraps rospy.Service registration itself,
        # so each call gets its own span with duration/success attributes)
        TracedService('/health/check_all', Trigger, self._check_all, tracer_name='health_checker')
        TracedService('/health/restart_node', Trigger, self._restart_node, tracer_name='health_checker')

        # Timer
        self.check_timer = rospy.Timer(rospy.Duration(self.check_interval), self._check_health)

        self.logger.info("node_initialized", extra={
            "monitored_nodes": self.monitored_nodes,
            "check_interval": self.check_interval,
            "timeout_threshold": self.timeout_threshold,
            "max_restarts": self.max_restarts
        })
        rospy.loginfo("Health Checker initialized")

    def _init_metrics(self):
        """Initialize Prometheus metrics for this node"""
        self.logger.debug("initializing_metrics")

        # Counters
        self.metric_health_checks_total = self.metrics.counter(
            "health_checks_total",
            "Total number of health check cycles run"
        )
        self.metric_unhealthy_nodes_total = self.metrics.counter(
            "unhealthy_nodes_total",
            "Total times a monitored node was found unhealthy",
            labels=["node", "reason_type"]
        )
        self.metric_restart_attempts_total = self.metrics.counter(
            "restart_attempts_total",
            "Total restart attempts issued",
            labels=["node"]
        )
        self.metric_restart_failures_total = self.metrics.counter(
            "restart_failures_total",
            "Total restart attempts that raised an exception",
            labels=["node"]
        )
        self.metric_restarts_exceeded_total = self.metrics.counter(
            "restarts_exceeded_total",
            "Total times a node exceeded max_restarts and was given up on",
            labels=["node"]
        )
        self.metric_node_recovered_total = self.metrics.counter(
            "node_recovered_total",
            "Total times a node recovered (reported healthy) after previously needing a restart",
            labels=["node"]
        )

        # Gauges
        self.metric_monitored_node_count = self.metrics.gauge(
            "monitored_node_count",
            "Number of nodes currently being monitored"
        )
        self.metric_healthy_node_count = self.metrics.gauge(
            "healthy_node_count",
            "Number of currently healthy monitored nodes"
        )
        self.metric_node_health_status = self.metrics.gauge(
            "node_health_status",
            "Per-node health (1=healthy, 0=unhealthy)",
            labels=["node"]
        )
        self.metric_node_restart_count = self.metrics.gauge(
            "node_restart_count",
            "Current restart counter for a node",
            labels=["node"]
        )

        # Histograms
        self.metric_health_check_duration = self.metrics.histogram(
            "health_check_duration_seconds",
            "Time taken to complete a periodic health check cycle",
            buckets=[0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0]
        )

        self.metric_monitored_node_count.set(len(self.monitored_nodes))
        self.logger.info("metrics_initialized")

    @traced_ros_callback('health_checker', 'health_callback')
    def _health_callback(self, msg, node_name):
        """Handle health messages from nodes"""
        self.logger.debug("health_report_received", extra={
            "node": node_name,
            "is_healthy": msg.is_healthy,
            "status": msg.status
        })
        self.node_health[node_name] = msg
        self.last_health_time[node_name] = time.time()
        self.metric_node_health_status.labels(node=node_name).set(1 if msg.is_healthy else 0)

        # A node that's reporting healthy again has recovered. Clear its
        # restart counter so a rare failure long in the past doesn't
        # combine with a later, unrelated failure to push it over
        # max_restarts and permanently give up on a node that's actually
        # fine most of the time.
        if msg.is_healthy and self.node_restart_counts.get(node_name, 0) > 0:
            self.logger.info("node_recovered", extra={
                "node": node_name,
                "previous_restart_count": self.node_restart_counts[node_name]
            })
            self.metric_node_recovered_total.labels(node=node_name).inc()
            self.node_restart_counts[node_name] = 0
            self.metric_node_restart_count.labels(node=node_name).set(0)

    def _check_health(self, event):
        """Periodic health check"""
        with span_context(self.tracer, "check_health", {
            "monitored_node_count": len(self.monitored_nodes)
        }) as span:
            with MetricTimer(self.metric_health_check_duration):
                current_time = time.time()
                unhealthy_nodes = []

                for node in self.monitored_nodes:
                    # Check if node reported health
                    if node not in self.node_health:
                        unhealthy_nodes.append((node, "No health report"))
                        self.metric_unhealthy_nodes_total.labels(node=node, reason_type="no_report").inc()
                        self.logger.warning("node_no_health_report", extra={"node": node})
                        continue

                    # Check health status
                    if not self.node_health[node].is_healthy:
                        unhealthy_nodes.append((node, "Reported unhealthy"))
                        self.metric_unhealthy_nodes_total.labels(node=node, reason_type="reported_unhealthy").inc()
                        self.logger.warning("node_reported_unhealthy", extra={
                            "node": node,
                            "status": self.node_health[node].status
                        })
                        continue

                    # Check timeout
                    if node in self.last_health_time:
                        elapsed = current_time - self.last_health_time[node]
                        if elapsed > self.timeout_threshold:
                            unhealthy_nodes.append((node, f"Timeout ({elapsed:.1f}s)"))
                            self.metric_unhealthy_nodes_total.labels(node=node, reason_type="timeout").inc()
                            self.logger.warning("node_health_timeout", extra={
                                "node": node,
                                "elapsed_seconds": elapsed,
                                "timeout_threshold": self.timeout_threshold
                            })

                # Handle unhealthy nodes
                if unhealthy_nodes:
                    rospy.logwarn(f"Unhealthy nodes: {len(unhealthy_nodes)}")
                    for node, reason in unhealthy_nodes:
                        rospy.logwarn(f"  {node}: {reason}")

                        if self.restart_on_failure:
                            self._restart_node_attempt(node, reason)
                else:
                    rospy.logdebug("All nodes healthy")
                    self.logger.debug("all_nodes_healthy")

                self.metric_health_checks_total.inc()
                self.metric_healthy_node_count.set(len(self.monitored_nodes) - len(unhealthy_nodes))

                span.set_attribute("unhealthy_node_count", len(unhealthy_nodes))
                span.set_attribute("unhealthy_nodes", ",".join(n for n, _ in unhealthy_nodes))

                # Publish system health
                health_msg = NodeHealth()
                health_msg.node_name = 'system'
                health_msg.status = 'healthy' if not unhealthy_nodes else 'degraded'
                health_msg.timestamp = rospy.Time.now()
                health_msg.is_healthy = len(unhealthy_nodes) == 0
                self.health_pub.publish(health_msg)

                self.logger.info("health_check_completed", extra={
                    "unhealthy_count": len(unhealthy_nodes),
                    "system_status": health_msg.status
                })

    @traced('health_checker', 'restart_node_attempt')
    def _restart_node_attempt(self, node_name, reason=None):
        """Attempt to restart a node"""
        with span_context(self.tracer, f"restart.{node_name}", {
            "node_name": node_name,
            "reason": reason or "unknown"
        }) as span:
            # Track restart attempts
            if node_name not in self.node_restart_counts:
                self.node_restart_counts[node_name] = 0

            self.node_restart_counts[node_name] += 1
            self.metric_restart_attempts_total.labels(node=node_name).inc()
            self.metric_node_restart_count.labels(node=node_name).set(self.node_restart_counts[node_name])
            span.set_attribute("attempt_number", self.node_restart_counts[node_name])

            if self.node_restart_counts[node_name] > self.max_restarts:
                rospy.logerr(f"Node {node_name} exceeded max restarts ({self.max_restarts})")
                self.logger.error("node_exceeded_max_restarts", extra={
                    "node": node_name,
                    "max_restarts": self.max_restarts
                })
                self.metric_restarts_exceeded_total.labels(node=node_name).inc()
                span.set_attribute("exceeded_max_restarts", True)
                return

            rospy.loginfo(f"Attempting to restart {node_name} (attempt {self.node_restart_counts[node_name]})")
            self.logger.info("restart_attempt", extra={
                "node": node_name,
                "attempt": self.node_restart_counts[node_name],
                "reason": reason
            })

            try:
                # Kill existing node. This can legitimately fail/return
                # nonzero if the node already died on its own (the common
                # case that got us here) -- that's expected, not an error, so
                # we don't check the return code, just swallow stdout/stderr.
                subprocess.run(['rosnode', 'kill', node_name], timeout=5, capture_output=True)
                rospy.sleep(1)

                # Relaunch the node. Previously this function only killed the
                # node and logged "restart initiated" -- nothing ever actually
                # started it again, so a crashed node stayed dead until a
                # human intervened, and this health checker would just keep
                # burning restart attempts against a node that could never
                # come back. Nodes in this workspace are catkin-installed
                # executables (e.g. /root/catkin_ws/devel/lib/drone_control/
                # yolo_detector.py), so `rosrun drone_control <node>.py` finds
                # them regardless of which scripts/ subfolder they live in.
                subprocess.Popen(
                    ['rosrun', 'drone_control', f'{node_name}.py'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                rospy.loginfo(f"Node {node_name} restart initiated")
                self.logger.info("restart_initiated", extra={"node": node_name})
                span.set_attribute("restart_initiated", True)

            except Exception as e:
                rospy.logerr(f"Failed to restart {node_name}: {e}")
                self.logger.exception("restart_failed", extra={
                    "node": node_name,
                    "error": str(e)
                })
                self.metric_restart_failures_total.labels(node=node_name).inc()
                span.set_attribute("restart_initiated", False)
                span.record_exception(e)

    def _check_all(self, req):
        """Check all nodes health service"""
        self.logger.debug("check_all_requested")
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
        """Restart a specific node service.

        NOTE: std_srvs/Trigger carries no request fields, so there is no
        way for a caller to say *which* node to restart -- this service
        can never be more than a stub until it's redefined with a custom
        service message that includes a node_name field.
        """
        self.logger.warning("restart_node_service_stub_called")
        return TriggerResponse(
            success=False,
            message="Restart functionality not fully implemented (Trigger has no node_name field)"
        )

if __name__ == '__main__':
    try:
        checker = HealthChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass