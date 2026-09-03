#!/usr/bin/env python3
"""
drone_control/script/controllers/performance_monitor.py
Comprehensive performance monitoring and benchmarking
"""

import json
import os
import sys
import threading
import time
from datetime import datetime, timedelta
from enum import Enum

import numpy as np
import psutil
import rospy
from std_msgs.msg import String

from drone_control.msg import DiagnosticStatus

from drone_control.utils.ros_tracing import create_traced_publisher, TracedService, traced_ros_callback
from drone_control.utils.tracing import init_tracing, get_component_tracer, span_context, traced
from drone_control.utils.logging_framework import get_logger_with_ros_level
from drone_control.utils.metrics_collector import MetricsCollector, MetricTimer

sys.path.append(os.path.join(os.path.dirname(__file__), '../../src'))
try:
    from error_handler import ErrorHandler
except ImportError:
    class ErrorHandler:
        def __init__(self, node_name):
            self.node_name = node_name
        def handle_error(self, error, context=None):
            rospy.logerr(f"[{self.node_name}] {error}: {context}")


class PerformanceMetricType(Enum):
    FPS = "fps"
    LATENCY = "latency"
    CPU_USAGE = "cpu_usage"
    MEMORY_USAGE = "memory_usage"
    BANDWIDTH = "bandwidth"
    PACKET_LOSS = "packet_loss"
    QUEUE_LENGTH = "queue_length"
    PROCESSING_TIME = "processing_time"


class PerformanceMonitorStatus(Enum):
    IDLE = "idle"
    MONITORING = "monitoring"
    COLLECTING = "collecting"
    ANALYZING = "analyzing"
    ALERTING = "alerting"


class PerformanceMonitor:
    """Comprehensive performance monitoring and benchmarking system"""

    def __init__(self):
        rospy.init_node('performance_monitor', anonymous=False)

        # Initialize structured logger
        self.logger = get_logger_with_ros_level("performance_monitor")
        self.logger.info("node_initializing", extra={"version": "1.0.0"})

        # Tracing setup. Do this before anything else registers
        # publishers/services/timers so every downstream call has a
        # tracer available.
        init_tracing(component='performance_monitor')
        self.tracer = get_component_tracer('performance_monitor')

        self.error_handler = ErrorHandler(node_name='performance_monitor')

        # Configuration parameters
        self.monitoring_duration = rospy.get_param('~monitoring_duration', 60.0)
        self.collection_interval = rospy.get_param('~collection_interval', 0.1)
        self.benchmark_interval = rospy.get_param('~benchmark_interval', 5.0)
        self.performance_threshold = rospy.get_param('~performance_threshold', 0.9)
        self.alert_threshold = rospy.get_param('~alert_threshold', 0.95)
        self.enable_benchmarking = rospy.get_param('~enable_benchmarking', True)
        self.simulation_mode = rospy.get_param('/use_simulation', True)

        # Initialize metrics collector
        self.metrics = MetricsCollector("performance_monitor", port=8004)
        self._init_metrics()

        # State variables
        self.status = PerformanceMonitorStatus.IDLE
        self.is_monitoring = False
        self.metrics_data = {}
        self.benchmark_results = {}
        self.alerts = []
        self.performance_history = []
        self.target_metrics = {}
        self.collection_threads = []
        self.benchmark_timer = None
        self.collection_timer = None
        self.alert_timer = None

        # Metric collections
        self.fps_data = []
        self.latency_data = []
        self.cpu_data = []
        self.memory_data = []
        self.network_data = []
        self.queue_data = []
        self.processing_time_data = []

        # Subscribers (traced so command handling appears in traces)
        self.performance_command_sub = rospy.Subscriber(
            '/performance/command', String, self.performance_command_callback
        )
        self.benchmark_request_sub = rospy.Subscriber(
            '/performance/benchmark', String, self.benchmark_request_callback
        )

        # Publishers (traced so metric/alert fan-out shows up in the
        # same trace as the collection/benchmark that produced it)
        self.metrics_pub = create_traced_publisher(
            '/performance/metrics', String, queue_size=10, tracer_name='performance_monitor'
        )
        self.benchmark_pub = create_traced_publisher(
            '/performance/benchmark_results', String, queue_size=10, tracer_name='performance_monitor'
        )
        self.alert_pub = create_traced_publisher(
            '/performance/alert', String, queue_size=10, tracer_name='performance_monitor'
        )
        self.diagnostic_pub = create_traced_publisher(
            '/diagnostic_status', DiagnosticStatus, queue_size=10, tracer_name='performance_monitor'
        )

        # Timers
        # NOTE: oneshot=True — without this, rospy.Timer fires every 2s forever,
        # and start_monitoring() would keep creating new benchmark/collection
        # timers without ever canceling the old ones (a resource/thread leak).
        self.startup_monitor = rospy.Timer(rospy.Duration(2.0), self.start_monitoring, oneshot=True)

        self.logger.info("node_initialized", extra={
            "monitoring_duration": self.monitoring_duration,
            "collection_interval": self.collection_interval,
            "benchmark_interval": self.benchmark_interval,
            "performance_threshold": self.performance_threshold,
            "alert_threshold": self.alert_threshold,
            "enable_benchmarking": self.enable_benchmarking,
            "simulation_mode": self.simulation_mode
        })
        rospy.loginfo("Performance Monitor initialized")

    def _init_metrics(self):
        """Initialize Prometheus metrics for this node"""
        self.logger.debug("initializing_metrics")

        # Counters
        self.metric_collection_cycles_total = self.metrics.counter(
            "collection_cycles_total",
            "Total number of metric collection cycles run"
        )
        self.metric_benchmark_runs_total = self.metrics.counter(
            "benchmark_runs_total",
            "Total number of benchmark runs completed",
            labels=["result"]
        )
        self.metric_alerts_total = self.metrics.counter(
            "alerts_total",
            "Total performance alerts published",
            labels=["type", "severity"]
        )
        self.metric_command_total = self.metrics.counter(
            "commands_total",
            "Total performance commands received",
            labels=["command_type"]
        )
        self.metric_benchmark_request_total = self.metrics.counter(
            "benchmark_requests_total",
            "Total benchmark requests received",
            labels=["benchmark_type"]
        )
        self.metric_ros_metric_errors_total = self.metrics.counter(
            "ros_metric_errors_total",
            "Total errors while collecting ROS graph metrics",
            labels=["source"]
        )

        # Gauges
        self.metric_is_monitoring = self.metrics.gauge(
            "is_monitoring",
            "Whether performance monitoring is currently active (1=yes, 0=no)"
        )
        self.metric_status = self.metrics.gauge(
            "status_code",
            "Current monitor status as numeric code (0=idle, 1=monitoring, 2=collecting, 3=analyzing, 4=alerting)"
        )
        self.metric_fps = self.metrics.gauge(
            "fps",
            "Most recent FPS sample"
        )
        self.metric_avg_fps = self.metrics.gauge(
            "avg_fps",
            "Average FPS over recent samples"
        )
        self.metric_cpu_percent = self.metrics.gauge(
            "cpu_percent",
            "Most recent CPU usage percent"
        )
        self.metric_avg_cpu_percent = self.metrics.gauge(
            "avg_cpu_percent",
            "Average CPU usage percent over recent samples"
        )
        self.metric_memory_percent = self.metrics.gauge(
            "memory_percent",
            "Most recent memory usage percent"
        )
        self.metric_avg_memory_percent = self.metrics.gauge(
            "avg_memory_percent",
            "Average memory usage percent over recent samples"
        )
        self.metric_alert_count = self.metrics.gauge(
            "alert_count",
            "Current number of active/tracked alerts"
        )
        self.metric_benchmark_score = self.metrics.gauge(
            "benchmark_overall_score",
            "Most recent overall benchmark performance score"
        )
        self.metric_topic_count = self.metrics.gauge(
            "ros_topic_count",
            "Number of ROS topics observed during last collection"
        )
        self.metric_service_count = self.metrics.gauge(
            "ros_service_count",
            "Number of ROS services observed during last collection"
        )
        self.metric_node_count = self.metrics.gauge(
            "ros_node_count",
            "Number of ROS nodes observed during last collection"
        )
        self.metric_fps_samples = self.metrics.gauge(
            "fps_samples",
            "Number of FPS samples currently retained"
        )
        self.metric_cpu_samples = self.metrics.gauge(
            "cpu_samples",
            "Number of CPU samples currently retained"
        )
        self.metric_memory_samples = self.metrics.gauge(
            "memory_samples",
            "Number of memory samples currently retained"
        )

        # Histograms
        self.metric_collection_duration = self.metrics.histogram(
            "collection_duration_seconds",
            "Time taken to complete a metric collection cycle",
            buckets=[0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0]
        )
        self.metric_benchmark_duration = self.metrics.histogram(
            "benchmark_duration_seconds",
            "Time taken to complete a full benchmark run",
            buckets=[0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 30.0, 60.0]
        )
        self.metric_benchmark_component_duration = self.metrics.histogram(
            "benchmark_component_duration_seconds",
            "Time taken by individual benchmark components",
            labels=["component"],
            buckets=[0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0]
        )
        self.metric_fps_histogram = self.metrics.histogram(
            "fps_distribution",
            "Distribution of observed FPS values",
            buckets=[1, 5, 10, 15, 20, 25, 30, 45, 60, 90, 120]
        )
        self.metric_cpu_histogram = self.metrics.histogram(
            "cpu_percent_distribution",
            "Distribution of observed CPU usage percent",
            buckets=[10, 20, 30, 40, 50, 60, 70, 80, 90, 95, 100]
        )
        self.metric_memory_histogram = self.metrics.histogram(
            "memory_percent_distribution",
            "Distribution of observed memory usage percent",
            buckets=[10, 20, 30, 40, 50, 60, 70, 80, 90, 95, 100]
        )

        self.metric_is_monitoring.set(0)
        self.metric_status.set(0)  # IDLE
        self.logger.info("metrics_initialized")

    def _status_code(self):
        """Map status enum to a stable numeric code for the gauge."""
        mapping = {
            PerformanceMonitorStatus.IDLE: 0,
            PerformanceMonitorStatus.MONITORING: 1,
            PerformanceMonitorStatus.COLLECTING: 2,
            PerformanceMonitorStatus.ANALYZING: 3,
            PerformanceMonitorStatus.ALERTING: 4,
        }
        return mapping.get(self.status, -1)

    def start_monitoring(self, event):
        """Start performance monitoring"""
        with span_context(self.tracer, "start_monitoring") as span:
            if self.is_monitoring:
                # Already running — don't stack duplicate timers/threads.
                self.logger.debug("start_monitoring_ignored_already_running")
                rospy.logdebug("start_monitoring called while already monitoring; ignoring")
                span.set_attribute("already_running", True)
                return

            if self.benchmark_interval > 0:
                self.benchmark_timer = rospy.Timer(
                    rospy.Duration(self.benchmark_interval), self.run_benchmark
                )

            if self.collection_interval > 0:
                self.collection_timer = rospy.Timer(
                    rospy.Duration(self.collection_interval), self.collect_metrics
                )

            self.status = PerformanceMonitorStatus.MONITORING
            self.is_monitoring = True
            self.metric_is_monitoring.set(1)
            self.metric_status.set(self._status_code())

            span.set_attribute("benchmark_interval", self.benchmark_interval)
            span.set_attribute("collection_interval", self.collection_interval)
            self.logger.info("monitoring_started", extra={
                "benchmark_interval": self.benchmark_interval,
                "collection_interval": self.collection_interval
            })
            rospy.loginfo("Performance monitoring started")

    def collect_metrics(self, event):
        """Collect performance metrics"""
        if not self.is_monitoring:
            return

        with span_context(self.tracer, "collect_metrics") as span:
            with MetricTimer(self.metric_collection_duration):
                self.status = PerformanceMonitorStatus.COLLECTING
                self.metric_status.set(self._status_code())

                # Collect system metrics
                current_time = time.time()

                # FPS metric
                if 'last_frame_time' in self.metrics_data:
                    fps = 1.0 / (current_time - self.metrics_data['last_frame_time'])
                    self.fps_data.append({
                        'timestamp': current_time,
                        'fps': fps,
                        'drop_detected': fps < 10.0
                    })
                    self.metric_fps.set(fps)
                    self.metric_fps_histogram.observe(fps)
                    if self.fps_data:
                        avg_fps = float(np.mean([d['fps'] for d in self.fps_data[-10:]]))
                        self.metric_avg_fps.set(avg_fps)
                        span.set_attribute("avg_fps", avg_fps)

                self.metrics_data['last_frame_time'] = current_time

                # CPU usage
                cpu_percent = psutil.cpu_percent(interval=None)
                self.cpu_data.append({
                    'timestamp': current_time,
                    'cpu_percent': cpu_percent,
                    'cpu_cores': psutil.cpu_count(logical=False),
                    'cpu_logical_cores': psutil.cpu_count(logical=True)
                })
                self.metric_cpu_percent.set(cpu_percent)
                self.metric_cpu_histogram.observe(cpu_percent)
                if self.cpu_data:
                    avg_cpu = float(np.mean([d['cpu_percent'] for d in self.cpu_data[-10:]]))
                    self.metric_avg_cpu_percent.set(avg_cpu)
                    span.set_attribute("avg_cpu_percent", avg_cpu)

                # Memory usage
                memory = psutil.virtual_memory()
                self.memory_data.append({
                    'timestamp': current_time,
                    'memory_percent': memory.percent,
                    'memory_total': memory.total,
                    'memory_available': memory.available,
                    'memory_used': memory.used,
                    'swap_percent': psutil.swap_memory().percent
                })
                self.metric_memory_percent.set(memory.percent)
                self.metric_memory_histogram.observe(memory.percent)
                if self.memory_data:
                    avg_memory = float(np.mean([d['memory_percent'] for d in self.memory_data[-10:]]))
                    self.metric_avg_memory_percent.set(avg_memory)
                    span.set_attribute("avg_memory_percent", avg_memory)

                # Network metrics
                net_io = psutil.net_io_counters()
                self.network_data.append({
                    'timestamp': current_time,
                    'bytes_sent': net_io.bytes_sent,
                    'bytes_recv': net_io.bytes_recv,
                    'packets_sent': net_io.packets_sent,
                    'packets_recv': net_io.packets_recv,
                    # drop_in/drop_out are Linux-only fields on psutil's snetio tuple;
                    # not present on some platforms/psutil versions, so fall back to 0
                    # instead of letting the AttributeError kill this timer's thread.
                    'packets_dropped_in': getattr(net_io, 'drop_in', 0),
                    'packets_dropped_out': getattr(net_io, 'drop_out', 0)
                })

                # Collect application-specific metrics
                self.collect_application_metrics()

                # Keep only recent data
                self.clean_old_data()

                # Check for alerts
                self.check_for_alerts()

                self.metric_collection_cycles_total.inc()
                self.metric_fps_samples.set(len(self.fps_data))
                self.metric_cpu_samples.set(len(self.cpu_data))
                self.metric_memory_samples.set(len(self.memory_data))
                self.metric_alert_count.set(len(self.alerts))

                span.set_attribute("fps_samples", len(self.fps_data))
                span.set_attribute("cpu_samples", len(self.cpu_data))
                span.set_attribute("memory_samples", len(self.memory_data))
                span.set_attribute("alert_count", len(self.alerts))

                self.logger.debug("metrics_collected", extra={
                    "fps_samples": len(self.fps_data),
                    "cpu_samples": len(self.cpu_data),
                    "memory_samples": len(self.memory_data),
                    "alert_count": len(self.alerts)
                })

    def collect_application_metrics(self):
        """Collect application-specific performance metrics"""
        with span_context(self.tracer, "collect_application_metrics") as span:
            # Get ROS metrics if available
            try:
                import rosgraph
                master = rosgraph.Master('/rosgraph')
                master_uri = master.master_uri

                self.metrics_data['ros_master_uri'] = master_uri

                # rosgraph.Master has no getTopicList()/getServiceList() methods —
                # the real XML-RPC API exposes graph state via getSystemState(),
                # which returns (publishers, subscribers, services), each a list
                # of [name, [node_names]].
                publishers, subscribers, services = master.getSystemState()
                topic_names = {name for name, _ in publishers} | {name for name, _ in subscribers}

                self.metrics_data['topic_count'] = len(topic_names)
                self.metrics_data['service_count'] = len(services)
                self.metric_topic_count.set(len(topic_names))
                self.metric_service_count.set(len(services))
                span.set_attribute("topic_count", len(topic_names))
                span.set_attribute("service_count", len(services))

            except Exception as e:
                self.metric_ros_metric_errors_total.labels(source="rosgraph").inc()
                self.logger.warning("ros_metrics_collection_failed", extra={"error": str(e)})
                rospy.logwarn(f"Could not collect ROS metrics: {e}")

            # Collect node-specific metrics
            try:
                from rosnode import get_node_names
                node_count = len(get_node_names())
                self.metrics_data['ros_node_count'] = node_count
                self.metric_node_count.set(node_count)
                span.set_attribute("node_count", node_count)
            except Exception as e:
                self.metric_ros_metric_errors_total.labels(source="rosnode").inc()
                self.logger.warning("ros_node_metrics_collection_failed", extra={"error": str(e)})
                rospy.logwarn(f"Could not collect ROS node metrics: {e}")

    def clean_old_data(self):
        """Remove old metric data to keep memory usage manageable"""
        current_time = time.time()
        cutoff_time = current_time - 300.0  # 5 minutes ago

        before_fps = len(self.fps_data)
        before_cpu = len(self.cpu_data)
        before_memory = len(self.memory_data)
        before_network = len(self.network_data)

        # Clean FPS data
        self.fps_data = [d for d in self.fps_data if d['timestamp'] > cutoff_time]

        # Clean CPU data
        self.cpu_data = [d for d in self.cpu_data if d['timestamp'] > cutoff_time]

        # Clean memory data
        self.memory_data = [d for d in self.memory_data if d['timestamp'] > cutoff_time]

        # Clean network data
        self.network_data = [d for d in self.network_data if d['timestamp'] > cutoff_time]

        pruned = (
            (before_fps - len(self.fps_data)) +
            (before_cpu - len(self.cpu_data)) +
            (before_memory - len(self.memory_data)) +
            (before_network - len(self.network_data))
        )
        if pruned > 0:
            self.logger.debug("old_data_pruned", extra={
                "pruned_samples": pruned,
                "cutoff_age_seconds": 300.0
            })

    def check_for_alerts(self):
        """Check for performance alerts"""
        with span_context(self.tracer, "check_for_alerts") as span:
            current_time = time.time()
            alerts_triggered = []

            # Check FPS alerts
            if self.fps_data:
                recent_fps = self.fps_data[-10:] if len(self.fps_data) >= 10 else self.fps_data
                avg_fps = np.mean([d['fps'] for d in recent_fps])

                if avg_fps < 10.0:
                    alerts_triggered.append({
                        'type': 'low_fps',
                        'severity': 'warning',
                        'value': avg_fps,
                        'threshold': 10.0,
                        'message': f"Average FPS ({avg_fps:.1f}) below minimum threshold"
                    })
                elif avg_fps < 5.0:
                    alerts_triggered.append({
                        'type': 'critical_fps',
                        'severity': 'critical',
                        'value': avg_fps,
                        'threshold': 5.0,
                        'message': f"Average FPS ({avg_fps:.1f}) critically low"
                    })

            # Check CPU alerts
            if self.cpu_data:
                recent_cpu = self.cpu_data[-10:] if len(self.cpu_data) >= 10 else self.cpu_data
                avg_cpu = np.mean([d['cpu_percent'] for d in recent_cpu])

                if avg_cpu >= 90.0:
                    alerts_triggered.append({
                        'type': 'high_cpu',
                        'severity': 'warning',
                        'value': avg_cpu,
                        'threshold': 90.0,
                        'message': f"Average CPU usage ({avg_cpu:.1f}%) high"
                    })
                elif avg_cpu >= 95.0:
                    alerts_triggered.append({
                        'type': 'critical_cpu',
                        'severity': 'critical',
                        'value': avg_cpu,
                        'threshold': 95.0,
                        'message': f"Average CPU usage ({avg_cpu:.1f}%) critical"
                    })

            # Check memory alerts
            if self.memory_data:
                recent_memory = self.memory_data[-10:] if len(self.memory_data) >= 10 else self.memory_data
                avg_memory = np.mean([d['memory_percent'] for d in recent_memory])

                if avg_memory >= 90.0:
                    alerts_triggered.append({
                        'type': 'high_memory',
                        'severity': 'warning',
                        'value': avg_memory,
                        'threshold': 90.0,
                        'message': f"Average memory usage ({avg_memory:.1f}%) high"
                    })
                elif avg_memory >= 95.0:
                    alerts_triggered.append({
                        'type': 'critical_memory',
                        'severity': 'critical',
                        'value': avg_memory,
                        'threshold': 95.0,
                        'message': f"Average memory usage ({avg_memory:.1f}%) critical"
                    })

            # Add new alerts to alert list
            new_alert_count = 0
            for alert in alerts_triggered:
                if alert not in self.alerts:
                    self.alerts.append(alert)
                    new_alert_count += 1

                    self.metric_alerts_total.labels(
                        type=alert['type'], severity=alert['severity']
                    ).inc()

                    self.logger.warning("performance_alert", extra={
                        "alert_type": alert['type'],
                        "severity": alert['severity'],
                        "value": alert['value'],
                        "threshold": alert['threshold'],
                        "message": alert['message']
                    })

                    # Publish alert
                    alert_msg = String()
                    alert_msg.data = json.dumps(alert)
                    self.alert_pub.publish(alert_msg)

            span.set_attribute("alerts_triggered", len(alerts_triggered))
            span.set_attribute("new_alerts", new_alert_count)
            self.metric_alert_count.set(len(self.alerts))

    def run_benchmark(self, event):
        """Run performance benchmark"""
        if not self.enable_benchmarking:
            self.logger.debug("benchmark_skipped_disabled")
            return

        with span_context(self.tracer, "run_benchmark") as span:
            with MetricTimer(self.metric_benchmark_duration):
                self.status = PerformanceMonitorStatus.ANALYZING
                self.metric_status.set(self._status_code())

                try:
                    # Run benchmark tests
                    benchmark_results = {}

                    # FPS benchmark
                    with MetricTimer(self.metric_benchmark_component_duration.labels(component="fps")):
                        benchmark_results['fps_benchmark'] = self.benchmark_fps()

                    # CPU benchmark
                    with MetricTimer(self.metric_benchmark_component_duration.labels(component="cpu")):
                        benchmark_results['cpu_benchmark'] = self.benchmark_cpu()

                    # Memory benchmark
                    with MetricTimer(self.metric_benchmark_component_duration.labels(component="memory")):
                        benchmark_results['memory_benchmark'] = self.benchmark_memory()

                    # Network benchmark
                    with MetricTimer(self.metric_benchmark_component_duration.labels(component="network")):
                        benchmark_results['network_benchmark'] = self.benchmark_network()

                    # Application benchmark
                    with MetricTimer(self.metric_benchmark_component_duration.labels(component="application")):
                        benchmark_results['application_benchmark'] = self.benchmark_application()

                    # Calculate overall performance score
                    performance_scores = []
                    for test_name, result in benchmark_results.items():
                        if 'score' in result:
                            performance_scores.append(result['score'])

                    overall_score = np.mean(performance_scores) if performance_scores else 0.0

                    benchmark_results['overall_performance_score'] = overall_score
                    self.metric_benchmark_score.set(overall_score)
                    span.set_attribute("overall_score", overall_score)

                    # Publish benchmark results
                    if overall_score >= self.alert_threshold:
                        # Publish as alert if performance is poor
                        alert_payload = {
                            'type': 'benchmark_failed',
                            'severity': 'warning',
                            'score': overall_score,
                            'results': benchmark_results,
                            'message': (
                                f"Benchmark performance score: {overall_score:.2f} "
                                f"(threshold: {self.alert_threshold})"
                            )
                        }
                        alert_msg = String()
                        alert_msg.data = json.dumps(alert_payload)
                        self.alert_pub.publish(alert_msg)

                        self.metric_alerts_total.labels(
                            type='benchmark_failed', severity='warning'
                        ).inc()
                        self.logger.warning("benchmark_alert", extra={
                            "score": overall_score,
                            "threshold": self.alert_threshold
                        })

                    # Store benchmark results
                    self.benchmark_results = benchmark_results

                    # Publish benchmark results
                    benchmark_msg = String()
                    benchmark_msg.data = json.dumps(benchmark_results)
                    self.benchmark_pub.publish(benchmark_msg)

                    self.metric_benchmark_runs_total.labels(result="success").inc()
                    self.logger.info("benchmark_completed", extra={
                        "overall_score": overall_score,
                        "component_scores": {
                            k: v.get('score') for k, v in benchmark_results.items()
                            if isinstance(v, dict) and 'score' in v
                        }
                    })
                    rospy.loginfo(f"Benchmark completed with overall score: {overall_score:.2f}")

                except Exception as e:
                    self.metric_benchmark_runs_total.labels(result="failure").inc()
                    self.logger.exception("benchmark_failed", extra={"error": str(e)})
                    rospy.logerr(f"Benchmark failed: {e}")
                    self.error_handler.handle_error(e, "Benchmark execution")
                    span.record_exception(e)

    @traced('performance_monitor', 'benchmark_fps')
    def benchmark_fps(self):
        """Benchmark FPS performance"""
        self.logger.info("benchmark_fps_started")
        rospy.loginfo("Running FPS benchmark...")

        # Record start time
        start_time = time.time()
        frame_count = 0

        # Run benchmark for specified duration
        while time.time() - start_time < self.benchmark_interval:
            # Simulate processing by doing some work
            frame_count += 1

            # Small computation to simulate processing
            result = sum(i * i for i in range(1000))

            # Sleep briefly to simulate processing time
            rospy.sleep(0.001)

        # Calculate FPS
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time if elapsed_time > 0 else 0.0

        # Calculate score (normalized to expected FPS of 30)
        score = min(fps / 30.0, 1.0)

        self.logger.info("benchmark_fps_completed", extra={
            "fps": fps,
            "frames_processed": frame_count,
            "elapsed_time": elapsed_time,
            "score": score
        })

        return {
            'fps': fps,
            'frames_processed': frame_count,
            'elapsed_time': elapsed_time,
            'score': score
        }

    @traced('performance_monitor', 'benchmark_cpu')
    def benchmark_cpu(self):
        """Benchmark CPU performance"""
        self.logger.info("benchmark_cpu_started")
        rospy.loginfo("Running CPU benchmark...")

        # CPU benchmark using psutil
        start_time = time.time()

        # CPU intensive workload
        result = 0
        for i in range(10000000):
            result += i * i

        elapsed_time = time.time() - start_time

        # Calculate score (normalize to reasonable baseline)
        # This is a simplified scoring
        score = min(1.0, elapsed_time / 1.0)  # Should complete in ~1 second

        self.logger.info("benchmark_cpu_completed", extra={
            "elapsed_time": elapsed_time,
            "operations_per_second": 10000000 / elapsed_time if elapsed_time > 0 else 0,
            "score": score
        })

        return {
            'elapsed_time': elapsed_time,
            'operations_per_second': 10000000 / elapsed_time if elapsed_time > 0 else 0,
            'score': score
        }

    @traced('performance_monitor', 'benchmark_memory')
    def benchmark_memory(self):
        """Benchmark memory performance"""
        self.logger.info("benchmark_memory_started")
        rospy.loginfo("Running memory benchmark...")

        # Memory benchmark
        import array

        start_time = time.time()

        # Allocate memory
        memory_blocks = []
        for i in range(100):
            block = array.array('d', [0.0] * 100000)
            memory_blocks.append(block)

        # Access memory
        total = 0
        for block in memory_blocks:
            total += sum(block)

        elapsed_time = time.time() - start_time

        # Clean up
        del memory_blocks

        # Calculate score
        memory_used = psutil.virtual_memory().used
        score = min(1.0, 1.0 / (memory_used / (1024 * 1024 * 100)))  # Normalize to 100MB

        self.logger.info("benchmark_memory_completed", extra={
            "elapsed_time": elapsed_time,
            "memory_used": memory_used,
            "score": score
        })

        return {
            'elapsed_time': elapsed_time,
            'memory_used': memory_used,
            'operations_per_second': 100 / elapsed_time if elapsed_time > 0 else 0,
            'score': score
        }

    @traced('performance_monitor', 'benchmark_network')
    def benchmark_network(self):
        """Benchmark network performance"""
        self.logger.info("benchmark_network_started")
        rospy.loginfo("Running network benchmark...")

        # Network benchmark
        import socket

        start_time = time.time()
        packet_count = 0

        # Create socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0)

        try:
            # Send test packets
            target_host = 'localhost'
            target_port = 11311  # ROS master

            for i in range(100):
                data = f"test_packet_{i}".encode()
                try:
                    sock.sendto(data, (target_host, target_port))
                    packet_count += 1
                except socket.error:
                    pass

                # Brief sleep
                time.sleep(0.01)

        finally:
            sock.close()

        elapsed_time = time.time() - start_time

        # Calculate score
        packets_per_second = packet_count / elapsed_time if elapsed_time > 0 else 0
        score = min(1.0, packets_per_second / 1000.0)  # Normalize to 1000 packets/sec

        self.logger.info("benchmark_network_completed", extra={
            "elapsed_time": elapsed_time,
            "packets_sent": packet_count,
            "packets_per_second": packets_per_second,
            "score": score
        })

        return {
            'elapsed_time': elapsed_time,
            'packets_sent': packet_count,
            'packets_per_second': packets_per_second,
            'score': score
        }

    @traced('performance_monitor', 'benchmark_application')
    def benchmark_application(self):
        """Benchmark application performance"""
        self.logger.info("benchmark_application_started")
        rospy.loginfo("Running application benchmark...")

        # Simulate application workload
        start_time = time.time()

        # Simulate processing pipeline
        results = []
        for i in range(1000):
            # Simulate data processing
            data = {
                'id': i,
                'timestamp': time.time(),
                'data': list(range(100)),
                'processing': np.random.randn(50)
            }

            # Process data
            processed = {
                'sum': sum(data['data']),
                'mean': np.mean(data['processing']),
                'std': np.std(data['processing'])
            }

            results.append(processed)

        elapsed_time = time.time() - start_time

        # Calculate score based on throughput
        processing_per_second = len(results) / elapsed_time if elapsed_time > 0 else 0
        score = min(1.0, processing_per_second / 1000.0)  # Normalize to 1000 operations/sec

        self.logger.info("benchmark_application_completed", extra={
            "elapsed_time": elapsed_time,
            "items_processed": len(results),
            "processing_per_second": processing_per_second,
            "score": score
        })

        return {
            'elapsed_time': elapsed_time,
            'items_processed': len(results),
            'processing_per_second': processing_per_second,
            'score': score
        }

    @traced_ros_callback('performance_monitor', 'performance_command_callback')
    def performance_command_callback(self, msg):
        """Handle performance monitoring commands"""
        try:
            command = json.loads(msg.data)
            command_type = command.get('type')

            self.metric_command_total.labels(command_type=command_type or "unknown").inc()
            self.logger.info("performance_command_received", extra={
                "command_type": command_type
            })

            if command_type == 'start_monitoring':
                self.start_monitoring(None)
                self.logger.info("monitoring_started_via_command")
                rospy.loginfo("Performance monitoring started via command")

            elif command_type == 'stop_monitoring':
                self.stop_monitoring()
                self.logger.info("monitoring_stopped_via_command")
                rospy.loginfo("Performance monitoring stopped via command")

            elif command_type == 'run_benchmark':
                self.run_benchmark(None)
                self.logger.info("benchmark_run_via_command")
                rospy.loginfo("Benchmark run via command")

            elif command_type == 'get_metrics':
                self.publish_current_metrics()

            elif command_type == 'set_thresholds':
                self.set_thresholds(command.get('thresholds', {}))

            else:
                self.logger.warning("unknown_performance_command", extra={
                    "command_type": command_type
                })

        except Exception as e:
            self.logger.warning("performance_command_parse_failed", extra={"error": str(e)})
            rospy.logwarn(f"Could not parse performance command: {e}")

    @traced_ros_callback('performance_monitor', 'benchmark_request_callback')
    def benchmark_request_callback(self, msg):
        """Handle benchmark requests"""
        try:
            request = json.loads(msg.data)
            benchmark_type = request.get('type', 'full')

            self.metric_benchmark_request_total.labels(benchmark_type=benchmark_type).inc()
            self.logger.info("benchmark_request_received", extra={
                "benchmark_type": benchmark_type
            })

            if benchmark_type == 'full':
                self.run_benchmark(None)
            elif benchmark_type == 'fps':
                fps_result = self.benchmark_fps()
                # Publish FPS result only
                fps_msg = String()
                fps_msg.data = json.dumps(fps_result)
                self.benchmark_pub.publish(fps_msg)
            else:
                self.logger.warning("unknown_benchmark_type", extra={
                    "benchmark_type": benchmark_type
                })

        except Exception as e:
            self.logger.warning("benchmark_request_parse_failed", extra={"error": str(e)})
            rospy.logwarn(f"Could not process benchmark request: {e}")

    def set_thresholds(self, thresholds):
        """Set performance thresholds"""
        with span_context(self.tracer, "set_thresholds", {
            "keys": ",".join(thresholds.keys()) if thresholds else ""
        }) as span:
            if 'performance_threshold' in thresholds:
                old = self.performance_threshold
                self.performance_threshold = thresholds['performance_threshold']
                self.logger.info("performance_threshold_updated", extra={
                    "old": old,
                    "new": self.performance_threshold
                })

            if 'alert_threshold' in thresholds:
                old = self.alert_threshold
                self.alert_threshold = thresholds['alert_threshold']
                self.logger.info("alert_threshold_updated", extra={
                    "old": old,
                    "new": self.alert_threshold
                })

            span.set_attribute("performance_threshold", self.performance_threshold)
            span.set_attribute("alert_threshold", self.alert_threshold)

    def publish_current_metrics(self):
        """Publish current metrics"""
        with span_context(self.tracer, "publish_current_metrics") as span:
            metrics = {
                'timestamp': time.time(),
                'status': self.status.value,
                'is_monitoring': self.is_monitoring,
                'fps_data': self.fps_data[-10:] if len(self.fps_data) >= 10 else self.fps_data,
                'cpu_data': self.cpu_data[-10:] if len(self.cpu_data) >= 10 else self.cpu_data,
                'memory_data': self.memory_data[-10:] if len(self.memory_data) >= 10 else self.memory_data,
                'network_data': self.network_data[-10:] if len(self.network_data) >= 10 else self.network_data,
                'alerts': self.alerts[-10:] if len(self.alerts) >= 10 else self.alerts,
                'benchmark_results': self.benchmark_results
            }

            metrics_msg = String()
            metrics_msg.data = json.dumps(metrics)
            self.metrics_pub.publish(metrics_msg)

            span.set_attribute("fps_samples_published", len(metrics['fps_data']))
            span.set_attribute("alert_count", len(metrics['alerts']))
            self.logger.debug("metrics_published", extra={
                "status": self.status.value,
                "is_monitoring": self.is_monitoring,
                "fps_samples": len(metrics['fps_data']),
                "alert_count": len(metrics['alerts'])
            })

    def stop_monitoring(self):
        """Stop performance monitoring"""
        with span_context(self.tracer, "stop_monitoring") as span:
            self.is_monitoring = False
            self.status = PerformanceMonitorStatus.IDLE
            self.metric_is_monitoring.set(0)
            self.metric_status.set(self._status_code())

            if self.benchmark_timer:
                self.benchmark_timer.shutdown()
                self.benchmark_timer = None

            if self.collection_timer:
                self.collection_timer.shutdown()
                self.collection_timer = None

            span.set_attribute("stopped", True)
            self.logger.info("monitoring_stopped")
            rospy.loginfo("Performance monitoring stopped")

    def get_performance_summary(self):
        """Get current performance summary"""
        return {
            'status': self.status.value,
            'is_monitoring': self.is_monitoring,
            'avg_fps': np.mean([d['fps'] for d in self.fps_data]) if self.fps_data else 0.0,
            'avg_cpu': np.mean([d['cpu_percent'] for d in self.cpu_data]) if self.cpu_data else 0.0,
            'avg_memory': np.mean([d['memory_percent'] for d in self.memory_data]) if self.memory_data else 0.0,
            'alert_count': len(self.alerts),
            'benchmark_score': self.benchmark_results.get('overall_performance_score', 0.0),
            'last_data_collection': self.metrics_data.get('last_frame_time', 0.0)
        }


if __name__ == '__main__':
    try:
        monitor = PerformanceMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass