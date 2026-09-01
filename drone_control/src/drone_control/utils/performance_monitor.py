#!/usr/bin/env python3
"""
drone_control/src/drone_control/utils/performance_monitor.py
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

# sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../catkin_ws/src/drone_control/scripts'))

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

        self.error_handler = ErrorHandler(node_name='performance_monitor')

        # Configuration parameters
        self.monitoring_duration = rospy.get_param('~monitoring_duration', 60.0)
        self.collection_interval = rospy.get_param('~collection_interval', 0.1)
        self.benchmark_interval = rospy.get_param('~benchmark_interval', 5.0)
        self.performance_threshold = rospy.get_param('~performance_threshold', 0.9)
        self.alert_threshold = rospy.get_param('~alert_threshold', 0.95)
        self.enable_benchmarking = rospy.get_param('~enable_benchmarking', True)
        self.simulation_mode = rospy.get_param('/use_simulation', True)

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

        # Subscribers
        self.performance_command_sub = rospy.Subscriber('/performance/command', String, self.performance_command_callback)
        self.benchmark_request_sub = rospy.Subscriber('/performance/benchmark', String, self.benchmark_request_callback)

        # Publishers
        self.metrics_pub = rospy.Publisher('/performance/metrics', String, queue_size=10)
        self.benchmark_pub = rospy.Publisher('/performance/benchmark_results', String, queue_size=10)
        self.alert_pub = rospy.Publisher('/performance/alert', String, queue_size=10)
        self.diagnostic_pub = rospy.Publisher('/diagnostic_status', DiagnosticStatus, queue_size=10)

        # Timers
        # NOTE: oneshot=True — without this, rospy.Timer fires every 2s forever,
        # and start_monitoring() would keep creating new benchmark/collection
        # timers without ever canceling the old ones (a resource/thread leak).
        self.startup_monitor = rospy.Timer(rospy.Duration(2.0), self.start_monitoring, oneshot=True)

        rospy.loginfo("Performance Monitor initialized")

    def start_monitoring(self, event):
        """Start performance monitoring"""
        if self.is_monitoring:
            # Already running — don't stack duplicate timers/threads.
            rospy.logdebug("start_monitoring called while already monitoring; ignoring")
            return

        if self.benchmark_interval > 0:
            self.benchmark_timer = rospy.Timer(rospy.Duration(self.benchmark_interval), self.run_benchmark)

        if self.collection_interval > 0:
            self.collection_timer = rospy.Timer(rospy.Duration(self.collection_interval), self.collect_metrics)

        self.status = PerformanceMonitorStatus.MONITORING
        self.is_monitoring = True

        rospy.loginfo("Performance monitoring started")

    def collect_metrics(self, event):
        """Collect performance metrics"""
        if not self.is_monitoring:
            return

        self.status = PerformanceMonitorStatus.COLLECTING

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

        self.metrics_data['last_frame_time'] = current_time

        # CPU usage
        cpu_percent = psutil.cpu_percent(interval=None)
        self.cpu_data.append({
            'timestamp': current_time,
            'cpu_percent': cpu_percent,
            'cpu_cores': psutil.cpu_count(logical=False),
            'cpu_logical_cores': psutil.cpu_count(logical=True)
        })

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

    def collect_application_metrics(self):
        """Collect application-specific performance metrics"""
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

        except Exception as e:
            rospy.logwarn(f"Could not collect ROS metrics: {e}")

        # Collect node-specific metrics
        try:
            from rosnode import get_node_names
            self.metrics_data['ros_node_count'] = len(get_node_names())
        except Exception as e:
            rospy.logwarn(f"Could not collect ROS node metrics: {e}")

    def clean_old_data(self):
        """Remove old metric data to keep memory usage manageable"""
        current_time = time.time()
        cutoff_time = current_time - 300.0  # 5 minutes ago

        # Clean FPS data
        self.fps_data = [d for d in self.fps_data if d['timestamp'] > cutoff_time]

        # Clean CPU data
        self.cpu_data = [d for d in self.cpu_data if d['timestamp'] > cutoff_time]

        # Clean memory data
        self.memory_data = [d for d in self.memory_data if d['timestamp'] > cutoff_time]

        # Clean network data
        self.network_data = [d for d in self.network_data if d['timestamp'] > cutoff_time]

    def check_for_alerts(self):
        """Check for performance alerts"""
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
        for alert in alerts_triggered:
            if alert not in self.alerts:
                self.alerts.append(alert)

                # Publish alert
                alert_msg = String()
                alert_msg.data = json.dumps(alert)
                self.alert_pub.publish(alert_msg)

    def run_benchmark(self, event):
        """Run performance benchmark"""
        if not self.enable_benchmarking:
            return

        self.status = PerformanceMonitorStatus.ANALYZING

        try:
            # Run benchmark tests
            benchmark_results = {}

            # FPS benchmark
            benchmark_results['fps_benchmark'] = self.benchmark_fps()

            # CPU benchmark
            benchmark_results['cpu_benchmark'] = self.benchmark_cpu()

            # Memory benchmark
            benchmark_results['memory_benchmark'] = self.benchmark_memory()

            # Network benchmark
            benchmark_results['network_benchmark'] = self.benchmark_network()

            # Application benchmark
            benchmark_results['application_benchmark'] = self.benchmark_application()

            # Calculate overall performance score
            performance_scores = []
            for test_name, result in benchmark_results.items():
                if 'score' in result:
                    performance_scores.append(result['score'])

            overall_score = np.mean(performance_scores) if performance_scores else 0.0

            benchmark_results['overall_performance_score'] = overall_score

            # Publish benchmark results
            if overall_score >= self.alert_threshold:
                # Publish as alert if performance is poor
                alert_msg = String()
                alert_msg.data = json.dumps({
                    'type': 'benchmark_failed',
                    'severity': 'warning',
                    'score': overall_score,
                    'results': benchmark_results,
                    'message': f"Benchmark performance score: {overall_score:.2f} (threshold: {self.alert_threshold})"
                })
                self.alert_pub.publish(alert_msg)

            # Store benchmark results
            self.benchmark_results = benchmark_results

            # Publish benchmark results
            benchmark_msg = String()
            benchmark_msg.data = json.dumps(benchmark_results)
            self.benchmark_pub.publish(benchmark_msg)

            rospy.loginfo(f"Benchmark completed with overall score: {overall_score:.2f}")

        except Exception as e:
            rospy.logerr(f"Benchmark failed: {e}")
            self.error_handler.handle_error(e, "Benchmark execution")

    def benchmark_fps(self):
        """Benchmark FPS performance"""
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

        return {
            'fps': fps,
            'frames_processed': frame_count,
            'elapsed_time': elapsed_time,
            'score': score
        }

    def benchmark_cpu(self):
        """Benchmark CPU performance"""
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

        return {
            'elapsed_time': elapsed_time,
            'operations_per_second': 10000000 / elapsed_time if elapsed_time > 0 else 0,
            'score': score
        }

    def benchmark_memory(self):
        """Benchmark memory performance"""
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

        return {
            'elapsed_time': elapsed_time,
            'memory_used': memory_used,
            'operations_per_second': 100 / elapsed_time if elapsed_time > 0 else 0,
            'score': score
        }

    def benchmark_network(self):
        """Benchmark network performance"""
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

        return {
            'elapsed_time': elapsed_time,
            'packets_sent': packet_count,
            'packets_per_second': packets_per_second,
            'score': score
        }

    def benchmark_application(self):
        """Benchmark application performance"""
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

        return {
            'elapsed_time': elapsed_time,
            'items_processed': len(results),
            'processing_per_second': processing_per_second,
            'score': score
        }

    def performance_command_callback(self, msg):
        """Handle performance monitoring commands"""
        try:
            command = json.loads(msg.data)
            command_type = command.get('type')

            if command_type == 'start_monitoring':
                self.start_monitoring(None)
                rospy.loginfo("Performance monitoring started via command")

            elif command_type == 'stop_monitoring':
                self.stop_monitoring()
                rospy.loginfo("Performance monitoring stopped via command")

            elif command_type == 'run_benchmark':
                self.run_benchmark(None)
                rospy.loginfo("Benchmark run via command")

            elif command_type == 'get_metrics':
                self.publish_current_metrics()

            elif command_type == 'set_thresholds':
                self.set_thresholds(command.get('thresholds', {}))

        except Exception as e:
            rospy.logwarn(f"Could not parse performance command: {e}")

    def benchmark_request_callback(self, msg):
        """Handle benchmark requests"""
        try:
            request = json.loads(msg.data)
            benchmark_type = request.get('type', 'full')

            if benchmark_type == 'full':
                self.run_benchmark(None)
            elif benchmark_type == 'fps':
                fps_result = self.benchmark_fps()
                # Publish FPS result only
                fps_msg = String()
                fps_msg.data = json.dumps(fps_result)
                self.benchmark_pub.publish(fps_msg)

        except Exception as e:
            rospy.logwarn(f"Could not process benchmark request: {e}")

    def set_thresholds(self, thresholds):
        """Set performance thresholds"""
        if 'performance_threshold' in thresholds:
            self.performance_threshold = thresholds['performance_threshold']

        if 'alert_threshold' in thresholds:
            self.alert_threshold = thresholds['alert_threshold']

    def publish_current_metrics(self):
        """Publish current metrics"""
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

    def stop_monitoring(self):
        """Stop performance monitoring"""
        self.is_monitoring = False
        self.status = PerformanceMonitorStatus.IDLE

        if self.benchmark_timer:
            self.benchmark_timer.shutdown()
            self.benchmark_timer = None

        if self.collection_timer:
            self.collection_timer.shutdown()
            self.collection_timer = None

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