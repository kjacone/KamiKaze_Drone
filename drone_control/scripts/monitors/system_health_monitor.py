import rospy
import psutil
import time
import sys
import os
import json
import subprocess
from enum import Enum
from std_msgs.msg import String
from drone_control.msg import NodeHealth, DiagnosticStatus
import socket

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))

try:
    from error_handler import ErrorHandler
except ImportError:
    class ErrorHandler:
        def __init__(self, node_name):
            self.node_name = node_name
        def handle_error(self, error, context=None):
            rospy.logerr(f"[{self.node_name}] {error}: {context}")

class SystemHealthStatus(Enum):
    HEALTHY = "healthy"
    WARNING = "warning"
    CRITICAL = "critical"
    UNKNOWN = "unknown"

class SystemHealthMonitor:
    """Comprehensive system health monitoring and resource management"""

    def __init__(self):
        rospy.init_node('system_health_monitor', anonymous=False)

        self.error_handler = ErrorHandler(node_name='system_health_monitor')

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

        # Subscribers
        self.health_check_sub = rospy.Subscriber('/health/check_all', String, self.check_all_callback)

        # Publishers
        self.health_pub = rospy.Publisher('/system_health', NodeHealth, queue_size=10)
        self.diagnostic_pub = rospy.Publisher('/diagnostic_status', DiagnosticStatus, queue_size=10)
        self.performance_pub = rospy.Publisher('/performance_metrics', String, queue_size=10)
        self.alert_pub = rospy.Publisher('/system_alert', String, queue_size=10)

        # Timers
        self.health_check_timer = rospy.Timer(rospy.Duration(self.monitoring_interval), self.health_check_loop)
        self.background_monitor_timer = rospy.Timer(rospy.Duration(30.0), self.background_monitor)

        rospy.loginfo("System Health Monitor initialized")

    def health_check_loop(self, event):
        """Main health check loop"""
        current_time = time.time()

        # Collect system health metrics
        self.collect_system_metrics()

        # Check for issues
        self.check_for_issues()

        # Update overall status and publish it (this was previously never
        # triggered on a timer, so /system_health only updated on-demand)
        self.publish_status(event)

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
                except Exception as e:
                    rospy.logwarn(f"Could not get disk usage for {partition.mountpoint}: {e}")

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

        # Service status check (populates self.service_warning_issues,
        # which check_for_issues() merges in afterwards)
        self.check_service_status()

        self.last_check_time = current_time

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

    def check_for_issues(self):
        """Check for critical and warning issues"""
        self.critical_issues = []
        self.warning_issues = []

        # Check CPU
        if self.cpu_info.get('percent', 0.0) >= self.critical_threshold_cpu:
            self.critical_issues.append(f"CPU usage critical: {self.cpu_info.get('percent', 0.0):.1f}%")
        elif self.cpu_info.get('percent', 0.0) >= self.warning_threshold_cpu:
            self.warning_issues.append(f"CPU usage warning: {self.cpu_info.get('percent', 0.0):.1f}%")

        # Check memory
        if self.memory_info.get('percent', 0.0) >= self.critical_threshold_memory:
            self.critical_issues.append(f"Memory usage critical: {self.memory_info.get('percent', 0.0):.1f}%")
        elif self.memory_info.get('percent', 0.0) >= self.warning_threshold_memory:
            self.warning_issues.append(f"Memory usage warning: {self.memory_info.get('percent', 0.0):.1f}%")

        # Check disk space
        for mountpoint, info in self.disk_info.items():
            if info['percent'] >= 90:
                self.critical_issues.append(f"Disk usage critical ({mountpoint}): {info['percent']:.1f}%")
            elif info['percent'] >= 80:
                self.warning_issues.append(f"Disk usage warning ({mountpoint}): {info['percent']:.1f}%")

        # Check temperature (reuse the value already collected instead of
        # calling get_cpu_temperature() a second time)
        cpu_temp = self.cpu_info.get('temperature')
        if cpu_temp and cpu_temp >= 80.0:
            self.warning_issues.append(f"CPU temperature warning: {cpu_temp:.1f}\u00b0C")

        # Check network connectivity
        if self.enable_network_monitoring:
            self.check_network_connectivity()

        # Merge in service-status warnings collected during
        # collect_system_metrics(); previously these were wiped out because
        # this method reset warning_issues *after* check_service_status()
        # had already appended to it.
        self.warning_issues.extend(self.service_warning_issues)

    def check_service_status(self):
        """Check status of critical services"""
        critical_services = [
            'ros', 'docker', 'px4', 'mavros', 'gazebo'
        ]

        self.service_warning_issues = []
        for service in critical_services:
            try:
                # Check if service is running
                result = subprocess.run(['systemctl', 'is-active', service],
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    self.service_status[service] = 'running'
                else:
                    self.service_status[service] = 'stopped'
                    self.service_warning_issues.append(f"Service {service} is not running")
            except Exception as e:
                self.service_status[service] = 'unknown'
                self.service_warning_issues.append(f"Could not check service {service}: {e}")

    def check_network_connectivity(self):
        """Check network connectivity and endpoints"""
        # Check localhost (ROS master)
        self._check_tcp_port("127.0.0.1", 11311, "ROS master (localhost:11311)")

        # Check PX4
        self._check_tcp_port("127.0.0.1", 14540, "PX4 UDP port (14540)")

        # Check MAVROS
        self._check_tcp_port("127.0.0.1", 14550, "MAVROS UDP port (14550)")

    def _check_tcp_port(self, host, port, label):
        """Attempt a TCP connection and always close the socket afterwards.

        The original code called socket.create_connection() and discarded
        the returned socket object without closing it, leaking a file
        descriptor on every successful check.
        """
        try:
            with socket.create_connection((host, port), timeout=2):
                pass
        except OSError:
            self.warning_issues.append(f"Cannot connect to {label}")

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
        except Exception:
            pass

        # Try alternative method
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read()) / 1000.0
                return temp
        except Exception:
            pass

        return None

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

    def check_all_callback(self, msg):
        """Handle comprehensive health check request"""
        # Trigger immediate health check
        self.collect_system_metrics()
        self.check_for_issues()

        # Publish comprehensive health status
        self.publish_status(None)

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