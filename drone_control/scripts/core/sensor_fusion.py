#!/usr/bin/env python3
"""
drone_control/scripts/core/sensor_fusion.py
Sensor fusion node
"""

import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix

from drone_control.utils.tracing import init_tracing, get_component_tracer, span_context
from drone_control.msg import NodeHealth
from drone_control.utils.logging_framework import (
    get_logger_with_ros_level,
)
from drone_control.utils.metrics_collector import MetricsCollector, MetricTimer
from drone_control.utils.ros_tracing import (
    create_traced_publisher,
    traced_ros_callback,
)


class SensorFusion:
    """Sensor fusion node"""

    def __init__(self):
        # Initialize ROS node FIRST
        rospy.init_node('sensor_fusion', anonymous=False)

        # Initialize structured logger
        self.logger = get_logger_with_ros_level("sensor_fusion")
        self.logger.info("node_initializing", extra={"version": "1.0.0"})

        # Tracing setup, before anything else registers publishers/timers
        init_tracing(component='sensor_fusion')
        self.tracer = get_component_tracer('sensor_fusion')

        self.publish_rate = rospy.get_param('~publish_rate', 20)
        self.simulation_mode = rospy.get_param('/use_simulation', True)
        self.test_mode = rospy.get_param('/test_mode', False)

        # How long a sensor can go without a new message before it's
        # considered stale, for the health check below.
        self.staleness_threshold = rospy.get_param('~staleness_threshold', 2.0)

        # Initialize metrics collector (parameters above must be set first)
        self.metrics = MetricsCollector("sensor_fusion", port=8004)
        self._init_metrics()

        # State
        self.imu_data = None
        self.gps_data = None
        self.odom_data = None

        # Last-received timestamps for each sensor. publish_health() used
        # to report is_healthy=True unconditionally, regardless of whether
        # any sensor data had ever arrived or had since stopped arriving --
        # which defeats the point of health_checker.py restarting nodes
        # based on is_healthy. These let us actually check staleness.
        self.last_imu_time = None
        self.last_gps_time = None
        self.last_pose_time = None

        # Subscribers
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Publishers
        self.health_pub = create_traced_publisher(
            '/sensor_fusion/node_health', NodeHealth, queue_size=10, tracer_name='sensor_fusion'
        )

        # Timer
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self.publish_health)

        self.logger.info("node_initialized", extra={
            "publish_rate": self.publish_rate,
            "staleness_threshold": self.staleness_threshold,
            "simulation_mode": self.simulation_mode
        })
        rospy.loginfo("Sensor Fusion initialized")

    def _init_metrics(self):
        """Initialize Prometheus metrics for this node"""
        self.logger.debug("initializing_metrics")

        # Counters
        self.metric_sensor_messages_total = self.metrics.counter(
            "sensor_messages_total",
            "Total messages received per sensor",
            labels=["sensor"]
        )
        self.metric_health_checks_total = self.metrics.counter(
            "health_checks_total",
            "Total health check publications"
        )

        # Gauges
        self.metric_sensor_fresh = self.metrics.gauge(
            "sensor_fresh",
            "Whether a sensor's data is within the staleness threshold (1=fresh, 0=stale)",
            labels=["sensor"]
        )
        self.metric_sensor_age_seconds = self.metrics.gauge(
            "sensor_age_seconds",
            "Seconds since the last message received from a sensor",
            labels=["sensor"]
        )
        self.metric_health_status = self.metrics.gauge(
            "health_status",
            "Overall sensor fusion health (1=healthy, 0=degraded)"
        )

        self.logger.info("metrics_initialized")

    @traced_ros_callback('sensor_fusion', 'imu_callback')
    def imu_callback(self, msg):
        self.imu_data = msg
        self.last_imu_time = time.time()
        self.metric_sensor_messages_total.labels(sensor="imu").inc()

    @traced_ros_callback('sensor_fusion', 'gps_callback')
    def gps_callback(self, msg):
        self.gps_data = msg
        self.last_gps_time = time.time()
        self.metric_sensor_messages_total.labels(sensor="gps").inc()

    @traced_ros_callback('sensor_fusion', 'pose_callback')
    def pose_callback(self, msg):
        self.odom_data = msg
        self.last_pose_time = time.time()
        self.metric_sensor_messages_total.labels(sensor="pose").inc()

    def publish_health(self, event):
        with span_context(self.tracer, "publish_health") as span:
            now = time.time()

            def age(last_seen):
                return (now - last_seen) if last_seen is not None else None

            def is_fresh(last_seen):
                return last_seen is not None and (now - last_seen) < self.staleness_threshold

            imu_age = age(self.last_imu_time)
            gps_age = age(self.last_gps_time)
            pose_age = age(self.last_pose_time)

            imu_ok = is_fresh(self.last_imu_time)
            gps_ok = is_fresh(self.last_gps_time)
            pose_ok = is_fresh(self.last_pose_time)
            is_healthy = imu_ok and gps_ok and pose_ok

            self.metric_sensor_fresh.labels(sensor="imu").set(1 if imu_ok else 0)
            self.metric_sensor_fresh.labels(sensor="gps").set(1 if gps_ok else 0)
            self.metric_sensor_fresh.labels(sensor="pose").set(1 if pose_ok else 0)
            if imu_age is not None:
                self.metric_sensor_age_seconds.labels(sensor="imu").set(imu_age)
            if gps_age is not None:
                self.metric_sensor_age_seconds.labels(sensor="gps").set(gps_age)
            if pose_age is not None:
                self.metric_sensor_age_seconds.labels(sensor="pose").set(pose_age)
            self.metric_health_status.set(1 if is_healthy else 0)
            self.metric_health_checks_total.inc()

            span.set_attribute("imu_ok", imu_ok)
            span.set_attribute("gps_ok", gps_ok)
            span.set_attribute("pose_ok", pose_ok)
            span.set_attribute("is_healthy", is_healthy)

            health_msg = NodeHealth()
            health_msg.node_name = 'sensor_fusion'
            health_msg.status = 'running' if is_healthy else 'degraded'
            health_msg.timestamp = rospy.Time.now()
            health_msg.is_healthy = is_healthy
            self.health_pub.publish(health_msg)

            if not is_healthy:
                rospy.logwarn(
                    f"Sensor fusion degraded - imu_ok={imu_ok}, gps_ok={gps_ok}, pose_ok={pose_ok}"
                )
                self.logger.warning("sensor_fusion_degraded", extra={
                    "imu_ok": imu_ok,
                    "gps_ok": gps_ok,
                    "pose_ok": pose_ok,
                    "imu_age": imu_age,
                    "gps_age": gps_age,
                    "pose_age": pose_age
                })
            else:
                self.logger.debug("health_published", extra={"is_healthy": True})


if __name__ == '__main__':
    try:
        fusion = SensorFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass