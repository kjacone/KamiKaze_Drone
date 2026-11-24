#!/usr/bin/env python3
"""
drone_control/scripts/controllers/target_tracking_controller.py
Target provider for the mission manager.

This node never commands the vehicle. It turns detections into
/tracked_targets; target_follower owns the control setpoint.
"""

import math
import time

import numpy as np
import psutil
import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from drone_control.utils import ErrorHandler

from drone_control.msg import (
    DetectedObjects,
    MissionStatus,
    NodeHealth,
    TrackedTarget,
    TrackedTargets,
)

try:
    from filterpy.kalman import KalmanFilter
except ImportError:
    # Fallback simple Kalman filter
    class KalmanFilter:
        def __init__(self, dim_x, dim_z):
            self.dim_x = dim_x
            self.dim_z = dim_z
            self.x = np.zeros(dim_x)
            self.P = np.eye(dim_x)
            self.F = np.eye(dim_x)
            self.H = np.eye(dim_z, dim_x)
            self.R = np.eye(dim_z)
            self.Q = np.eye(dim_x)

        def predict(self):
            self.x = self.F @ self.x
            self.P = self.F @ self.P @ self.F.T + self.Q

        def update(self, z):
            y = z - self.H @ self.x
            S = self.H @ self.P @ self.H.T + self.R
            K = self.P @ self.H.T @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.P = (np.eye(self.dim_x) - K @ self.H) @ self.P


class TargetTrackingController:
    """Track detections and publish them as targets, no vehicle commands."""

    def __init__(self):
        rospy.init_node('target_tracking_controller', anonymous=False)

        self.error_handler = ErrorHandler(node_name='target_tracking_controller')

        self.state = "SEARCHING"
        self.previous_state = None
        self.state_start_time = time.time()

        self.error_count = 0

        self.engagement_distance = rospy.get_param('tracking/engagement_distance', 10.0)
        self.disengagement_distance = rospy.get_param('tracking/disengagement_distance', 15.0)
        self.simulation_mode = rospy.get_param('use_simulation', True)
        self.debug_mode = rospy.get_param('debug_mode', False)
        self.test_mode = rospy.get_param('test_mode', False)

        # What this mission accepts as its target: a detection is only fed
        # into the filter if its class is in this list; repeated non-matching
        # detections drop an existing track (the "not my target" case).
        self.target_classes = set(
            rospy.get_param('~target_classes', ['car', 'truck', 'bus']))
        self.reject_frames = int(rospy.get_param('~reject_frames', 5))
        # mono_cam sits at (.12, .03, .242) on base_link.
        self.camera_offset_body = np.array(
            rospy.get_param('~camera_offset', [0.12, 0.03, 0.242]))

        self.odom_sub = rospy.Subscriber(
            'mavros/local_position/odom', Odometry, self.odometry_callback
        )
        self.det_sub = rospy.Subscriber(
            'detected_objects', DetectedObjects, self.detections_callback
        )
        self.camera_sub = rospy.Subscriber(
            'camera/color/camera_info', CameraInfo, self.camera_info_callback
        )
        self.state_sub = rospy.Subscriber(
            'mavros/state', State, self.mavros_state_callback
        )
        self.mission_sub = rospy.Subscriber(
            'mission_status', MissionStatus, self._handle_mission_status
        )
        self.command_sub = rospy.Subscriber(
            'tracking_command', String, self._handle_tracking_command
        )

        self.target_pub = rospy.Publisher(
            'tracked_targets', TrackedTargets, queue_size=10
        )
        self.health_pub = rospy.Publisher(
            'target_tracking_controller/node_health', NodeHealth, queue_size=10
        )

        self.drone_pose = None
        self.camera_info = None
        self.target_kf = None
        self.mavros_state = None
        self.tracking_start_time = None
        self.target_id = 0
        self.last_confidence = 0.0
        self.tracked_targets = []
        self.reject_streak = 0
        self.last_measurement_time = None
        self.last_rejection = None
        # Operator override: while held, detections are ignored and any
        # track is dropped, so the vehicle stays in its search hold.
        self.operator_hold = False
        # Manual override: the operator judged the classifier wrong, so
        # detections of any class are accepted as the mission target.
        self.operator_override = False
        self._override_log = 0.0
        self.measurement_noise_base = rospy.get_param(
            'tracking/kalman/measurement_noise', 0.05)

        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)

        self.rate = rospy.Rate(20)
        rospy.loginfo("Target tracking controller started as target provider")

    def _handle_mission_status(self, msg):
        rospy.logdebug(f"Mission state: {msg.state}")

        mapping = {
            'emergency': 'EMERGENCY',
            'paused': 'PAUSED',
            'completed': 'SEARCHING',
            'failed': 'SEARCHING',
            'searching': 'SEARCHING',
            'tracking': 'TRACKING',
            'engaging': 'ENGAGING',
            'initializing': 'SEARCHING',
            'idle': 'SEARCHING',
        }
        new_state = mapping.get(str(msg.state).lower())
        if new_state is None:
            return

        self._set_state(new_state)
        if new_state in ("SEARCHING", "PAUSED", "EMERGENCY"):
            # Do not re-feed a stale track after the mission dropped it.
            self.target_kf = None
            self.target_id = 0
            self.last_confidence = 0.0
            self.reject_streak = 0

    def _set_state(self, new_state):
        if new_state != self.state:
            self.previous_state = self.state
            self.state = new_state
            self.state_start_time = time.time()
            rospy.loginfo(f"State changed: {self.previous_state} -> {self.state}")

    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def odometry_callback(self, msg):
        self.drone_pose = msg.pose.pose

    def camera_info_callback(self, msg):
        self.camera_info = msg

    # -- geometry -----------------------------------------------------------

    @staticmethod
    def _body_to_enu(pose):
        q = pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def pixel_to_enu(self, bbox):
        """Project a bbox ground-contact point onto z=0 given the camera pose.

        Camera optical frame (right, down, forward) maps to body FLU, then
        into the local ENU frame via the attitude quaternion. Only valid for
        ground targets; a ray at or above the horizon returns None.
        """
        if self.camera_info is None or self.drone_pose is None:
            return None
        k = self.camera_info.K
        fx, cx = k[0], k[2]
        fy, cy = k[4], k[5]
        if fx <= 0 or fy <= 0:
            return None

        # Bottom-centre of the bbox = where the object meets the ground.
        u = bbox.x + bbox.width / 2.0
        v = bbox.y + bbox.height
        ray_optical = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        ray_body = np.array(
            [ray_optical[2], -ray_optical[0], -ray_optical[1]])

        rot = self._body_to_enu(self.drone_pose)
        ray_world = rot @ ray_body
        drone_pos = np.array([self.drone_pose.position.x,
                              self.drone_pose.position.y,
                              self.drone_pose.position.z])
        cam_pos = drone_pos + rot @ self.camera_offset_body
        if ray_world[2] >= -1e-3:
            return None
        t = cam_pos[2] / -ray_world[2]
        if t <= 0:
            return None
        return cam_pos + t * ray_world

    # -- detections ---------------------------------------------------------

    def _handle_tracking_command(self, msg):
        command = msg.data.strip().lower()
        if command not in ('hold', 'engage', 'override'):
            rospy.logwarn("Unknown tracking_command '%s'", command)
            return
        if command == 'hold':
            self.operator_hold = True
            self.operator_override = False
            self.target_kf = None
            self.target_id = 0
            self.last_confidence = 0.0
            self.reject_streak = 0
            self._set_state("SEARCHING")
            rospy.loginfo("Operator hold engaged; tracking released")
        elif command == 'override':
            self.operator_hold = False
            self.operator_override = True
            self.reject_streak = 0
            rospy.loginfo(
                "Operator override: classifier bypassed, any detected "
                "object is treated as the mission target")
        else:
            self.operator_hold = False
            self.operator_override = False
            rospy.loginfo(
                "Operator hold/override released; class check restored")

    def detections_callback(self, msg):
        """Feed verified detections into the filter; reject non-targets."""
        if self.operator_hold:
            return
        if self.state not in ["SEARCHING", "TRACKING", "ENGAGING"] or not msg.objects:
            return

        best = max(msg.objects, key=lambda o: o.confidence)
        if self.operator_override:
            now = time.time()
            if now - self._override_log > 5.0:
                self._override_log = now
                rospy.loginfo(
                    "Override active: treating '%s' as the mission target",
                    best.class_name)
        elif best.class_name not in self.target_classes:
            self._reject(best)
            return

        measurement = self.pixel_to_enu(best.bbox)
        if measurement is None:
            return

        self.reject_streak = 0
        if self.state == "SEARCHING":
            self.initialize_kalman_filter(best, measurement)
            self._set_state("TRACKING")
            self.tracking_start_time = time.time()
            rospy.loginfo(
                "Target acquired (%s @ %.1f,%.1f) -> TRACKING",
                best.class_name, measurement[0], measurement[1])
        elif self.state == "TRACKING":
            self._update_target_position(best, measurement)

    def _reject(self, detection):
        self.reject_streak += 1
        if self.target_kf is None or self.reject_streak < self.reject_frames:
            return
        self.last_rejection = detection.class_name
        rospy.logwarn(
            "Vision check failed: '%s' is not a mission target; dropping track",
            detection.class_name)
        self.target_kf = None
        self.target_id = 0
        self.last_confidence = 0.0
        self.reject_streak = 0
        self._set_state("SEARCHING")

    def initialize_kalman_filter(self, detection, measurement):
        """Initialize Kalman filter for target tracking."""
        self.target_kf = KalmanFilter(dim_x=6, dim_z=3)
        dt = 0.1

        self.target_kf.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])

        self.target_kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ])

        self.target_kf.P *= 10.0
        self.target_kf.R = np.eye(3) * self.measurement_noise_base
        self.target_kf.Q = np.eye(6) * rospy.get_param(
            'tracking/kalman/process_noise', 0.1
        )

        self.target_kf.x = np.array(
            [measurement[0], measurement[1], measurement[2], 0, 0, 0]
        )
        self.last_measurement_time = time.time()

        self.target_id += 1
        self.last_confidence = detection.confidence

    def _measurement_noise(self, measurement):
        """Pixel error grows with range: loosen R as the ground point recedes."""
        range_m = math.hypot(measurement[0], measurement[1])
        return self.measurement_noise_base * max(1.0, range_m / 10.0) ** 2

    def _update_target_position(self, detection, measurement):
        if self.target_kf is None:
            return

        now = time.time()
        dt = now - (self.last_measurement_time or now)
        dt = min(max(dt, 0.02), 0.5)
        self.target_kf.F[0, 3] = dt
        self.target_kf.F[1, 4] = dt
        self.target_kf.F[2, 5] = dt
        self.target_kf.predict()

        noise = self._measurement_noise(measurement)
        self.target_kf.R = np.eye(3) * noise
        self.target_kf.update(measurement)
        self.last_measurement_time = now
        self.last_confidence = detection.confidence

    def run(self):
        while not rospy.is_shutdown():
            try:
                self._publish_tracked_targets()
            except Exception as e:
                self.error_handler.handle_error(e, "Target publish error")
                rospy.logwarn(f"Error publishing target: {e}")

            self.rate.sleep()

    def _publish_tracked_targets(self):
        now = rospy.Time.now()
        targets_msg = TrackedTargets()
        targets_msg.header.stamp = now
        targets_msg.header.frame_id = 'map'

        if self.target_kf is None:
            targets_msg.count = 0
            self.target_pub.publish(targets_msg)
            return

        target = TrackedTarget()
        target.id = self.target_id
        target.position.x = self.target_kf.x[0]
        target.position.y = self.target_kf.x[1]
        target.position.z = self.target_kf.x[2]
        target.velocity.x = self.target_kf.x[3]
        target.velocity.y = self.target_kf.x[4]
        target.velocity.z = self.target_kf.x[5]
        target.confidence = self.last_confidence
        target.state = self.state.lower()

        if self.drone_pose is not None:
            drone_pos = np.array([
                self.drone_pose.position.x,
                self.drone_pose.position.y,
                self.drone_pose.position.z,
            ])
            target.distance = float(
                np.linalg.norm(self.target_kf.x[:3] - drone_pos)
            )
        else:
            target.distance = 0.0

        target.heading = math.atan2(target.position.y, target.position.x)
        target.timestamp = now

        targets_msg.targets.append(target)
        targets_msg.count = 1
        self.target_pub.publish(targets_msg)

    def _publish_health(self, event):
        try:
            health_msg = NodeHealth()
            health_msg.node_name = 'target_tracking_controller'
            health_msg.status = self.state
            health_msg.timestamp = rospy.Time.now()

            is_healthy = True
            if self.state == "EMERGENCY":
                is_healthy = False
                rospy.logwarn_throttle(5.0, "Node in EMERGENCY state")

            if self.state == "TRACKING":
                if self.target_kf is None:
                    is_healthy = False
                    rospy.logwarn_throttle(5.0, "Kalman filter not initialized")
                elif self.drone_pose is None:
                    is_healthy = False
                    rospy.logwarn_throttle(5.0, "No drone pose available")

            if self.mavros_state is not None and not self.mavros_state.connected:
                is_healthy = False
                rospy.logwarn_throttle(5.0, "MAVROS disconnected")

            health_msg.is_healthy = is_healthy
            health_msg.detection_count = len(self.tracked_targets)
            health_msg.error_count = self.error_count
            health_msg.fps = 20.0
            health_msg.cpu_usage = psutil.cpu_percent()
            health_msg.memory_usage = psutil.virtual_memory().percent

            self.health_pub.publish(health_msg)

        except Exception as e:
            rospy.logwarn(f"Error publishing health: {e}")


if __name__ == '__main__':
    tracker = TargetTrackingController()
    tracker.run()
