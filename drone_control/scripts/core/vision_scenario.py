#!/usr/bin/env python3
"""
drone_control/scripts/core/vision_scenario.py
Vision front end for the SITL target-verification scenario.

The gz world carries two static props (see fleet/gz_scenario.py): a red
car-sized box and a grey pedestrian-sized cylinder. This node projects the
selected prop's real geometry through the live camera pose and publishes
the bbox it occupies in the actual camera image, labelled with the class a
detector would report. Switching to the decoy makes the tracker's vision
check fail, which is the "this is not my target" case.
"""

import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from drone_control.msg import BBox, DetectedObjects, Object

OBJECTS = {
    'target': {
        'centre': np.array([12.0, 0.0, 0.75]),
        'size': np.array([4.6, 1.8, 1.5]),
        'class_id': 2,
        'class_name': 'car',
    },
    'decoy': {
        'centre': np.array([12.0, 5.0, 0.9]),
        'size': np.array([0.7, 0.7, 1.8]),
        'class_id': 0,
        'class_name': 'person',
    },
}


class VisionScenario:
    def __init__(self):
        rospy.init_node('vision_scenario', anonymous=False)

        self.rate_hz = float(rospy.get_param('~rate', 10.0))
        self.camera_offset = np.array(
            rospy.get_param('~camera_offset', [0.12, 0.03, 0.242]))
        # The tracker treats class names in ~target_classes as mission
        # targets; the decoy's class deliberately falls outside it.
        for key in ('target', 'decoy'):
            OBJECTS[key]['class_name'] = rospy.get_param(
                f'~{key}_class', OBJECTS[key]['class_name'])

        self.mode = rospy.get_param('~mode', 'target')
        self.pose = None
        self.camera_info = None

        self.pub = rospy.Publisher(
            'detected_objects', DetectedObjects, queue_size=10)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                         self._on_pose, queue_size=1)
        rospy.Subscriber('camera/color/camera_info', CameraInfo,
                         self._on_camera_info, queue_size=1)
        rospy.Subscriber('scenario_mode', String, self._on_mode,
                         queue_size=1)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self._publish)
        rospy.loginfo("vision_scenario started in '%s' mode", self.mode)

    def _on_pose(self, msg):
        self.pose = msg.pose

    def _on_camera_info(self, msg):
        self.camera_info = msg

    def _on_mode(self, msg):
        mode = msg.data.strip().lower()
        if mode not in OBJECTS and mode != 'none':
            rospy.logwarn("vision_scenario: unknown mode '%s'", mode)
            return
        self.mode = mode
        rospy.loginfo("vision_scenario: mode -> %s", mode)

    @staticmethod
    def _body_to_enu(pose):
        q = pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def _project(self, points_enu):
        """World points -> pixel coords; returns (uv, valid mask)."""
        k = self.camera_info.K
        fx, cx = k[0], k[2]
        fy, cy = k[4], k[5]

        rot = self._body_to_enu(self.pose)
        drone_pos = np.array([self.pose.position.x, self.pose.position.y,
                              self.pose.position.z])
        cam_pos = drone_pos + rot @ self.camera_offset

        body = (points_enu - cam_pos) @ rot  # inverse rotation = transpose
        # body FLU -> optical (right, down, forward)
        optical = np.stack([-body[:, 1], -body[:, 2], body[:, 0]], axis=1)
        valid = optical[:, 2] > 0.2
        uv = np.empty((len(points_enu), 2))
        uv[:, 0] = fx * optical[:, 0] / optical[:, 2] + cx
        uv[:, 1] = fy * optical[:, 1] / optical[:, 2] + cy
        return uv, valid

    def _object_bbox(self, spec):
        half = spec['size'] / 2.0
        centre = spec['centre']
        corners = np.array([
            centre + half * np.array([sx, sy, sz])
            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
        ])
        uv, valid = self._project(corners)
        if not valid.any():
            return None

        width = self.camera_info.width
        height = self.camera_info.height
        uv = uv[valid]
        x1, y1 = uv.min(axis=0)
        x2, y2 = uv.max(axis=0)
        x1, y1 = max(x1, 0.0), max(y1, 0.0)
        x2, y2 = min(x2, width - 1.0), min(y2, height - 1.0)
        if x2 - x1 < 8 or y2 - y1 < 8:
            return None
        # Require the projected centre to be in frame; the tracker works
        # from the bbox bottom edge, so a partially clipped box is fine.
        cxp, cyp = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        if not (0 <= cxp < width and 0 <= cyp < height):
            return None

        bbox = BBox()
        bbox.x = float(x1)
        bbox.y = float(y1)
        bbox.width = float(x2 - x1)
        bbox.height = float(y2 - y1)
        return bbox

    def _publish(self, _event):
        if self.mode == 'none' or self.pose is None or self.camera_info is None:
            return
        spec = OBJECTS[self.mode]
        bbox = self._object_bbox(spec)
        if bbox is None:
            return

        det = DetectedObjects()
        det.header.stamp = rospy.Time.now()
        det.header.frame_id = 'camera_frame'
        obj = Object()
        obj.class_id = spec['class_id']
        obj.class_name = spec['class_name']
        obj.confidence = 0.9
        obj.bbox = bbox
        det.objects.append(obj)
        self.pub.publish(det)


if __name__ == '__main__':
    try:
        VisionScenario()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
