#!/usr/bin/env python3
"""
drone_control/scripts/core/synthetic_camera.py
Synthetic image source for the perception smoke test.

Produces a plain sensor_msgs/Image so the YOLO node has frames to consume.
It is not a perception source; detections still need pixel-to-world projection.
"""

import math

import numpy as np
import rospy
from sensor_msgs.msg import Image


class SyntheticCamera:
    def __init__(self):
        rospy.init_node('synthetic_camera', anonymous=False)

        self.width = int(rospy.get_param('~width', 640))
        self.height = int(rospy.get_param('~height', 480))
        self.rate_hz = float(rospy.get_param('~rate', 10.0))
        self.frame_id = rospy.get_param('~frame_id', 'camera_link')

        self.pub = rospy.Publisher(
            'camera/color/image_raw', Image, queue_size=1
        )
        self.start_time = rospy.Time.now()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self._publish
        )
        rospy.loginfo(
            "Synthetic camera started: %dx%d at %.1f Hz",
            self.width, self.height, self.rate_hz
        )

    def _publish(self, _event):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img[:, :, 2] = 40  # dim blue background

        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        bar_width = 80
        bar_height = 80
        x = int((self.width - bar_width) * (0.5 + 0.5 * math.sin(elapsed)))
        y = self.height // 2 - bar_height // 2
        img[y:y + bar_height, x:x + bar_width] = (0, 200, 0)

        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = self.width * 3
        msg.data = img.tobytes()
        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        SyntheticCamera()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
