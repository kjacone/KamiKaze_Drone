#!/usr/bin/env python3
"""
drone_control/scripts/core/gz_camera_bridge.py
Republish Gazebo camera frames as ROS image messages.

Pairs with fleet/gz_camera_bridge.py, which runs beside the simulator and
forwards the newest downsampled frame over TCP (the SITL container has gz
transport but no ROS). This node owns the ROS side: camera/color/image_raw
plus matching camera_info using the pinhole model of the gz camera.
"""

import math
import socket
import struct
import threading

import rospy
from sensor_msgs.msg import CameraInfo, Image

HEADER = struct.Struct('<4sIqHHBBI')
MAGIC = b'GZC1'


class GzCameraBridge:
    def __init__(self):
        rospy.init_node('gz_camera_bridge', anonymous=False)

        self.port = int(rospy.get_param('~port', 9200))
        self.image_topic = rospy.get_param(
            '~image_topic', 'camera/color/image_raw')
        self.info_topic = rospy.get_param(
            '~camera_info_topic', 'camera/color/camera_info')
        self.frame_id = rospy.get_param('~frame_id', 'camera_link')
        self.hfov = float(rospy.get_param('~horizontal_fov', 1.74))

        self.image_pub = rospy.Publisher(
            self.image_topic, Image, queue_size=1)
        self.info_pub = rospy.Publisher(
            self.info_topic, CameraInfo, queue_size=1)

        self._stop = False
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        rospy.loginfo(
            "gz_camera_bridge listening on :%d -> %s",
            self.port, self.image_topic)

    def _camera_info(self, width, height):
        fx = (width / 2.0) / math.tan(self.hfov / 2.0)
        info = CameraInfo()
        info.header.stamp = rospy.Time.now()
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [fx, 0.0, width / 2.0,
                  0.0, fx, height / 2.0,
                  0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [fx, 0.0, width / 2.0, 0.0,
                  0.0, fx, height / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def _publish(self, width, height, payload):
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.height = height
        msg.width = width
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = payload
        self.image_pub.publish(msg)
        self.info_pub.publish(self._camera_info(width, height))

    def _serve(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.port))
        server.listen(1)
        while not rospy.is_shutdown() and not self._stop:
            try:
                conn, addr = server.accept()
            except OSError:
                continue
            rospy.loginfo("gz_camera_bridge: gz side connected from %s",
                          addr[0])
            conn.settimeout(5.0)
            try:
                while not rospy.is_shutdown():
                    header = self._recv_exact(conn, HEADER.size)
                    if header is None:
                        break
                    magic, _seq, _stamp, width, height, _fmt, _r, size = \
                        HEADER.unpack(header)
                    if magic != MAGIC or size == 0 or size > 16 * 1024 * 1024:
                        break
                    payload = self._recv_exact(conn, size)
                    if payload is None:
                        break
                    self._publish(width, height, payload)
            except socket.timeout:
                rospy.logwarn("gz_camera_bridge: stream stalled")
            finally:
                conn.close()
                rospy.logwarn("gz_camera_bridge: gz side disconnected")

    @staticmethod
    def _recv_exact(conn, size):
        chunks = []
        remaining = size
        while remaining > 0:
            chunk = conn.recv(remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b''.join(chunks)


if __name__ == '__main__':
    try:
        GzCameraBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
