#!/usr/bin/env python3
"""Stream the newest Gazebo camera frame to the ROS container over TCP.

Runs inside the PX4 SITL container, which has gz transport but no ROS.
Frames are halved (stride-2) before sending: YOLO and the portal do not
need 1280x960, and the host link stays at ~9 MB/s instead of ~37.
"""

import argparse
import struct
import sys
import time

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

HEADER = struct.Struct('<4sIqHHBBI')  # magic, seq, stamp_ns, w, h, fmt, reserved, size


def halve(data, width, height, fmt):
    """Downsample RGB/RGBA/L8 by stride 2; returns (bytes, w, h)."""
    if width < 1000:  # already at the target size
        h, w = height, width
        if fmt == 3 and len(data) >= w * h * 3:
            return data[:w * h * 3], w, h
        if fmt == 4 and len(data) >= w * h * 4:
            arr = np.frombuffer(data, dtype=np.uint8)[:w * h * 4]
            return np.ascontiguousarray(
                arr.reshape(h, w, 4)[:, :, :3]).tobytes(), w, h
        if fmt == 1 and len(data) >= w * h:
            arr = np.frombuffer(data, dtype=np.uint8)[:w * h]
            return np.repeat(arr.reshape(h, w), 3, axis=1).tobytes(), w, h
        return None
    if fmt == 3 and len(data) >= width * height * 3:
        arr = np.frombuffer(data, dtype=np.uint8)[:width * height * 3]
        arr = arr.reshape(height, width, 3)[::2, ::2]
        return arr.tobytes(), width // 2, height // 2
    if fmt == 4 and len(data) >= width * height * 4:
        arr = np.frombuffer(data, dtype=np.uint8)[:width * height * 4]
        arr = arr.reshape(height, width, 4)[::2, ::2, :3]
        return np.ascontiguousarray(arr).tobytes(), width // 2, height // 2
    if fmt == 1 and len(data) >= width * height:
        arr = np.frombuffer(data, dtype=np.uint8)[:width * height]
        arr = arr.reshape(height, width)[::2, ::2]
        return np.repeat(arr, 3, axis=1).tobytes(), width // 2, height // 2
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--rate', type=float, default=10.0)
    args = parser.parse_args()

    pending = {}
    seq = [0]

    def on_image(msg):
        frame = halve(msg.data, msg.width, msg.height, msg.pixel_format_type)
        if frame is None:
            return
        seq[0] += 1
        pending['frame'] = (frame[0], frame[1], frame[2],
                            msg.header.stamp.sec * 10**9
                            + msg.header.stamp.nsec)

    node = Node()
    if not node.subscribe(Image, args.topic, on_image):
        print(f'gz_camera_bridge: subscribe failed: {args.topic}', flush=True)
        return 1
    print(f'gz_camera_bridge: listening on {args.topic}', flush=True)

    sock = None
    period = 1.0 / args.rate
    while True:
        if sock is None:
            try:
                import socket
                sock = socket.create_connection((args.host, args.port),
                                                timeout=3)
                sock.settimeout(3)
                print(f'gz_camera_bridge: connected {args.host}:{args.port}',
                      flush=True)
            except OSError:
                sock = None
                time.sleep(1)
                continue
        frame = pending.pop('frame', None)
        if frame is None:
            time.sleep(min(period, 0.05))
            continue
        data, w, h, stamp = frame
        packet = HEADER.pack(b'GZC1', seq[0] & 0xFFFFFFFF, stamp,
                             w, h, 3, 0, len(data)) + data
        try:
            sock.sendall(packet)
        except OSError:
            try:
                sock.close()
            except OSError:
                pass
            sock = None
            continue
        time.sleep(period)


if __name__ == '__main__':
    sys.exit(main())
