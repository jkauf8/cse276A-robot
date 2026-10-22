#!/usr/bin/env python3
"""Run on the robot while the camera node is running. Press Enter to save the current frame; type q + Enter to quit.

    python3 capture_frames.py [out_dir]      # default ~/calib_frames
Watch /camera/image_raw in Foxglove to frame the checkerboard, then press Enter.
"""
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class Grabber(Node):
    def __init__(self):
        super().__init__('calib_grabber')
        self.latest = None
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        self.latest = bytes(msg.data)


def main():
    out = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/calib_frames')
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    node = Grabber()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    n = len([f for f in os.listdir(out) if f.endswith('.jpg')])
    print(f'Saving to {out} (already {n} frames). Enter = save frame, q = quit.')
    while True:
        if input().strip().lower() == 'q':
            break
        if node.latest is None:
            print('No frame received yet; is the camera node running?')
            continue
        path = os.path.join(out, f'frame_{n:03d}.jpg')
        with open(path, 'wb') as f:
            f.write(node.latest)
        n += 1
        print(f'saved {path} ({n} total)')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
