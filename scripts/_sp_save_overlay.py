#!/usr/bin/env python3
"""Grab the first N /argus/superpoint/overlay frames and save them as PNGs.
Run with the SuperPoint venv (has cv2) + ROS sourced. Used to capture the
keypoint-overlay evidence for Scenario-B (low-texture) confirmation."""
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

OUT = sys.argv[1] if len(sys.argv) > 1 else '/home/vittal/argus/data/eval/superpoint'
N = int(sys.argv[2]) if len(sys.argv) > 2 else 6


class Saver(Node):
    def __init__(self):
        super().__init__('sp_overlay_saver')
        import os
        os.makedirs(OUT, exist_ok=True)
        self.i = 0
        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/argus/superpoint/overlay', self._cb, q)
        self.get_logger().info(f'saving up to {N} overlay frames -> {OUT}')

    def _cb(self, msg):
        img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
        # save every ~15th frame to spread across the flight
        if self.i % 15 == 0:
            p = f'{OUT}/overlay_{self.i:04d}.png'
            cv2.imwrite(p, img)
            self.get_logger().info(f'saved {p}')
        self.i += 1


def main():
    rclpy.init()
    n = Saver()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
