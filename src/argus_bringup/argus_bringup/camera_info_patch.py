#!/usr/bin/env python3
"""ARGUS cam1 (right) CameraInfo baseline patch.

Frozen-contract deviation #3: Gazebo emits two independent cameras and does NOT
encode the stereo baseline, so the bridged CameraInfo for the right camera has
projection P[3] = 0. ROS stereo expects the right camera's P[3] = -fx * baseline
(the Tx term). This node subscribes the bridged-but-raw right CameraInfo, sets
P[3], and republishes on the sibling camera_info topic that downstream stereo
nodes expect. The left camera (reference) needs no patch (P[3] = 0).

Defaults match the frozen contract: fx = 640, baseline = 0.12 m -> P[3] = -76.8.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPatch(Node):
    def __init__(self):
        super().__init__('camera_info_patch')

        # Contract defaults; overridable so intrinsics/baseline stay data-driven.
        self.declare_parameter('fx', 640.0)
        self.declare_parameter('baseline', 0.12)
        self.declare_parameter('input_topic', '/argus/cam1/camera_info_gz')
        self.declare_parameter('output_topic', '/argus/cam1/camera_info')

        fx = self.get_parameter('fx').value
        baseline = self.get_parameter('baseline').value
        self.tx = -fx * baseline  # = -76.8 with the contract defaults

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # Match the bridge's default QoS (reliable, depth 10).
        self.pub = self.create_publisher(CameraInfo, out_topic, 10)
        self.sub = self.create_subscription(
            CameraInfo, in_topic, self.on_camera_info, 10)

        self.get_logger().info(
            f'camera_info_patch: {in_topic} -> {out_topic}, '
            f'P[3] = -fx*baseline = {self.tx:.4f}')

    def on_camera_info(self, msg):
        # Right-camera projection: set the Tx term, pass everything else through.
        p = list(msg.p)
        p[3] = self.tx
        msg.p = p
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
