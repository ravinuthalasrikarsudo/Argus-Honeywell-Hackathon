#!/usr/bin/env python3
"""Audit: BEST_EFFORT Image subscriber (mimics rqt_image_view) — counts overlay
frames received, to confirm the QoS fix actually delivers over Cyclone/loopback."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # BEST_EFFORT, KEEP_LAST 5
from sensor_msgs.msg import Image

rclpy.init()
n = Node("audit_be_sub")
cnt = {"c": 0}
n.create_subscription(Image, "/argus/superpoint/overlay",
                      lambda m: cnt.__setitem__("c", cnt["c"] + 1),
                      qos_profile_sensor_data)
t = time.time()
while time.time() - t < 6.0:
    rclpy.spin_once(n, timeout_sec=0.2)
print(f"BEST_EFFORT sub received {cnt['c']} overlay frames in 6s")
n.destroy_node()
rclpy.shutdown()
