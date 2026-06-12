#!/usr/bin/env python3
"""Grab first /argus/cam0/image_raw frame: print mean/min/max/std (flat-vs-textured)
and save a PPM (no PIL dep). std>>0 => real geometry, not a flat fill."""
import sys, time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/cam0_live.ppm"
rclpy.init()
node = rclpy.create_node("cam_grab")
got = []


def cb(m):
    if got:
        return
    a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, -1)
    got.append((a, m.encoding))


qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(Image, "/argus/cam0/image_raw", cb, qos)
t0 = time.monotonic()
while rclpy.ok() and not got and time.monotonic() - t0 < 20:
    rclpy.spin_once(node, timeout_sec=0.5)

if got:
    a, enc = got[0]
    h, w, c = a.shape
    print(f"frame {w}x{h} enc={enc} mean={a.mean():.1f} min={a.min()} "
          f"max={a.max()} std={a.std():.1f}")
    rgb = a[:, :, :3]
    if enc.startswith("bgr"):
        rgb = rgb[:, :, ::-1]
    with open(OUT, "wb") as f:
        f.write(f"P6\n{w} {h}\n255\n".encode())
        f.write(np.ascontiguousarray(rgb).tobytes())
    print(f"saved {OUT}")
else:
    print("no cam0 frame received")
node.destroy_node()
rclpy.shutdown()
