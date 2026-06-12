#!/usr/bin/env python3
"""ARGUS check_stack helper.

Quick interactive health probe for a running stack: confirms the frozen
``/argus/*`` contract topics are present and flowing, that the cam1 baseline
patch is live (right ``CameraInfo.P[3] = -76.8``, deviation #3), that the left
camera is the unshifted reference (``P[3] = 0``), that ground truth and the sim
clock are alive, and prints the observed wall rates.

This is the lightweight "is the stack up?" check. It is NOT the formal
acceptance suite (which scores the 10 points and measures true RTF under the
full world). Rates here are WALL-clock and informational: under sim RTF < 1
(known iGPU load) they read low without anything being wrong.

Run (with the stack already up)::

    ros2 run argus_bringup check_stack --window 5

Exit code 0 if every gated check passes, 1 otherwise.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Imu, Image
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock

EXPECTED_TOPICS = [
    '/argus/cam0/image_raw', '/argus/cam0/camera_info',
    '/argus/cam1/image_raw', '/argus/cam1/camera_info',
    '/argus/imu', '/argus/ground_truth/pose', '/argus/cmd_vel',
    '/clock', '/argus/clock',
]
EXPECTED_TX = -76.8   # cam1 P[3] = -fx*baseline (deviation #3)
TX_TOL = 0.1


class CheckStack(Node):
    def __init__(self):
        super().__init__('check_stack')
        self.counts = {k: 0 for k in
                       ('cam0_img', 'cam1_img', 'imu', 'pose', 'clock')}
        self.cam0_p = None
        self.cam1_p = None
        self.pose = None
        self.clock_first = None
        self.clock_last = None

        self.create_subscription(CameraInfo, '/argus/cam0/camera_info',
                                 lambda m: setattr(self, 'cam0_p', list(m.p)), 10)
        self.create_subscription(CameraInfo, '/argus/cam1/camera_info',
                                 lambda m: setattr(self, 'cam1_p', list(m.p)), 10)
        self.create_subscription(Image, '/argus/cam0/image_raw',
                                 lambda m: self._bump('cam0_img'), 10)
        self.create_subscription(Image, '/argus/cam1/image_raw',
                                 lambda m: self._bump('cam1_img'), 10)
        self.create_subscription(Imu, '/argus/imu',
                                 lambda m: self._bump('imu'), 10)
        self.create_subscription(PoseStamped, '/argus/ground_truth/pose',
                                 self._on_pose, 10)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)

    def _bump(self, key):
        self.counts[key] += 1

    def _on_pose(self, m):
        self.counts['pose'] += 1
        self.pose = m.pose.position

    def _on_clock(self, m):
        self.counts['clock'] += 1
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if self.clock_first is None:
            self.clock_first = t
        self.clock_last = t


def _tag(ok):
    return '\033[32m[ PASS ]\033[0m' if ok else '\033[31m[ FAIL ]\033[0m'


def main(argv=None):
    parser = argparse.ArgumentParser(description='Probe a running ARGUS stack.')
    parser.add_argument('--window', type=float, default=5.0,
                        help='Seconds to sample (wall clock, default 5).')
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    rclpy.init()
    node = CheckStack()
    print(f'[check_stack] sampling for {args.window:.0f}s ...', flush=True)

    end = time.monotonic() + args.window
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    present = dict(node.get_topic_names_and_types())
    win = args.window
    ok_all = True

    print('\n=== ARGUS stack health ===')

    # Gated: contract topics present.
    missing = [t for t in EXPECTED_TOPICS if t not in present]
    ok = not missing
    ok_all &= ok
    print(f'{_tag(ok)} contract topics present '
          f'({len(EXPECTED_TOPICS) - len(missing)}/{len(EXPECTED_TOPICS)})'
          + (f'  missing: {missing}' if missing else ''))

    # Gated: cam0 reference projection P[3] == 0.
    ok = node.cam0_p is not None and abs(node.cam0_p[3]) < 1e-6
    ok_all &= ok
    val = 'no msg' if node.cam0_p is None else f'{node.cam0_p[3]:.3f}'
    print(f'{_tag(ok)} cam0 P[3] = {val}  (expected 0, left=reference)')

    # Gated: cam1 baseline patch live, P[3] == -76.8.
    ok = node.cam1_p is not None and abs(node.cam1_p[3] - EXPECTED_TX) < TX_TOL
    ok_all &= ok
    val = 'no msg' if node.cam1_p is None else f'{node.cam1_p[3]:.3f}'
    print(f'{_tag(ok)} cam1 P[3] = {val}  (expected {EXPECTED_TX}, patch live)')

    # Gated: ground truth pose flowing.
    ok = node.pose is not None
    ok_all &= ok
    pstr = ('no msg' if node.pose is None
            else f'({node.pose.x:.2f}, {node.pose.y:.2f}, {node.pose.z:.2f})')
    print(f'{_tag(ok)} ground_truth pose = {pstr}')

    # Gated: sim clock advancing.
    advancing = (node.clock_last is not None and node.clock_first is not None
                 and node.clock_last > node.clock_first)
    ok_all &= advancing
    span = (0.0 if node.clock_first is None
            else node.clock_last - node.clock_first)
    print(f'{_tag(advancing)} /clock advancing (+{span:.2f}s sim over the window)')

    # Gated: sensor streams non-empty.
    for key, label in (('imu', 'imu'), ('cam0_img', 'cam0 image'),
                       ('cam1_img', 'cam1 image')):
        n = node.counts[key]
        ok = n > 0
        ok_all &= ok
        print(f'{_tag(ok)} {label}: {n} msgs  (~{n / win:.1f} Hz wall, informational)')

    print('\n[check_stack] NOTE: wall rates read low under sim RTF<1 (iGPU).')
    print(f'[check_stack] {"ALL CHECKS PASSED" if ok_all else "SOME CHECKS FAILED"}')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
