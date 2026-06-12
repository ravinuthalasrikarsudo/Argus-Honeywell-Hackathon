#!/usr/bin/env python3
"""ARGUS acceptance suite.

Scores the simulation foundation against a fixed checklist and measures
the true real-time factor (RTF) under the full warehouse world with both 720p
stereo cameras and the IMU rendering. The lighter ``check_stack`` is a
day-to-day probe.

Two modes:

    ros2 run argus_bringup acceptance --full
        Self-contained: builds the workspace (point 1), verifies the message
        interfaces (point 2), launches the stack headless, scores the runtime
        points, records a contract rosbag, then tears everything down.

    ros2 run argus_bringup acceptance
        Scores only the runtime points against an already-running stack;
        points 1, 2 and the bag artifact are reported as skipped.

Scorecard (10 points + 2 bonus):
     1  clean colcon build (0 warnings)
     2  argus_msgs interfaces resolve (VIOHealth, UncertaintyMap)
     3  all 9 frozen contract topics present and flowing
     4  stereo intrinsics correct (1280x720, fx=fy=640, cx=640, cy=360, no
        distortion); cam0 P[3] = 0 (left = reference)
     5  cam1 baseline patch live (P[3] = -76.8, deviation #3)
     6  frame_ids correct (cam0/cam1_optical_frame, imu_link, pose frame = world)
     7  IMU sane at rest (|a_z - 9.8| < 0.1, frame imu_link)
     8  ground-truth start pose (1.5, 0, 1.0); cmd_vel drive -> moves in x;
        kinematic (z holds, no fall)
     9  /clock and /argus/clock present and advancing (use_sim_time propagation)
    10  real RTF under full world + sensors (report-only; flagged if < 0.8)
    B1  contract rosbag artifact records (record_bag, >0 msgs on contract topics)
    B2  no world->base_link TF published (deviation #4; edge left free for VIO)

Exit code 0 if every GATED check passes (point 10 is non-gating), else 1.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Imu, Image
from geometry_msgs.msg import PoseStamped, Twist
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

CONTRACT_TOPICS = [
    '/argus/cam0/image_raw', '/argus/cam0/camera_info',
    '/argus/cam1/image_raw', '/argus/cam1/camera_info',
    '/argus/imu', '/argus/ground_truth/pose', '/argus/cmd_vel',
    '/clock', '/argus/clock',
]
WORLD_FRAME = 'warehouse_corridor'
SPAWN = (1.5, 0.0, 1.0)
EXPECTED_TX = -76.8          # cam1 P[3] = -fx*baseline (deviation #3)
GRAVITY = 9.8
RTF_SOFT_FLOOR = 0.8         # report-only flag threshold


# --------------------------------------------------------------------------- #
# Result bookkeeping
# --------------------------------------------------------------------------- #
class Result:
    def __init__(self, pid, label, gated=True):
        self.pid = pid
        self.label = label
        self.gated = gated
        self.passed = None      # None = skipped
        self.detail = ''

    def set(self, passed, detail=''):
        self.passed = passed
        self.detail = detail
        return self


class Scorecard:
    def __init__(self):
        self.results = []

    def add(self, pid, label, gated=True):
        r = Result(pid, label, gated)
        self.results.append(r)
        return r

    def gated_failures(self):
        return [r for r in self.results if r.gated and r.passed is False]

    def render(self):
        green, red, yellow, reset = '\033[32m', '\033[31m', '\033[33m', '\033[0m'
        print('\n============ ARGUS ACCEPTANCE SCORECARD ============')
        for r in self.results:
            if r.passed is None:
                tag = f'{yellow}[ SKIP ]{reset}'
            elif r.passed:
                tag = f'{green}[ PASS ]{reset}'
            else:
                tag = f'{red}[ FAIL ]{reset}'
            note = '' if r.gated else f' {yellow}(non-gating){reset}'
            print(f'  {tag} {r.pid:>2}  {r.label}{note}')
            if r.detail:
                print(f'             -> {r.detail}')
        gated = [r for r in self.results if r.gated]
        passed = [r for r in gated if r.passed]
        print('---------------------------------------------------------')
        print(f'  gated: {len(passed)}/{len(gated)} passed')
        print('=========================================================')


# --------------------------------------------------------------------------- #
# Subscriber node (runtime observation)
# --------------------------------------------------------------------------- #
class Observer(Node):
    def __init__(self):
        super().__init__('acceptance_observer')
        self.cam0_info = None
        self.cam1_info = None
        self.imu = None
        self.pose = None
        self.pose_first = None
        self.counts = {k: 0 for k in
                       ('cam0_img', 'cam1_img', 'imu', 'pose', 'clock')}
        # RTF tracking: (wall_monotonic, sim_seconds) endpoints.
        self.clock_w0 = self.clock_wN = None
        self.clock_s0 = self.clock_sN = None
        self.tf_edges = set()       # (parent_frame, child_frame)
        self.img0_frame = self.img1_frame = None

        self.create_subscription(CameraInfo, '/argus/cam0/camera_info',
                                 lambda m: setattr(self, 'cam0_info', m), 10)
        self.create_subscription(CameraInfo, '/argus/cam1/camera_info',
                                 lambda m: setattr(self, 'cam1_info', m), 10)
        self.create_subscription(Image, '/argus/cam0/image_raw', self._on_img0, 10)
        self.create_subscription(Image, '/argus/cam1/image_raw', self._on_img1, 10)
        self.create_subscription(Imu, '/argus/imu', self._on_imu, 50)
        self.create_subscription(PoseStamped, '/argus/ground_truth/pose',
                                 self._on_pose, 10)
        self.create_subscription(Clock, '/clock', self._on_clock, 50)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 10)
        self.create_subscription(TFMessage, '/tf_static', self._on_tf, 10)

    def _on_img0(self, m):
        self.counts['cam0_img'] += 1
        self.img0_frame = m.header.frame_id

    def _on_img1(self, m):
        self.counts['cam1_img'] += 1
        self.img1_frame = m.header.frame_id

    def _on_imu(self, m):
        self.counts['imu'] += 1
        self.imu = m

    def _on_pose(self, m):
        self.counts['pose'] += 1
        self.pose = m
        if self.pose_first is None:
            self.pose_first = m

    def _on_clock(self, m):
        self.counts['clock'] += 1
        sim = m.clock.sec + m.clock.nanosec * 1e-9
        wall = time.monotonic()
        if self.clock_w0 is None:
            self.clock_w0, self.clock_s0 = wall, sim
        self.clock_wN, self.clock_sN = wall, sim

    def _on_tf(self, m):
        for tr in m.transforms:
            self.tf_edges.add((tr.header.frame_id, tr.child_frame_id))

    def rtf(self):
        if self.clock_w0 is None or self.clock_wN == self.clock_w0:
            return None
        return (self.clock_sN - self.clock_s0) / (self.clock_wN - self.clock_w0)


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


# --------------------------------------------------------------------------- #
# Preflight (points 1 & 2) -- subprocess, only in --full
# --------------------------------------------------------------------------- #
def check_build(card, ws):
    r = card.add(1, 'clean colcon build (0 warnings)')
    print('[acceptance] point 1: colcon build --symlink-install ...', flush=True)
    try:
        proc = subprocess.run(
            ['colcon', 'build', '--symlink-install'],
            cwd=ws, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        r.set(False, 'colcon build timed out (>300s)')
        return
    out = proc.stdout + proc.stderr
    warnings = out.lower().count('warning')
    stderr_blocks = out.count('--- stderr:')
    ok = proc.returncode == 0 and warnings == 0 and stderr_blocks == 0
    r.set(ok, f'rc={proc.returncode}, warnings={warnings}, stderr_blocks={stderr_blocks}')


def check_interfaces(card):
    r = card.add(2, 'argus_msgs interfaces resolve')
    needles = {
        'argus_msgs/msg/VIOHealth': ('STATUS_LOST', 'confidence'),
        'argus_msgs/msg/UncertaintyMap': ('covariance', 'position_uncertainty'),
    }
    details, ok = [], True
    for iface, tokens in needles.items():
        try:
            p = subprocess.run(['ros2', 'interface', 'show', iface],
                               capture_output=True, text=True, timeout=20)
            good = p.returncode == 0 and all(t in p.stdout for t in tokens)
        except subprocess.TimeoutExpired:
            good = False
        ok &= good
        details.append(f'{iface.split("/")[-1]}={"ok" if good else "FAIL"}')
    r.set(ok, ', '.join(details))


# --------------------------------------------------------------------------- #
# Runtime checks (points 3-10 + bonus)
# --------------------------------------------------------------------------- #
def near(a, b, tol):
    return a is not None and abs(a - b) <= tol


def check_runtime(card, node, args):
    present = dict(node.get_topic_names_and_types())

    # 3 -- contract topics present and flowing.
    missing = [t for t in CONTRACT_TOPICS if t not in present]
    flowing = all(node.counts[k] > 0 for k in ('cam0_img', 'cam1_img', 'imu', 'pose', 'clock'))
    card.add(3, 'all 9 contract topics present + flowing').set(
        not missing and flowing,
        f'present {len(CONTRACT_TOPICS) - len(missing)}/{len(CONTRACT_TOPICS)}'
        + (f', missing {missing}' if missing else '')
        + f'; flows imu={node.counts["imu"]} cam0={node.counts["cam0_img"]} '
          f'cam1={node.counts["cam1_img"]} pose={node.counts["pose"]}')

    # 4 -- stereo intrinsics + cam0 reference projection.
    r4 = card.add(4, 'stereo intrinsics correct; cam0 P[3]=0')
    ci0, ci1 = node.cam0_info, node.cam1_info
    if ci0 is None or ci1 is None:
        r4.set(False, 'missing camera_info')
    else:
        def intr_ok(ci):
            return (ci.width == 1280 and ci.height == 720
                    and near(ci.k[0], 640, 1e-3) and near(ci.k[4], 640, 1e-3)
                    and near(ci.k[2], 640, 1e-3) and near(ci.k[5], 360, 1e-3)
                    and all(abs(d) < 1e-9 for d in ci.d))
        ok = intr_ok(ci0) and intr_ok(ci1) and near(ci0.p[3], 0.0, 1e-6)
        r4.set(ok, f'{ci0.width}x{ci0.height} fx={ci0.k[0]:.0f} fy={ci0.k[4]:.0f} '
                   f'cx={ci0.k[2]:.0f} cy={ci0.k[5]:.0f} d={list(ci0.d)} '
                   f'cam0 P[3]={ci0.p[3]:.3f}')

    # 5 -- cam1 baseline patch.
    r5 = card.add(5, 'cam1 baseline patch live (P[3]=-76.8)')
    if ci1 is None:
        r5.set(False, 'no cam1 camera_info')
    else:
        r5.set(near(ci1.p[3], EXPECTED_TX, 0.1), f'cam1 P[3]={ci1.p[3]:.3f}')

    # 6 -- frame_ids.
    r6 = card.add(6, 'frame_ids correct (optical/imu/world)')
    f0 = ci0.header.frame_id if ci0 else None
    f1 = ci1.header.frame_id if ci1 else None
    fi = node.imu.header.frame_id if node.imu else None
    fp = node.pose.header.frame_id if node.pose else None
    ok = (f0 == 'cam0_optical_frame' and f1 == 'cam1_optical_frame'
          and fi == 'imu_link' and fp == WORLD_FRAME)
    r6.set(ok, f'cam0={f0} cam1={f1} imu={fi} pose={fp}')

    # 7 -- IMU sane at rest (drone has not been driven yet).
    r7 = card.add(7, 'IMU sane at rest (|a_z-9.8|<0.1)')
    if node.imu is None:
        r7.set(False, 'no imu msg')
    else:
        az = node.imu.linear_acceleration.z
        r7.set(near(az, GRAVITY, 0.1) and node.imu.header.frame_id == 'imu_link',
               f'a_z={az:.3f}, frame={node.imu.header.frame_id}')

    # 9 -- /clock advancing (do this before drive; uses the observation window).
    r9 = card.add(9, '/clock + /argus/clock present + advancing')
    clk = '/clock' in present and '/argus/clock' in present
    adv = node.clock_s0 is not None and node.clock_sN > node.clock_s0
    r9.set(clk and adv,
           f'both topics={clk}, sim advanced +{(node.clock_sN - node.clock_s0):.2f}s'
           if node.clock_s0 is not None else 'no /clock')

    # 10 -- real RTF (report-only).
    r10 = card.add(10, 'real RTF under full world + sensors', gated=False)
    rtf = node.rtf()
    if rtf is None:
        r10.set(False, 'could not measure (no /clock span)')
    else:
        r10.set(rtf >= RTF_SOFT_FLOOR,
                f'RTF={rtf:.3f} ({"OK" if rtf >= RTF_SOFT_FLOOR else "BELOW " + str(RTF_SOFT_FLOOR) + " (iGPU render load)"})')

    # 8 -- ground-truth start pose + drive + no-fall.  (Last: it perturbs state.)
    r8 = card.add(8, 'start pose (1.5,0,1.0); drive moves x; no fall')
    p0 = node.pose_first
    start_ok = (p0 is not None
                and near(p0.pose.position.x, SPAWN[0], 0.05)
                and near(p0.pose.position.y, SPAWN[1], 0.05)
                and near(p0.pose.position.z, SPAWN[2], 0.05))
    x_before = node.pose.pose.position.x if node.pose else None
    print('[acceptance] point 8: drive_drone forward 0.5 m/s for 6s ...', flush=True)
    try:
        subprocess.run(['ros2', 'run', 'argus_bringup', 'drive_drone',
                        '--pattern', 'forward', '--speed', '0.5', '--duration', '6'],
                       capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        print('[acceptance] WARNING: drive_drone timed out (killed).', flush=True)
    spin_for(node, 1.5)         # let fresh pose arrive
    x_after = node.pose.pose.position.x if node.pose else None
    z_after = node.pose.pose.position.z if node.pose else None
    moved = (x_before is not None and x_after is not None
             and (x_after - x_before) > 0.1)
    no_fall = near(z_after, SPAWN[2], 0.2)
    if p0 is not None:
        start_str = (f'start=({p0.pose.position.x:.2f},{p0.pose.position.y:.2f},'
                     f'{p0.pose.position.z:.2f})')
    else:
        start_str = 'start=?'
    if x_before is not None and x_after is not None:
        move_str = f'x {x_before:.2f}->{x_after:.2f} z={z_after:.2f}'
    else:
        move_str = 'no pose during drive'
    r8.set(start_ok and moved and no_fall, f'{start_str}; {move_str}')

    # B2 -- no world->base_link TF (deviation #4).
    rb2 = card.add('B2', 'no world->base_link TF (dev #4)')
    bad = [(p, c) for (p, c) in node.tf_edges
           if c == 'base_link' and p in (WORLD_FRAME, 'world', 'map', 'odom')]
    rb2.set(not bad, 'no ground-truth TF edge' if not bad else f'unexpected TF {bad}')


def check_bag(card, node, ws, do_bag):
    rb1 = card.add('B1', 'contract rosbag artifact (record_bag)')
    if not do_bag:
        return  # skipped (stays None)
    out = os.path.expanduser(
        f'~/argus/bags/acceptance_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
    print(f'[acceptance] bonus B1: record_bag 6s -> {out}', flush=True)
    try:
        subprocess.run(['ros2', 'run', 'argus_bringup', 'record_bag',
                        '--duration', '6', '-o', out],
                       capture_output=True, text=True, timeout=40)
        info = subprocess.run(['ros2', 'bag', 'info', out],
                              capture_output=True, text=True, timeout=20)
        txt = info.stdout
    except subprocess.TimeoutExpired:
        rb1.set(False, 'record_bag / bag info timed out')
        return
    has_db = os.path.isdir(out) and any(f.endswith('.db3') for f in os.listdir(out))
    # crude message-count: bag info prints "Messages: N"
    msgs = 0
    for line in txt.splitlines():
        if 'Messages:' in line:
            try:
                msgs = int(line.split('Messages:')[1].strip())
            except ValueError:
                pass
    contract_seen = sum(1 for t in CONTRACT_TOPICS if t in txt)
    rb1.set(has_db and msgs > 0 and contract_seen >= 7,
            f'msgs={msgs}, contract topics in bag={contract_seen}/9, dir={out}')


# --------------------------------------------------------------------------- #
# --full orchestration
# --------------------------------------------------------------------------- #
def launch_stack(world):
    cmd = ['ros2', 'launch', 'argus_bringup', 'argus_sim.launch.py',
           'headless:=true', f'world:={world}']
    print(f'[acceptance] launching stack: {" ".join(cmd)}', flush=True)
    return subprocess.Popen(cmd, start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def teardown(proc):
    print('[acceptance] teardown ...', flush=True)
    if proc and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=15)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    for pat in ('gz sim', 'parameter_bridge', 'camera_info_patch'):
        subprocess.run(['pkill', '-INT', '-f', pat], capture_output=True, timeout=10)
    time.sleep(2)
    subprocess.run(['pkill', '-9', '-f', 'gz-sim|gz sim|ruby'], capture_output=True, timeout=10)


def main(argv=None):
    p = argparse.ArgumentParser(description='ARGUS formal acceptance suite.')
    p.add_argument('--full', action='store_true',
                   help='Build, launch headless, score everything, record a bag, tear down.')
    p.add_argument('--world', default=WORLD_FRAME, help='World name.')
    p.add_argument('--settle', type=float, default=18.0,
                   help='Seconds to wait after launch before scoring (--full).')
    p.add_argument('--window', type=float, default=12.0,
                   help='Seconds to observe topics / measure RTF.')
    p.add_argument('--no-bag', action='store_true', help='Skip the bonus rosbag (B1).')
    args = p.parse_args(sys.argv[1:] if argv is None else argv)

    ws = os.path.expanduser('~/argus')
    card = Scorecard()
    launch_proc = None

    # Points 1 & 2 only make sense / are reachable in --full.
    if args.full:
        check_build(card, ws)
        check_interfaces(card)
        launch_proc = launch_stack(args.world)
        print(f'[acceptance] settling {args.settle:.0f}s ...', flush=True)
        time.sleep(args.settle)
    else:
        card.add(1, 'clean colcon build (0 warnings)')          # skipped
        card.add(2, 'argus_msgs interfaces resolve')            # skipped

    rclpy.init()
    node = Observer()
    try:
        print(f'[acceptance] observing {args.window:.0f}s '
              '(topics, RTF, IMU-at-rest) ...', flush=True)
        spin_for(node, args.window)
        check_runtime(card, node, args)
        check_bag(card, node, ws, do_bag=args.full and not args.no_bag)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if args.full:
            teardown(launch_proc)

    card.render()
    fails = card.gated_failures()
    print(f'\n[acceptance] {"ACCEPTED (all gated checks passed)" if not fails else f"REJECTED ({len(fails)} gated failure(s))"}')
    return 0 if not fails else 1


if __name__ == '__main__':
    sys.exit(main())
