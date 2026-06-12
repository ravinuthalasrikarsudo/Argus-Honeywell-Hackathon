#!/usr/bin/env python3
"""ARGUS :: fly_circuit.py — fly CCW lap(s) of the tunnel_circuit stadium.

Ground-truth-feedback path follower for the Scenario E 200 m drift gate. The
drone holds the tunnel centreline at constant forward speed; the end-caps are
flown as feed-forward arcs (wz = v/R while translating), so the estimator sees
yaw WITH parallax at all times — the regime VINS-Fusion handles well. There are
no stops, no reversals, no in-place turns anywhere in the profile.

GT feedback steers the VEHICLE only; the VIO under test never sees GT.

Geometry must match generate_tunnel_circuit.py: straights y=0 / y=2R for
x in [0, L]; semicircular caps radius R about (0, R) and (L, R); CCW.

--excite (6 s smooth vertical-sinusoid preamble) is a DOCUMENTED NEGATIVE
(day-7 log): even C1-smooth vertical velocity zero-crossings during the init
window corrupt the VINS gravity init (first pose ~12 km out, runaway tilt).
Kept behind the flag for reproducibility of the experiment; default is the
corridor-proven plain step-start.

Run with the ROS env sourced (rclpy + system python, NOT the eval venv):
  python3 scripts/fly_circuit.py --laps 1 --speed 0.8
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

L = 70.0
R = 10.0
PERIM = 2 * L + 2 * math.pi * R     # 202.83 m


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def path_state(x: float, y: float):
    """(s, theta_tangent, e_left, on_arc) for the CCW stadium at world (x, y)."""
    if x > L:                                   # right end-cap, centre (L, R)
        phi = math.atan2(y - R, x - L)
        s = L + R * wrap_phase(phi + math.pi / 2)
        return s, phi + math.pi / 2, R - math.hypot(x - L, y - R), True
    if x < 0.0:                                 # left end-cap, centre (0, R)
        phi = math.atan2(y - R, x)
        s = 2 * L + math.pi * R + R * wrap_phase(phi - math.pi / 2)
        return s, phi + math.pi / 2, R - math.hypot(x, y - R), True
    if y < R:                                   # straight A (heading +x)
        return x, 0.0, y, False
    return L + math.pi * R + (L - x), math.pi, 2 * R - y, False   # straight B


def wrap_phase(a: float) -> float:
    """Angle wrapped to [0, 2*pi) — arc progress is always forward (CCW)."""
    while a < 0.0:
        a += 2 * math.pi
    while a >= 2 * math.pi:
        a -= 2 * math.pi
    return a


class CircuitFlyer(Node):
    def __init__(self, a):
        super().__init__('circuit_flyer')
        self.a = a
        self.pub = self.create_publisher(Twist, a.topic, 10)
        self.sub = self.create_subscription(PoseStamped, a.gt_topic, self._gt, 10)
        self.pose = None
        self.yaw = None
        self.t0 = None                  # wall time of first command
        self.s_prev = None
        self.dist = 0.0                 # unwrapped centreline progress (m)
        self.goal = a.laps * PERIM + a.overrun
        self.done = False
        self._log_t = 0.0
        self.timer = self.create_timer(1.0 / a.rate, self._tick)

    def _gt(self, msg):
        p = msg.pose.position
        self.pose = (p.x, p.y, p.z)
        self.yaw = yaw_of(msg.pose.orientation)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self):
        if self.pose is None:
            return
        if self.t0 is None:
            self.t0 = self._now()
            self.get_logger().info(
                f"circuit: {self.a.laps} lap(s) x {PERIM:.1f} m + {self.a.overrun} m overrun"
                f" at {self.a.speed} m/s, excite={'on' if self.a.excite else 'off'}")
        t = self._now() - self.t0
        x, y, z = self.pose
        s, th_t, e, on_arc = path_state(x, y)

        # unwrapped progress along the lap
        if self.s_prev is not None:
            ds = s - self.s_prev
            if ds < -PERIM / 2:
                ds += PERIM          # lap wrap 202.8 -> 0
            if -2.0 < ds < 5.0:      # reject GT glitches
                self.dist += max(ds, 0.0)
        self.s_prev = s

        cmd = Twist()
        v = self.a.speed

        # ---- speed profile: excite preamble -> ramp -> cruise -> brake ----
        t_ex = 6.0 if self.a.excite else 0.0
        if t < t_ex:
            # smooth slow-forward + 3 periods of vertical sinusoid (ends at 0)
            vx = 0.3 * v * math.sin(math.pi / 2 * min(t / 3.0, 1.0))
            cmd.linear.z = 0.22 * math.sin(2 * math.pi * 0.5 * t)
        else:
            # HARD step to cruise speed (day-6/day-7: ramped starts starve the
            # VINS gravity init -> Z-ramp tilt; step-starts init reliably).
            tr = t - t_ex
            if self.a.excite:
                ramp = math.sin(math.pi / 2 * min(tr / 4.0, 1.0))
                vx = 0.3 * v + (v - 0.3 * v) * ramp
            else:
                vx = v
            cmd.linear.z = self._clamp(0.8 * (self.a.alt - z), -0.3, 0.3)

        remain = self.goal - self.dist
        if remain <= 0.0 and not self.done:
            self.done = True
            self.get_logger().info(f"circuit: done — {self.dist:.1f} m flown, hovering.")
        if self.done:
            self.pub.publish(Twist())            # hover (VelocityControl holds)
            return
        if remain < 2.0:                          # smooth brake over last 2 m
            vx = min(vx, max(0.15, v * remain / 2.0))

        # ---- steering: tangent heading + lateral correction + arc feed-forward ----
        th_cmd = th_t - self._clamp(self.a.k_e * e, -0.35, 0.35)
        ff = (v / R) if on_arc else 0.0
        cmd.linear.x = vx
        cmd.angular.z = self._clamp(ff + self.a.k_h * wrap(th_cmd - self.yaw), -0.5, 0.5)
        self.pub.publish(cmd)

        if t - self._log_t > 5.0:
            self._log_t = t
            self.get_logger().info(
                f"s={s:6.1f} dist={self.dist:6.1f}/{self.goal:.0f} m  e={e:+.2f} m"
                f"  z={z:.2f}  {'ARC' if on_arc else 'STR'}")

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--laps', type=int, default=1)
    p.add_argument('--speed', type=float, default=0.8, help='cruise speed (m/s)')
    p.add_argument('--alt', type=float, default=1.0, help='hold altitude (m)')
    p.add_argument('--overrun', type=float, default=4.0,
                   help='extra metres past lap end (re-observe start for loop closure)')
    p.add_argument('--excite', action='store_true',
                   help='6 s smooth vertical-sinusoid init preamble')
    p.add_argument('--k-e', type=float, default=0.4, help='lateral gain (rad/m)')
    p.add_argument('--k-h', type=float, default=1.2, help='heading gain (1/s)')
    p.add_argument('--rate', type=float, default=20.0)
    p.add_argument('--topic', default='/argus/cmd_vel')
    p.add_argument('--gt-topic', default='/argus/ground_truth/pose')
    a = p.parse_args()

    rclpy.init()
    node = CircuitFlyer(a)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        # publish a few explicit hover zeros before exiting
        for _ in range(5):
            node.pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
