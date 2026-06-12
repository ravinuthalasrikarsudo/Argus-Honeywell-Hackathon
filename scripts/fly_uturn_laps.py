#!/usr/bin/env python3
"""ARGUS :: fly_uturn_laps.py

Fly a multi-lap out-and-back path using GROUND-TRUTH-FEEDBACK 180-deg U-turns
instead of backward flight. Backward flight reverses velocity in one step, which
diverges VINS-Fusion at the turnaround (Ceres NaN). A yaw U-turn instead gives
the estimator rotational excitation (which it handles well) and improves the
observability the dead-straight flight lacked.

Robustness: forward legs run until GROUND TRUTH says the drone has travelled
leg_m metres (so each leg is exactly leg_m regardless of sim RTF); U-turns run
until GROUND TRUTH yaw has changed ~180 deg (so legs stay axis-aligned and the
drone never drifts laterally into a wall). Smooth accel/decel ramps (in distance
/ angle) keep the IMU signal physical.

Run with the ROS env sourced (uses rclpy + system python, NOT the eval venv):
  python3 scripts/fly_uturn_laps.py --laps 4 --leg-m 15 --speed 0.6
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


def yaw_of(q) -> float:
    """Yaw about +z (ENU) from a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class UTurnFlyer(Node):
    def __init__(self, a):
        super().__init__('uturn_flyer')
        self.a = a
        self.pub = self.create_publisher(Twist, a.topic, 10)
        self.sub = self.create_subscription(PoseStamped, a.gt_topic, self._gt, 10)
        self.pos = None        # latest GT (x, y)
        self.yaw = None        # latest GT yaw
        self.phase = 'WAIT_GT'
        self.leg_idx = 0       # legs completed
        self.ref_pos = None    # leg start position
        self.prev_yaw = None
        self.yaw_accum = 0.0
        self._dbg = 0
        self.phase_start = self.get_clock().now()
        self.timer = self.create_timer(1.0 / a.rate, self._tick)

    def _gt(self, msg):
        p = msg.pose.position
        self.pos = (p.x, p.y)
        self.yaw = yaw_of(msg.pose.orientation)

    def _elapsed(self, since) -> float:
        return (self.get_clock().now() - since).nanoseconds / 1e9

    def _pub(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.pub.publish(t)

    def _begin(self, ph):
        self.phase = ph
        self.phase_start = self.get_clock().now()
        p = None if self.pos is None else (round(self.pos[0], 2), round(self.pos[1], 2))
        yd = None if self.yaw is None else round(math.degrees(self.yaw), 1)
        self.get_logger().info(
            f'phase -> {ph}  legs={self.leg_idx}/{self.a.laps * 2}  pos={p}  yaw_deg={yd}')

    @staticmethod
    def _ramp(progress, total, ramp):
        """Trapezoidal scale in [floor,1] over a leg/turn of length `total`."""
        if ramp <= 0.0:
            return 1.0
        up = progress / ramp if progress < ramp else 1.0
        left = total - progress
        down = left / ramp if left < ramp else 1.0
        return max(0.15, min(up, down))

    @staticmethod
    def _wrap(angle):
        """Wrap an angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _tick(self):
        if self.pos is None or self.yaw is None:
            return  # wait for first ground-truth message

        ph = self.phase
        if ph == 'WAIT_GT':
            self._begin('HOVER_START')

        elif ph == 'HOVER_START':
            self._pub()
            if self._elapsed(self.phase_start) >= self.a.hover_s:
                self.ref_pos = self.pos
                self._begin('LEG')

        elif ph == 'LEG':
            if self.ref_pos is None:
                self.ref_pos = self.pos
            d = math.dist(self.pos, self.ref_pos)
            if d >= self.a.leg_m:
                self._pub()  # stop
                self.leg_idx += 1
                if self.leg_idx >= self.a.laps * 2:
                    self._begin('HOVER_END')
                else:
                    self._begin('HOVER_TURN_PRE')
            else:
                self._dbg += 1
                # Closed-loop heading hold: steer to the corridor axis (0 rad on
                # outbound legs, pi on return legs) with a proportional yaw term.
                # This keeps the drone flying straight down the corridor even if
                # the U-turn over/undershot or left residual spin -- without it
                # the drone veers sideways into a wall and wedges (kinematic body).
                base = 0.0 if (self.leg_idx % 2 == 0) else math.pi
                wz = max(-0.4, min(0.4, self.a.heading_kp * self._wrap(base - self.yaw)))
                if self._dbg % 20 == 0:
                    self.get_logger().info(
                        f'  LEG d={d:.2f}/{self.a.leg_m}m  pos=({self.pos[0]:.2f},{self.pos[1]:.2f})  '
                        f'yaw={math.degrees(self.yaw):.0f}  wz={wz:.2f}')
                self._pub(vx=self.a.speed * self._ramp(d, self.a.leg_m, self.a.ramp_m), wz=wz)

        elif ph == 'HOVER_TURN_PRE':
            self._pub()
            if self._elapsed(self.phase_start) >= self.a.turn_hover_s:
                self.prev_yaw = self.yaw
                self.yaw_accum = 0.0
                self._begin('TURN')

        elif ph == 'TURN':
            d = self.yaw - self.prev_yaw
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            self.yaw_accum += abs(d)
            self.prev_yaw = self.yaw
            target = math.radians(self.a.turn_deg)
            if self.yaw_accum >= target:
                self._pub()  # stop yaw
                self._begin('HOVER_TURN_POST')
            else:
                s = self._ramp(self.yaw_accum, target, 0.4)
                self._pub(wz=self.a.turn * s)

        elif ph == 'HOVER_TURN_POST':
            self._pub()
            if self._elapsed(self.phase_start) >= self.a.turn_hover_s:
                self.ref_pos = None
                self._begin('LEG')

        elif ph == 'HOVER_END':
            self._pub()
            if self._elapsed(self.phase_start) >= self.a.hover_s:
                self.phase = 'DONE'


def main():
    p = argparse.ArgumentParser(description='GT-feedback U-turn multi-lap flyer.')
    p.add_argument('--laps', type=int, default=4, help='out-and-back laps (2 legs each).')
    p.add_argument('--leg-m', type=float, default=15.0, help='leg length, GT metres.')
    p.add_argument('--speed', type=float, default=0.6, help='cruise speed, m/s.')
    p.add_argument('--turn', type=float, default=0.5, help='yaw rate, rad/s.')
    p.add_argument('--turn-deg', type=float, default=178.0, help='U-turn angle, deg.')
    p.add_argument('--heading-kp', type=float, default=1.5,
                   help='proportional gain for in-leg heading hold (rad/s per rad).')
    p.add_argument('--ramp-m', type=float, default=1.5, help='accel/decel ramp, m.')
    p.add_argument('--hover-s', type=float, default=5.0, help='start/end hover, s.')
    p.add_argument('--turn-hover-s', type=float, default=3.0, help='hover each side of a turn, s.')
    p.add_argument('--rate', type=float, default=20.0, help='cmd publish rate, Hz.')
    p.add_argument('--topic', default='/argus/cmd_vel')
    p.add_argument('--gt-topic', default='/argus/ground_truth/pose')
    a = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv[1:]))

    rclpy.init()
    node = UTurnFlyer(a)
    # Hard wall-time safety bound so the flight can never hang the recorder.
    est = a.laps * 2 * (a.leg_m / max(0.05, a.speed * 0.25)) + a.laps * 2 * 30 + 60
    deadline = time.monotonic() + est
    try:
        while rclpy.ok() and node.phase != 'DONE' and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(f'flight finished: phase={node.phase}, legs={node.leg_idx}')
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            node._pub()  # ensure stopped
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
