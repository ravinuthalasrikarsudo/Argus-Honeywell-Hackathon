#!/usr/bin/env python3
"""ARGUS :: fly_scenario_D.py  (Scenario D "lights-off")

Forward corridor flight that triggers a Zone-B BLACKOUT mid-traverse and is aware
of the VIO health monitor's recovery signal -- the demonstration of Pillar 3.

Flight: start hover (VINS init) -> drive +x at cruise. When ground truth crosses
into Zone B (x >= blackout_x) it kills all corridor lights via blackout.sh, so the
stereo cameras go dark, KLT/feature tracking starves, and the health monitor flips
to LOST. The lights are restored after dark_s seconds; tracking re-acquires and the
monitor returns to NOMINAL. The drone then completes the traverse to stop_x.

Two ablation modes (selected by --yield-recovery):
  * C3 (--yield-recovery): the planner YIELDS to the safety monitor. While
    /argus/health/recovery_active is true the forward command is dropped to a hold,
    so the drone stops in place during the blackout instead of flying blind. This
    is the recovery behaviour: detect LOST -> hold -> auto-resume when recovered.
  * C1 (default, no flag): the drone drives straight through the dark on dead
    reckoning (no recovery) -> it overshoots/drifts through the blackout. The
    baseline the recovery is measured against.

y/z/yaw are held on the corridor centreline by proportional ground-truth feedback
(same as fly_shuttle.py). Run with the ROS env sourced (rclpy + system python):

  python3 scripts/fly_scenario_D.py --yield-recovery --blackout-x 10 --dark-s 6
"""

import argparse
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScenarioDFlyer(Node):
    def __init__(self, a):
        super().__init__('scenario_d_flyer')
        self.a = a
        self.pub = self.create_publisher(Twist, a.topic, 10)
        self.create_subscription(PoseStamped, a.gt_topic, self._gt, 10)
        self.create_subscription(Bool, a.recovery_topic, self._recovery, 10)
        self.pos = None
        self.yaw = None
        self.recovery_active = False
        self.phase = 'WAIT_GT'
        self.x0 = None
        self.blacked = False
        self.restored = False
        self.blackout_t = None
        self.hold_start = None       # when the current recovery hold began
        self.resume_forced = False   # bounded-hold deadlock breaker latched
        self._dbg = 0
        self.phase_start = self.get_clock().now()
        self.timer = self.create_timer(1.0 / a.rate, self._tick)

    def _gt(self, msg):
        p = msg.pose.position
        self.pos = (p.x, p.y, p.z)
        self.yaw = yaw_of(msg.pose.orientation)

    def _recovery(self, msg):
        self.recovery_active = bool(msg.data)

    def _elapsed(self, since) -> float:
        return (self.get_clock().now() - since).nanoseconds / 1e9

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _wrap(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _holds(self):
        _, y, z = self.pos
        vy = self._clamp(self.a.lat_kp * (self.a.y0 - y), -0.3, 0.3)
        vz = self._clamp(self.a.alt_kp * (self.a.z0 - z), -0.3, 0.3)
        wz = self._clamp(self.a.yaw_kp * self._wrap(0.0 - self.yaw), -0.4, 0.4)
        return vy, vz, wz

    def _pub(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.linear.z = float(vz)
        t.angular.z = float(wz)
        self.pub.publish(t)

    def _hold_pub(self, vx=0.0):
        if self.pos is None or self.yaw is None:
            self._pub(vx=vx)
            return
        vy, vz, wz = self._holds()
        self._pub(vx=vx, vy=vy, vz=vz, wz=wz)

    def _lights(self, action):
        """Fire blackout.sh non-blocking (off|on). gz service call, ~1-2 s."""
        try:
            subprocess.Popen(['bash', self.a.blackout_script, action, self.a.world],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().warn(f'LIGHTS {action.upper()} (Zone-B blackout) @ x={self.pos[0]:.1f}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'blackout.sh {action} failed: {exc}')

    def _tick(self):
        if self.pos is None or self.yaw is None:
            return
        x = self.pos[0]
        ph = self.phase

        if ph == 'WAIT_GT':
            self.x0 = x
            self._begin('HOVER_START')

        elif ph == 'HOVER_START':
            self._hold_pub()
            if self._elapsed(self.phase_start) >= self.a.hover_s:
                self._begin('DRIVE')

        elif ph == 'DRIVE':
            # --- Blackout event management (position-triggered, time-restored) ---
            if not self.blacked and x >= self.a.blackout_x:
                self._lights('off')
                self.blacked = True
                self.blackout_t = self.get_clock().now()
            if self.blacked and not self.restored \
                    and self._elapsed(self.blackout_t) >= self.a.dark_s:
                self._lights('on')
                self.restored = True

            # --- Arrival ---
            if x >= self.a.stop_x:
                self._hold_pub()
                self._begin('HOVER_END')
                return

            # --- Drive, yielding to the safety monitor if armed (BOUNDED hold) ---
            yielding = (self.a.yield_recovery and self.recovery_active
                        and not self.resume_forced)
            if yielding and self.hold_start is None:
                self.hold_start = self.get_clock().now()
                self.get_logger().warn('YIELDING to recovery hold (VIO LOST) -- stopping')
            if yielding and self._elapsed(self.hold_start) >= self.a.max_hold_s:
                # A frozen drone has no parallax, so VINS can never re-acquire while
                # held -> after a bounded hold, resume (cautiously) to break the
                # deadlock. This is the recovery policy: hold, then re-acquire.
                self.resume_forced = True
                yielding = False
                self.get_logger().warn(
                    f'max hold {self.a.max_hold_s:.0f}s reached -> resuming to re-acquire')
            if yielding:
                self._hold_pub(vx=0.0)               # stop in place
            else:
                if self.hold_start is not None and not self.recovery_active:
                    self.get_logger().info('recovery cleared -> resuming forward')
                self.hold_start = None
                remaining = self.a.stop_x - x
                # Cautious half-speed creep while still flagged LOST (re-acquiring).
                base = self.a.speed * (0.5 if self.recovery_active else 1.0)
                speed = base * max(0.3, min(1.0, remaining / self.a.ramp_m))
                self._hold_pub(vx=speed)

            self._dbg += 1
            if self._dbg % 20 == 0:
                self.get_logger().info(
                    f'  DRIVE x={x:.2f}->{self.a.stop_x:.0f} y={self.pos[1]:.2f} '
                    f'z={self.pos[2]:.2f} blackout={self.blacked}/{self.restored} '
                    f'recovery={self.recovery_active}')

        elif ph == 'HOVER_END':
            self._hold_pub()
            # Safety: make sure lights end up restored.
            if not self.restored:
                self._lights('on')
                self.restored = True
            if self._elapsed(self.phase_start) >= self.a.hover_s:
                self.phase = 'DONE'

    def _begin(self, ph):
        self.phase = ph
        self.phase_start = self.get_clock().now()
        p = None if self.pos is None else tuple(round(v, 2) for v in self.pos)
        self.get_logger().info(f'phase -> {ph}  pos={p}')


def main():
    p = argparse.ArgumentParser(description='Scenario D lights-off forward flight.')
    p.add_argument('--speed', type=float, default=0.5)
    p.add_argument('--blackout-x', type=float, default=10.0, help='x to kill lights (Zone B entry).')
    p.add_argument('--dark-s', type=float, default=10.0, help='blackout duration, s.')
    p.add_argument('--stop-x', type=float, default=27.0, help='traverse end x.')
    p.add_argument('--ramp-m', type=float, default=2.0)
    p.add_argument('--yield-recovery', action='store_true',
                   help='C3: yield forward command to the recovery hold when LOST.')
    p.add_argument('--max-hold-s', type=float, default=8.0,
                   help='bounded recovery hold before forcing a re-acquisition resume.')
    p.add_argument('--y0', type=float, default=0.0)
    p.add_argument('--z0', type=float, default=1.0)
    # Holds default OFF: the kinematic drone does not drift off the centreline, and
    # continuous yaw/lateral correction rotates the stereo cameras enough to starve
    # KLT (inliers collapse to ~5, VINS diverges -- the rotation failure mode).
    # Pure forward translation keeps parallax clean (baseline_ABC: ~84 inliers).
    p.add_argument('--lat-kp', type=float, default=0.0)
    p.add_argument('--alt-kp', type=float, default=0.0)
    p.add_argument('--yaw-kp', type=float, default=0.0)
    p.add_argument('--hover-s', type=float, default=5.0)
    p.add_argument('--rate', type=float, default=20.0)
    p.add_argument('--topic', default='/argus/cmd_vel')
    p.add_argument('--gt-topic', default='/argus/ground_truth/pose')
    p.add_argument('--recovery-topic', default='/argus/health/recovery_active')
    p.add_argument('--world', default='warehouse_corridor')
    p.add_argument('--blackout-script', default='/home/vittal/argus/scripts/blackout.sh')
    a = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv[1:]))

    rclpy.init()
    node = ScenarioDFlyer(a)
    # Hard wall-time bound: traverse time + a generous allowance for the hold.
    est = (a.stop_x / max(0.05, a.speed * 0.3)) + 2 * a.hover_s + a.dark_s + 60
    deadline = time.monotonic() + est
    try:
        while rclpy.ok() and node.phase != 'DONE' and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(f'flight finished: phase={node.phase}')
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
