#!/usr/bin/env python3
"""ARGUS drive_drone helper.

Convenience driver that publishes ``geometry_msgs/Twist`` on ``/argus/cmd_vel``
(the ROS->gz VelocityControl input per the frozen bridge contract) so the
kinematic drone can be flown without a teleop dependency. Useful for demoing the
sim and for handing motion to the downstream VIO pillar.

The twist is expressed in the drone body frame (FLU: +x forward, +y left,
+z up; +wz yaws left), matching the ``argus_drone`` VelocityControl plugin. The
drone is kinematic (gravity off + VelocityControl), so a zero twist = hover in
place; on exit this node always publishes a zero twist to stop cleanly.

Run::

    ros2 run argus_bringup drive_drone --pattern square --speed 0.5 --duration 4

Patterns: ``stop``/``hover`` (zero), ``forward``, ``backward``, ``left``,
``right`` (strafe), ``up``, ``down``, ``yaw`` (rotate), ``square`` (4 strafing
legs of --duration each, returns near the start). Explicit ``--vx/--vy/--vz/--wz``
override the pattern's linear/angular components.

Timing note: legs/durations are counted in WALL seconds. Under sim RTF < 1
(known iGPU render load, see the project notes) wall != sim time, so the drone
travels less sim-distance than ``speed * duration``. Fine for a demo helper; for
exact distances drive against ``/argus/ground_truth/pose`` instead.
"""

import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Single-velocity patterns: unit direction applied to --speed / --turn.
_LINEAR = {
    'stop': (0.0, 0.0, 0.0),
    'hover': (0.0, 0.0, 0.0),
    'forward': (1.0, 0.0, 0.0),
    'backward': (-1.0, 0.0, 0.0),
    'left': (0.0, 1.0, 0.0),
    'right': (0.0, -1.0, 0.0),
    'up': (0.0, 0.0, 1.0),
    'down': (0.0, 0.0, -1.0),
    'yaw': (0.0, 0.0, 0.0),
}
# square: per-leg unit linear direction (forward, left, backward, right).
_SQUARE_LEGS = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)]


class DriveDrone(Node):
    def __init__(self, args):
        super().__init__('drive_drone')
        self.args = args
        self.pub = self.create_publisher(Twist, args.topic, 10)

        self._legs = _SQUARE_LEGS if args.pattern == 'square' else None
        # Total run time: one leg-duration per square side, else --duration.
        self._total = args.duration * (len(_SQUARE_LEGS) if self._legs else 1)
        self._start = None
        self._done = False
        self.timer = self.create_timer(1.0 / args.rate, self._tick)

        self.get_logger().info(
            f"drive_drone: pattern='{args.pattern}' speed={args.speed} "
            f"turn={args.turn} for {self._total:.1f}s on {args.topic}")

    def _twist_for(self, elapsed):
        """Return the Twist for the current moment, or None when finished."""
        a = self.args
        if elapsed >= self._total:
            return None

        if self._legs is not None:
            leg = int(elapsed / a.duration)
            lx, ly, lz = self._legs[min(leg, len(self._legs) - 1)]
            wz = 0.0
        else:
            lx, ly, lz = _LINEAR[a.pattern]
            wz = 1.0 if a.pattern == 'yaw' else 0.0

        # Optional smooth accel/decel: scale linear velocity 0->1 over the first
        # --ramp seconds and 1->0 over the last --ramp seconds. Without it the
        # kinematic VelocityControl steps velocity in one physics step, which the
        # IMU reports as a huge acceleration impulse and the VIO sees as a
        # start/stop transient. A finite ramp keeps the motion physical.
        scale = 1.0
        if a.ramp > 0.0:
            if elapsed < a.ramp:
                scale = elapsed / a.ramp
            elif elapsed > self._total - a.ramp:
                scale = max(0.0, (self._total - elapsed) / a.ramp)

        t = Twist()
        # Explicit component overrides win over the pattern's unit direction.
        t.linear.x = (a.vx if a.vx is not None else lx * a.speed) * scale
        t.linear.y = (a.vy if a.vy is not None else ly * a.speed) * scale
        t.linear.z = (a.vz if a.vz is not None else lz * a.speed) * scale
        t.angular.z = (a.wz if a.wz is not None else wz * a.turn) * scale
        return t

    def _tick(self):
        if self._start is None:
            self._start = self.get_clock().now()
        elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9

        twist = self._twist_for(elapsed)
        if twist is None:
            self._finish()
            return
        self.pub.publish(twist)

    def _finish(self):
        if self._done:
            return
        self._done = True
        # Stop: publish a few zero twists so the drone station-keeps.
        for _ in range(5):
            self.pub.publish(Twist())
        self.get_logger().info('drive_drone: done, drone stopped (hover).')
        self.timer.cancel()
        # NOTE: do NOT call rclpy.shutdown() here. Shutting down from inside a
        # timer callback can deadlock when DDS peers are connected (e.g. the
        # running sim's cmd_vel subscriber) -- main() ends the spin via _done.


def _parse(argv):
    p = argparse.ArgumentParser(description='Publish /argus/cmd_vel to fly the kinematic drone.')
    p.add_argument('--pattern', default='forward', choices=sorted(set(_LINEAR) | {'square'}),
                   help='Motion pattern (default: forward).')
    p.add_argument('--speed', type=float, default=0.5, help='Linear speed, m/s (default 0.5).')
    p.add_argument('--turn', type=float, default=0.3, help='Yaw rate, rad/s (default 0.3).')
    p.add_argument('--duration', type=float, default=5.0,
                   help='Seconds (per leg for square) (default 5).')
    p.add_argument('--ramp', type=float, default=0.0,
                   help='Linear accel/decel ramp (s) at start and end; '
                        '0 = instant velocity step (default).')
    p.add_argument('--rate', type=float, default=20.0, help='Publish rate, Hz (default 20).')
    p.add_argument('--topic', default='/argus/cmd_vel', help='cmd_vel topic.')
    p.add_argument('--vx', type=float, help='Override linear x (m/s).')
    p.add_argument('--vy', type=float, help='Override linear y (m/s).')
    p.add_argument('--vz', type=float, help='Override linear z (m/s).')
    p.add_argument('--wz', type=float, help='Override angular z (rad/s).')
    return p.parse_args(argv)


def main(argv=None):
    import sys
    import time
    rclpy.init()
    # Strip ROS args (e.g. injected --ros-args) before our own parsing.
    args = _parse(rclpy.utilities.remove_ros_args(sys.argv[1:] if argv is None else argv))
    node = DriveDrone(args)
    # Spin until the motion finishes, bounded by a hard wall deadline so the
    # process can never hang (see _finish: shutdown-in-callback deadlocks with
    # DDS peers connected). The deadline also guarantees a clean stop+exit.
    deadline = time.monotonic() + node._total + 5.0
    try:
        while rclpy.ok() and not node._done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node._done:
            node._finish()
    except KeyboardInterrupt:
        node._finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
