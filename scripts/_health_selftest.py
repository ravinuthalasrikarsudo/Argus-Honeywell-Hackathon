#!/usr/bin/env python3
"""Standalone state-machine self-test for the ARGUS health monitor.

Runs the real ``HealthMonitor`` node in-process against a synthetic driver that
scripts a healthy -> lights-off -> recovered timeline, with NO simulator. Cheap
and deterministic (does not touch the iGPU), so the Pillar-3 state machine and
recovery logic can be validated before wiring it onto a live VINS stack.

Timeline (wall seconds):
  [0,2)   INIT   : point_cloud + imu + propagated odom, but NO optimized odom
                   -> monitor should report INITIALIZING.
  [2,6)   HEALTHY: + fresh optimized odom (moving 0.5 m/s), 100 inliers
                   -> NOMINAL.
  [6,10)  DARK   : optimized odom STOPS (goes stale) and inliers drop to 2;
                   propagated odom keeps dead-reckoning forward (diverges)
                   -> LOST, recovery engages (hold + flag + count).
  [10,13) RELIT  : optimized odom resumes fresh, 100 inliers
                   -> NOMINAL, recovery clears.

Run (after building + sourcing install):
  source /opt/ros/humble/setup.bash && source ~/argus/install/setup.bash
  python3 scripts/_health_selftest.py
Exit code 0 = PASS.
"""

from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import Twist  # noqa: F401  (monitor publishes it; import keeps types loaded)
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Bool

from argus_health.health_monitor import HealthMonitor
from argus_msgs.msg import VIOHealth

_NAME = {0: "INITIALIZING", 1: "NOMINAL", 2: "DEGRADED", 3: "LOST"}


class SyntheticDriver(Node):
    """Publishes scripted VIO inputs and records the monitor's health output."""

    def __init__(self) -> None:
        super().__init__("health_selftest_driver")
        self.pub_opt = self.create_publisher(Odometry, "/argus/vio/odom_optimized", 10)
        self.pub_prop = self.create_publisher(Odometry, "/argus/vio/odom", 10)
        self.pub_cloud = self.create_publisher(PointCloud, "/argus/vio/point_cloud", 10)
        self.pub_imu = self.create_publisher(Imu, "/argus/imu", 10)
        self.create_subscription(VIOHealth, "/argus/vio/health", self._on_health, 10)
        self.create_subscription(Bool, "/argus/health/recovery_active", self._on_recovery, 10)

        self.t0 = time.monotonic()
        self.statuses_seen: set[int] = set()
        self.last_health: VIOHealth | None = None
        self.recovery_seen_true = False
        self.recovery_cleared_after_true = False
        self._recovery_prev = False
        self.done = False

        self.create_timer(0.05, self._drive)   # 20 Hz scripted inputs

    # -- monitor outputs --
    def _on_health(self, msg: VIOHealth) -> None:
        self.statuses_seen.add(msg.status)
        self.last_health = msg

    def _on_recovery(self, msg: Bool) -> None:
        if msg.data:
            self.recovery_seen_true = True
        elif self._recovery_prev and not msg.data and self.recovery_seen_true:
            self.recovery_cleared_after_true = True
        self._recovery_prev = msg.data

    # -- scripted inputs --
    def _drive(self) -> None:
        t = time.monotonic() - self.t0
        if t >= 13.0:
            self.done = True
            return

        # IMU: always "moving" (modest accel/gyro so excitation is plausible).
        imu = Imu()
        imu.angular_velocity.z = 0.05
        imu.linear_acceleration.x = 0.3
        imu.linear_acceleration.z = 9.8
        self.pub_imu.publish(imu)

        # Propagated odom: marches forward the whole time (IMU dead-reckoning).
        prop_x = 1.5 + 0.5 * t
        self.pub_prop.publish(self._odom(prop_x, speed=0.5))

        if t < 2.0:
            # INIT: feed features but no optimized odom yet.
            self._cloud(100, around_x=1.5)
        elif t < 6.0:
            # HEALTHY: fresh optimized odom tracking the propagated pose.
            self.pub_opt.publish(self._odom(prop_x, speed=0.5))
            self._cloud(100, around_x=prop_x)
        elif t < 10.0:
            # DARK: optimized odom STOPS; inliers collapse. Propagated keeps going.
            self._cloud(2, around_x=3.0)
        else:
            # RELIT: optimized odom resumes, features restored.
            self.pub_opt.publish(self._odom(prop_x, speed=0.5))
            self._cloud(100, around_x=prop_x)

    def _odom(self, x: float, speed: float) -> Odometry:
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "world"
        o.pose.pose.position.x = x
        o.pose.pose.position.z = 1.0
        o.pose.pose.orientation.w = 1.0
        o.twist.twist.linear.x = speed
        return o

    def _cloud(self, n: int, around_x: float) -> None:
        c = PointCloud()
        c.header.stamp = self.get_clock().now().to_msg()
        c.header.frame_id = "world"
        for i in range(n):
            p = Point32()
            p.x = around_x + 3.0 + (i % 5) * 0.5   # ~3-5 m ahead -> finite depth
            p.y = (i % 7) * 0.2
            p.z = 1.0 + (i % 3) * 0.3
            c.points.append(p)
        self.pub_cloud.publish(c)


def main() -> int:
    rclpy.init()
    monitor = HealthMonitor()
    driver = SyntheticDriver()
    ex = SingleThreadedExecutor()
    ex.add_node(monitor)
    ex.add_node(driver)

    while rclpy.ok() and not driver.done:
        ex.spin_once(timeout_sec=0.1)

    # --- Evaluate ---
    checks: list[tuple[str, bool]] = [
        ("saw INITIALIZING", HealthMonitor.INITIALIZING in driver.statuses_seen),
        ("saw NOMINAL", HealthMonitor.NOMINAL in driver.statuses_seen),
        ("saw LOST", HealthMonitor.LOST in driver.statuses_seen),
        ("recovery engaged (flag went true)", driver.recovery_seen_true),
        ("recovery cleared after relit", driver.recovery_cleared_after_true),
        ("recovery_count >= 1", monitor._recovery_count >= 1),
        ("final status NOMINAL", driver.last_health is not None
            and driver.last_health.status == HealthMonitor.NOMINAL),
        ("final drift_rate finite & >= 0", driver.last_health is not None
            and driver.last_health.estimated_drift_rate >= 0.0),
    ]

    print("\n=== health monitor self-test ===")
    seen = ", ".join(sorted(_NAME[s] for s in driver.statuses_seen))
    print(f"statuses observed : {seen}")
    print(f"recovery count    : {monitor._recovery_count}")
    if driver.last_health is not None:
        h = driver.last_health
        print(f"final report      : status={_NAME[h.status]} conf={h.confidence:.2f} "
              f"inliers={h.num_inlier_features} parallax={h.avg_parallax:.2f}px "
              f"drift={h.estimated_drift_rate:.3f}m/s cov={h.position_covariance_trace:.3f} "
              f"excited={h.imu_excitation_ok} lat={h.processing_latency_ms:.1f}ms")
    print("--------------------------------")
    ok = True
    for name, passed in checks:
        print(f"[{'PASS' if passed else 'FAIL'}] {name}")
        ok = ok and passed
    print("================================")
    print("RESULT:", "PASS" if ok else "FAIL")

    monitor.destroy_node()
    driver.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
