#!/usr/bin/env python3
"""ARGUS VIO health monitor (Pillar 3).

A standalone watchdog that observes the running VINS-Fusion estimator through its
*public* ``/argus/vio/*`` topics and publishes a single ``argus_msgs/VIOHealth``
report plus a recovery signal. It does **not** patch or rebuild VINS -- every
field is derived from observable topics, which is the honest contract an external
health monitor lives under (VINS-Fusion does not publish feature counts,
parallax, or pose covariance as topics).

Signal sources (all remapped onto the frozen schema):

  * ``/argus/vio/point_cloud``  (sensor_msgs/PointCloud) -- the sliding-window
    triangulated inlier landmarks. ``len(points)`` is the best observable proxy
    for the inlier feature count; it craters when the cameras go dark (Scenario
    D) or stare at the blank Zone-B walls.
  * ``/argus/vio/odom_optimized`` (nav_msgs/Odometry) -- the keyframe-rate
    optimized estimate. Only published once the estimator has converged
    (solver_flag == NON_LINEAR), so its first arrival ends INITIALIZING and a
    stale age means the optimizer has stalled -> LOST.
  * ``/argus/vio/odom`` (nav_msgs/Odometry) -- the high-rate IMU-propagated
    odometry. Its divergence from the optimized pose is a direct, observable
    drift signal: the two agree when tracking is healthy and split apart when
    the estimator falls back to IMU-only dead-reckoning.
  * ``/argus/imu`` (sensor_msgs/Imu) -- gyro/accel variance over a short window,
    the secondary excitation term.

Derived health fields and how each is obtained:

  num_tracked_features / num_inlier_features
      Both set from the point-cloud size. VINS publishes only the solved inlier
      landmarks, not the raw tracker count, so the two report the same observed
      quantity (documented proxy, not a fabricated tracker number).
  avg_parallax
      Motion-aware proxy: focal * speed * keyframe_dt / mean_depth. Parallax is
      proportional to how far features sweep across the image between keyframes,
      which is translation over depth. Real triangulation strength signal.
  position_covariance_trace
      VINS publishes a zero covariance, so this is an *estimated* uncertainty:
      the squared IMU-vs-optimized divergence plus a feature-scarcity term. Grows
      when the optimizer disagrees with dead-reckoning or runs out of landmarks.
  estimated_drift_rate
      Rate of growth of the IMU-vs-optimized position divergence (m/s), smoothed.
      ~0 when healthy, climbs the moment the estimator loses the cameras.
  imu_excitation_ok
      For the *noise-free* kinematic IMU, translational speed is the
      practical excitation/parallax signal (a constant-velocity cruise reads zero
      accel/gyro variance yet still gives parallax). So excitation is
      speed > speed_min OR gyro/accel variance above threshold (the latter is the
      term that matters on a real noisy IMU).
  processing_latency_ms
      now - optimized-odom header stamp, the end-to-end pipeline latency.

Recovery (the Pillar-3 "recovery" enabled in ablation config C3): when the health
status sits at LOST for ``lost_persist_s``, the monitor engages recovery -- it
flags ``/argus/health/recovery_active`` true, emits a ``/argus/health/recovery_event``
log line, counts the activation, and (optionally) commands a zero-velocity hold on
``/argus/cmd_vel`` so the drone stops flying blind instead of dead-reckoning into a
wall. When the estimator recovers (status leaves LOST) the hold is released and the
flag clears. A full return-to-last-good-pose maneuver is planner territory (Pillar
4, out of hackathon scope); detect + hold + flag + auto-resume is the realistic
Pillar-3 behaviour and is what the C1-vs-C3 ablation measures.
"""

from __future__ import annotations

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud
from std_msgs.msg import Bool, String

from argus_msgs.msg import VIOHealth


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class HealthMonitor(Node):
    """Derives a VIOHealth report from the live VINS-Fusion output topics."""

    # VIOHealth status enum (mirrors argus_msgs/VIOHealth.msg).
    INITIALIZING = VIOHealth.STATUS_INITIALIZING
    NOMINAL = VIOHealth.STATUS_NOMINAL
    DEGRADED = VIOHealth.STATUS_DEGRADED
    LOST = VIOHealth.STATUS_LOST

    _STATUS_NAME = {
        VIOHealth.STATUS_INITIALIZING: "INITIALIZING",
        VIOHealth.STATUS_NOMINAL: "NOMINAL",
        VIOHealth.STATUS_DEGRADED: "DEGRADED",
        VIOHealth.STATUS_LOST: "LOST",
    }

    def __init__(self) -> None:
        super().__init__("argus_health_monitor")

        # --- Parameters (thresholds tuned against real counts in Scenario D) ---
        self.declare_parameter("publish_rate_hz", 20.0)
        # Feature-count (point-cloud size) bands. Calibrated against a live VINS
        # run on baseline_ABC: inlier median ~84, p25 ~64, p10 ~12 (the low tail is
        # the blank Zone-B walls). feat_degraded catches that low-texture tail.
        self.declare_parameter("feat_nominal", 80)     # >= this -> full feature score
        self.declare_parameter("feat_degraded", 40)    # < this  -> DEGRADED (Zone-B tail)
        self.declare_parameter("feat_lost", 5)         # < this  -> LOST (near-blackout)
        self.declare_parameter("feat_smooth_n", 5)     # median window on per-keyframe count
        # Liveness / timing. Liveness keys off the HIGH-RATE imu_propagate odom
        # (/argus/vio/odom), which streams continuously while VINS is alive. The
        # optimized (keyframe-rate) odom is bursty -- gaps > 1 s are normal -- so it
        # only feeds a softer "optimizer struggling" DEGRADED hint, never LOST.
        self.declare_parameter("lost_timeout_s", 1.0)        # no high-rate odom -> LOST
        self.declare_parameter("opt_stale_degraded_s", 3.0)  # optimized odom stale -> DEGRADED
        self.declare_parameter("init_grace_s", 3.0)          # startup window label
        self.declare_parameter("lost_persist_s", 0.4)        # LOST must hold before recovery engages
        self.declare_parameter("recovery_clear_persist_s", 1.5)  # non-LOST must hold before clearing
        # Excitation / parallax.
        self.declare_parameter("speed_min", 0.05)           # m/s, motion = excitation
        self.declare_parameter("gyro_excite_std", 0.02)     # rad/s std threshold
        self.declare_parameter("accel_excite_std", 0.15)    # m/s^2 std threshold
        self.declare_parameter("imu_window_s", 0.5)
        self.declare_parameter("focal_px", 640.0)
        # Parallax proxy is small in this corridor (median ~0.9 px) and noisy, so
        # it is a REPORTED field, not a status trigger by default. Set > 0 to arm
        # it as a DEGRADED trigger (px).
        self.declare_parameter("parallax_degraded_px", 0.0)
        # Require excitation for NOMINAL (hover = poor parallax -> DEGRADED).
        self.declare_parameter("require_excitation", True)
        # Recovery.
        self.declare_parameter("enable_recovery", True)
        self.declare_parameter("recovery_hold_cmd", True)   # publish zero cmd_vel while LOST
        # Uncertainty proxy scale (m^2 per missing feature).
        self.declare_parameter("cov_feature_scale", 2.0)

        g = self.get_parameter
        self.publish_rate = float(g("publish_rate_hz").value)
        self.feat_nominal = int(g("feat_nominal").value)
        self.feat_degraded = int(g("feat_degraded").value)
        self.feat_lost = int(g("feat_lost").value)
        self.feat_smooth_n = max(1, int(g("feat_smooth_n").value))
        self.lost_timeout = float(g("lost_timeout_s").value)
        self.opt_stale_degraded = float(g("opt_stale_degraded_s").value)
        self.init_grace = float(g("init_grace_s").value)
        self.lost_persist = float(g("lost_persist_s").value)
        self.recovery_clear_persist = float(g("recovery_clear_persist_s").value)
        self.speed_min = float(g("speed_min").value)
        self.gyro_excite_std = float(g("gyro_excite_std").value)
        self.accel_excite_std = float(g("accel_excite_std").value)
        self.imu_window = float(g("imu_window_s").value)
        self.focal = float(g("focal_px").value)
        self.parallax_degraded = float(g("parallax_degraded_px").value)
        self.require_excitation = bool(g("require_excitation").value)
        self.enable_recovery = bool(g("enable_recovery").value)
        self.recovery_hold_cmd = bool(g("recovery_hold_cmd").value)
        self.cov_feature_scale = float(g("cov_feature_scale").value)

        # --- State ---
        self._start_t = self._now()
        self._first_optimized = False
        # Liveness uses WALL-CLOCK receive times (monotonic): the simulator runs at
        # RTF < 1 and VINS can lag several seconds in *sim* time under load, so a
        # sim-time staleness test false-trips LOST on a working-but-lagging
        # estimator. Messages still arrive continuously in wall time while VINS is
        # alive, so wall-clock liveness is the robust signal. Sim time is used only
        # for output header stamps and the reported estimate latency.
        self._opt_t: float | None = None          # last optimized-odom rx time, SIM (s)
        self._opt_wall: float | None = None        # last optimized-odom rx time, WALL (s)
        self._opt_pos: tuple[float, float, float] | None = None
        self._opt_speed = 0.0
        self._opt_interval = 1.0 / 13.0            # measured keyframe dt (sim s); seeded
        self._latency_ms = 0.0
        self._prop_wall: float | None = None       # last high-rate odom rx time, WALL (s)
        self._prop_pos: tuple[float, float, float] | None = None
        self._feature_count = 0                    # raw latest count (reported)
        self._cloud_hist: deque[int] = deque(maxlen=self.feat_smooth_n)  # for status
        self._mean_depth = 0.0
        self._imu_buf: deque[tuple[float, float, float, float, float, float, float]] = deque()
        # Drift bookkeeping (divergence rate).
        self._prev_div: float | None = None
        self._prev_div_t: float | None = None
        self._drift_rate = 0.0
        # Recovery bookkeeping.
        self._lost_since: float | None = None
        self._recovered_since: float | None = None
        self._recovery_active = False
        self._recovery_count = 0

        # --- QoS: VINS + the gz bridge publish RELIABLE (deviation #7). A
        #     BEST_EFFORT sub receives nothing over Cyclone, so subscribe RELIABLE. ---
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        rel_imu = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Odometry, "/argus/vio/odom_optimized", self._on_optimized, rel)
        self.create_subscription(Odometry, "/argus/vio/odom", self._on_propagate, rel)
        self.create_subscription(PointCloud, "/argus/vio/point_cloud", self._on_cloud, rel)
        self.create_subscription(Imu, "/argus/imu", self._on_imu, rel_imu)

        self._pub_health = self.create_publisher(VIOHealth, "/argus/vio/health", 10)
        self._pub_recovery = self.create_publisher(Bool, "/argus/health/recovery_active", 10)
        self._pub_event = self.create_publisher(String, "/argus/health/recovery_event", 10)
        self._pub_cmd = self.create_publisher(Twist, "/argus/cmd_vel", 10)

        self.create_timer(1.0 / self.publish_rate, self._tick)

        self.get_logger().info(
            "argus_health_monitor up: feat bands nominal/degraded/lost = "
            f"{self.feat_nominal}/{self.feat_degraded}/{self.feat_lost}, "
            f"recovery={'on' if self.enable_recovery else 'off'} "
            f"(hold_cmd={'on' if self.recovery_hold_cmd else 'off'})"
        )

    # ------------------------------------------------------------------ time
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -------------------------------------------------------------- callbacks
    def _on_optimized(self, msg: Odometry) -> None:
        now = self._now()
        if self._opt_t is not None:
            dt = now - self._opt_t
            if 0.0 < dt < 1.0:  # ignore the first sample and stale gaps
                # Low-pass the keyframe interval (sim s) for the parallax estimate.
                self._opt_interval = 0.8 * self._opt_interval + 0.2 * dt
        self._opt_t = now
        self._opt_wall = time.monotonic()
        p = msg.pose.pose.position
        self._opt_pos = (p.x, p.y, p.z)
        v = msg.twist.twist.linear
        self._opt_speed = _norm3(v.x, v.y, v.z)
        self._first_optimized = True
        # Pipeline latency = now - stamp (end-to-end), guarded against clock skew.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            # Staleness of the optimized estimate vs. wall/sim now (the optimized
            # keyframe lags the latest frame by up to the window time).
            self._latency_ms = _clamp((now - stamp) * 1e3, 0.0, 5000.0)

    def _on_propagate(self, msg: Odometry) -> None:
        self._prop_wall = time.monotonic()
        p = msg.pose.pose.position
        self._prop_pos = (p.x, p.y, p.z)

    def _on_cloud(self, msg: PointCloud) -> None:
        self._feature_count = len(msg.points)
        self._cloud_hist.append(self._feature_count)
        # Mean depth of window landmarks from the current camera centre (~ body
        # pose); feeds the parallax proxy. Points are in the world frame.
        if self._opt_pos is not None and msg.points:
            cx, cy, cz = self._opt_pos
            total = 0.0
            for pt in msg.points:
                total += _norm3(pt.x - cx, pt.y - cy, pt.z - cz)
            self._mean_depth = total / len(msg.points)

    def _on_imu(self, msg: Imu) -> None:
        now = self._now()
        g = msg.angular_velocity
        a = msg.linear_acceleration
        self._imu_buf.append((now, g.x, g.y, g.z, a.x, a.y, a.z))
        cutoff = now - self.imu_window
        while self._imu_buf and self._imu_buf[0][0] < cutoff:
            self._imu_buf.popleft()

    # ---------------------------------------------------------------- helpers
    def _imu_excited(self) -> bool:
        """True if recent motion makes observability healthy.

        Primary term = translational speed (the only excitation a noise-free
        constant-velocity sim IMU exposes). Secondary term = gyro/accel variance,
        which is what carries the signal on a real noisy IMU.
        """
        if self._opt_speed > self.speed_min:
            return True
        n = len(self._imu_buf)
        if n < 3:
            return False
        # Std-dev of gyro-norm and accel-norm over the window.
        gs = [_norm3(r[1], r[2], r[3]) for r in self._imu_buf]
        as_ = [_norm3(r[4], r[5], r[6]) for r in self._imu_buf]
        return self._std(gs) > self.gyro_excite_std or self._std(as_) > self.accel_excite_std

    @staticmethod
    def _std(xs: list[float]) -> float:
        n = len(xs)
        if n < 2:
            return 0.0
        m = sum(xs) / n
        return math.sqrt(sum((x - m) ** 2 for x in xs) / n)

    def _parallax(self) -> float:
        """focal * speed * keyframe_dt / mean_depth (px). 0 if depth unknown."""
        if self._mean_depth <= 1e-3:
            return 0.0
        return self.focal * self._opt_speed * self._opt_interval / self._mean_depth

    def _feat_smoothed(self) -> int:
        """Median of the recent per-keyframe inlier counts (status decision input).
        The raw per-keyframe count is a noisy step function -- a single sparse
        keyframe should not flip the status -- so threshold on the median."""
        if not self._cloud_hist:
            return self._feature_count
        return int(sorted(self._cloud_hist)[len(self._cloud_hist) // 2])

    def _update_drift(self, now: float) -> None:
        """Drift rate = growth of IMU-vs-optimized position divergence (m/s)."""
        if self._opt_pos is None or self._prop_pos is None:
            return
        ox, oy, oz = self._opt_pos
        px, py, pz = self._prop_pos
        div = _norm3(px - ox, py - oy, pz - oz)
        if self._prev_div is not None and self._prev_div_t is not None:
            dt = now - self._prev_div_t
            if dt > 1e-3:
                raw = (div - self._prev_div) / dt
                # EMA, clamped non-negative (we report drift accumulation rate).
                self._drift_rate = 0.7 * self._drift_rate + 0.3 * max(raw, 0.0)
        self._prev_div = div
        self._prev_div_t = now

    # ------------------------------------------------------------------- tick
    def _tick(self) -> None:
        now = self._now()
        self._update_drift(now)

        feats = self._feature_count          # raw latest (reported)
        feats_s = self._feat_smoothed()      # median window (status decision)
        excited = self._imu_excited()
        parallax = self._parallax()
        # Liveness from the high-rate imu_propagate odom, measured in WALL time
        # (robust to sim RTF<1 and VINS lag); optimized age is a softer "optimizer
        # keeping up?" hint only.
        wall = time.monotonic()
        age_prop = (wall - self._prop_wall) if self._prop_wall is not None else float("inf")
        age_opt = (wall - self._opt_wall) if self._opt_wall is not None else float("inf")

        # --- Status state machine ---
        parallax_bad = self.parallax_degraded > 0.0 and parallax < self.parallax_degraded
        if not self._first_optimized:
            status = self.INITIALIZING
        elif age_prop > self.lost_timeout:
            status = self.LOST                       # estimator stopped emitting odom
        elif feats_s < self.feat_lost:
            status = self.LOST                       # almost no inliers left
        elif (feats_s < self.feat_degraded
              or age_opt > self.opt_stale_degraded
              or (self.require_excitation and not excited)
              or parallax_bad):
            status = self.DEGRADED
        else:
            status = self.NOMINAL

        # --- Confidence [0..1] ---
        if status == self.INITIALIZING:
            confidence = 0.0
        else:
            span = max(self.feat_nominal - self.feat_lost, 1)
            feat_score = _clamp((feats_s - self.feat_lost) / span, 0.0, 1.0)
            fresh_score = _clamp(1.0 - age_prop / self.lost_timeout, 0.0, 1.0)
            excite_score = 1.0 if excited else 0.6
            confidence = feat_score * fresh_score * (0.7 + 0.3 * excite_score)
            if status == self.LOST:
                confidence = min(confidence, 0.1)

        # --- Uncertainty proxies ---
        div = self._prev_div if self._prev_div is not None else 0.0
        cov_trace = div * div + self.cov_feature_scale / max(feats, 1)

        # --- Recovery logic (LOST-triggered, persistence-gated) ---
        self._handle_recovery(now, status)

        # --- Publish VIOHealth ---
        msg = VIOHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "argus_drone"
        msg.status = status
        msg.confidence = float(_clamp(confidence, 0.0, 1.0))
        msg.num_tracked_features = int(min(feats, 65535))
        msg.num_inlier_features = int(min(feats, 65535))
        msg.avg_parallax = float(parallax)
        msg.position_covariance_trace = float(cov_trace)
        msg.estimated_drift_rate = float(self._drift_rate)
        msg.imu_excitation_ok = bool(excited)
        msg.processing_latency_ms = float(self._latency_ms)
        self._pub_health.publish(msg)

    def _handle_recovery(self, now: float, status: int) -> None:
        is_lost = status == self.LOST
        # Track continuous LOST / non-LOST dwell for hysteresis (de-bounce the
        # flicker when feature count hovers at the threshold in low-texture zones).
        if is_lost:
            if self._lost_since is None:
                self._lost_since = now
            self._recovered_since = None
        else:
            self._lost_since = None
            if self._recovered_since is None:
                self._recovered_since = now

        want_engage = (
            self.enable_recovery
            and is_lost
            and self._lost_since is not None
            and (now - self._lost_since) >= self.lost_persist
        )
        want_clear = (
            self._recovery_active
            and not is_lost
            and self._recovered_since is not None
            and (now - self._recovered_since) >= self.recovery_clear_persist
        )

        if want_engage and not self._recovery_active:
            self._recovery_active = True
            self._recovery_count += 1
            self._emit_event(
                f"RECOVERY ENGAGED (#{self._recovery_count}): VIO LOST, "
                f"{self._feature_count} inliers -- holding position"
            )
        elif want_clear:
            self._recovery_active = False
            self._emit_event("RECOVERY CLEARED: VIO recovered, releasing hold")

        # While recovery is active, hold position with a zero-velocity command so
        # the drone stops dead-reckoning into the dark (opt-in via param).
        if self._recovery_active and self.recovery_hold_cmd:
            self._pub_cmd.publish(Twist())

        self._pub_recovery.publish(Bool(data=self._recovery_active))

    def _emit_event(self, text: str) -> None:
        self.get_logger().warn(text)
        self._pub_event.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
