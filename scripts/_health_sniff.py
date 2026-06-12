#!/usr/bin/env python3
"""Sniff /argus/vio/health + /argus/vio/point_cloud during a live VINS run.

Subscribes to the health monitor's output and to the raw VINS point cloud, logs a
timeline to CSV, and on exit prints a summary -- crucially the distribution of
real inlier feature counts (point-cloud size) across the flight, which is what the
health-monitor feat_nominal/feat_degraded/feat_lost thresholds get tuned against.

Run (env already sourced):
  python3 scripts/_health_sniff.py --duration 300 --out /tmp/health_timeline.csv
Stop early with SIGINT (Ctrl-C) -> still prints the summary.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
# NOTE: rclpy is imported lazily (live mode only). The offline --bag path needs
# only rosbags, so it runs in the eval venv which has no rclpy.

_NAME = {0: "INITIALIZING", 1: "NOMINAL", 2: "DEGRADED", 3: "LOST"}


def _pct(xs: list[float], q: float) -> float:
    if not xs:
        return float("nan")
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[i]


class Results:
    """Plain holder for sniffed rows (no rclpy dependency, so the offline --bag
    path runs in the eval venv). Field access matches both rclpy and rosbags
    deserialized messages."""

    def __init__(self) -> None:
        self.health_rows: list[tuple] = []
        self.cloud_sizes: list[int] = []

    def add_health(self, m) -> None:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.health_rows.append((t, m.status, m.confidence, m.num_inlier_features,
                                 m.avg_parallax, m.estimated_drift_rate,
                                 m.position_covariance_trace, int(m.imu_excitation_ok),
                                 m.processing_latency_ms))

    def add_cloud(self, m) -> None:
        self.cloud_sizes.append(len(m.points))


def summarize(s: Sniffer, out_path: str | None) -> None:
    rows = s.health_rows
    print("\n===== health sniff summary =====")
    print(f"health msgs: {len(rows)}   point_cloud msgs: {len(s.cloud_sizes)}")
    if not rows:
        print("NO health messages received -- check topic wiring / QoS / VINS state.")
        return

    # Status histogram.
    print("\nstatus distribution:")
    for code in (0, 1, 2, 3):
        n = sum(1 for r in rows if r[1] == code)
        if n:
            print(f"  {_NAME[code]:13s}: {n:5d}  ({100.0*n/len(rows):5.1f}%)")

    # Real feature-count distribution (the threshold-tuning signal).
    cs = [float(c) for c in s.cloud_sizes]
    if cs:
        print("\npoint_cloud inlier-count distribution (the threshold signal):")
        print(f"  n={len(cs)}  min={min(cs):.0f}  p10={_pct(cs,0.10):.0f}  "
              f"p25={_pct(cs,0.25):.0f}  median={_pct(cs,0.50):.0f}  "
              f"p75={_pct(cs,0.75):.0f}  p90={_pct(cs,0.90):.0f}  max={max(cs):.0f}")

    # Feature count conditioned on NOMINAL vs DEGRADED (helps set the band).
    for code in (1, 2, 3):
        fc = [float(r[3]) for r in rows if r[1] == code]
        if fc:
            print(f"  inliers|{_NAME[code]:11s}: min={min(fc):.0f} "
                  f"median={_pct(fc,0.5):.0f} max={max(fc):.0f}")

    par = [r[4] for r in rows if r[1] in (1, 2)]
    drift = [r[5] for r in rows]
    lat = [r[8] for r in rows if r[8] > 0]
    if par:
        print(f"\nparallax(px) NOMINAL/DEGRADED: min={min(par):.2f} "
              f"median={_pct(par,0.5):.2f} max={max(par):.2f}")
    print(f"drift_rate(m/s): min={min(drift):.3f} median={_pct(drift,0.5):.3f} max={max(drift):.3f}")
    if lat:
        print(f"latency(ms): min={min(lat):.1f} median={_pct(lat,0.5):.1f} max={max(lat):.1f}")

    if out_path:
        with open(out_path, "w") as fh:
            fh.write("t,status,confidence,inliers,parallax_px,drift_mps,cov_trace,excited,latency_ms\n")
            for r in rows:
                fh.write(",".join(str(x) for x in r) + "\n")
        print(f"\ntimeline CSV -> {out_path}")
    print("================================")


def from_bag(bag_dir: str, out_path: str | None) -> int:
    """Offline mode: read a recorded bag of /argus/vio/health + point_cloud and
    print the same summary (no live subscribers -> zero load on the VINS run)."""
    from pathlib import Path

    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS2_HUMBLE)
    # VIOHealth is a custom type the Humble store doesn't know -> register it from
    # the .msg so the recorded /argus/vio/health can be deserialized offline.
    try:
        from rosbags.typesys import get_types_from_msg
        msg_path = Path.home() / "argus/src/argus_msgs/msg/VIOHealth.msg"
        ts.register(get_types_from_msg(msg_path.read_text(), "argus_msgs/msg/VIOHealth"))
    except Exception as exc:  # noqa: BLE001
        print(f"[sniff] WARN: could not register VIOHealth ({exc}); health rows skipped")

    res = Results()
    with Reader(bag_dir) as r:
        for conn, _t, raw in r.messages():
            if conn.topic == "/argus/vio/health":
                res.add_health(ts.deserialize_cdr(raw, conn.msgtype))
            elif conn.topic == "/argus/vio/point_cloud":
                res.add_cloud(ts.deserialize_cdr(raw, conn.msgtype))
    summarize(res, out_path)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--bag", default=None, help="offline: analyze a recorded bag instead of live")
    args = ap.parse_args()

    if args.bag:
        return from_bag(args.bag, args.out)

    # Live mode: import rclpy lazily (system python + ROS env).
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud
    from argus_msgs.msg import VIOHealth

    rclpy.init()
    node = rclpy.create_node("health_sniffer")
    res = Results()
    rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(VIOHealth, "/argus/vio/health", res.add_health, rel)
    node.create_subscription(PointCloud, "/argus/vio/point_cloud", res.add_cloud, rel)
    stop = {"flag": False}

    def _sig(*_):
        stop["flag"] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    t0 = time.monotonic()
    while rclpy.ok() and not stop["flag"] and (time.monotonic() - t0) < args.duration:
        rclpy.spin_once(node, timeout_sec=0.2)

    summarize(res, args.out)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
