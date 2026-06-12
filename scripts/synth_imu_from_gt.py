#!/usr/bin/env python3
"""ARGUS :: synth_imu_from_gt.py — restore IMU physics for kinematic-drive bags.

Day-7 root cause: the contract drone is driven by gz VelocityControl, which
SETS link velocity each physics step — the body has no dynamics, so the gz IMU
system reports linear_acceleration = (0, 0, +9.8) CONSTANT (the velocity step,
the end-cap centripetal force, everything is invisible) and zero noise. The
gyro is fine (angular velocity is imposed directly and read back exactly).
VINS therefore initialises against an accelerometer that contradicts vision —
the systematic 11–26° gravity-tilt of iterations 3–5.

This tool rewrites a recorded sensor bag with a physically-true IMU, exactly
what the gz IMU plugin would produce given true dynamics (and what synthetic
VIO benchmarks do): linear acceleration is derived from the 250 Hz ground
truth (Savitzky–Golay double differentiation), rotated into the body frame,
gravity-reacted, and both channels get the CONTRACT noise model the estimator
config assumes (acc_n 0.002, gyr_n 1.7e-4, x sqrt(250 Hz)). The gyro VALUES
are kept from the sim (exact) + noise. Seeded -> deterministic output.

    f_b = R_wb^T (a_w + [0,0,9.8]) + n_a,   w_b = w_sim + n_g

Usage (host eval venv):
  ~/.venvs/argus-eval/bin/python scripts/synth_imu_from_gt.py \
      data/bags/scenario_E_tunnel data/bags/scenario_E_tunnel_imu
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation, Slerp

from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

G = 9.8                      # contract gravity (world -z), IMU reads +9.8 at rest
ACC_N, GYR_N, RATE = 0.002, 0.00016968, 250.0
SG_WIN, SG_POLY = 25, 3      # 0.1 s smoothing window at 250 Hz


def main(src: str, dst: str) -> int:
    ts = get_typestore(Stores.ROS2_HUMBLE)
    src_p, dst_p = Path(src), Path(dst)
    assert src_p.is_dir(), f"missing {src}"
    if dst_p.exists():
        raise SystemExit(f"refusing to overwrite {dst}")

    # ---- pass 1: ground truth kinematics ----
    t, p, q = [], [], []
    with AnyReader([src_p], default_typestore=ts) as r:
        gt_c = [c for c in r.connections if c.topic == '/argus/ground_truth/pose']
        for conn, _, raw in r.messages(connections=gt_c):
            m = r.deserialize(raw, conn.msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec / 1e9)
            p.append([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            q.append([m.pose.orientation.x, m.pose.orientation.y,
                      m.pose.orientation.z, m.pose.orientation.w])
    t = np.asarray(t)
    p = np.asarray(p)
    # keep a strictly-increasing subsequence (recorded stream has duplicate and
    # occasionally out-of-order header stamps)
    run_max = np.maximum.accumulate(np.concatenate(([-np.inf], t[:-1])))
    keep = t > run_max + 1e-9
    t, p = t[keep], p[keep]
    q = [qq for qq, k in zip(q, keep) if k]
    dt = float(np.median(np.diff(t)))
    a_w = np.column_stack([
        savgol_filter(p[:, i], SG_WIN, SG_POLY, deriv=2, delta=dt) for i in range(3)])
    rots = Rotation.from_quat(q)
    slerp = Slerp(t, rots)
    print(f"[synth] GT {len(t)} poses @ {1/dt:.0f} Hz; |a_w| max={np.linalg.norm(a_w,axis=1).max():.2f} m/s2")

    rng = np.random.default_rng(42)
    acc_sigma = ACC_N * np.sqrt(RATE)
    gyr_sigma = GYR_N * np.sqrt(RATE)

    def specific_force(stamp: float) -> np.ndarray | None:
        if stamp < t[0] or stamp > t[-1]:
            return None
        a = np.array([np.interp(stamp, t, a_w[:, i]) for i in range(3)])
        R_wb = slerp([stamp])[0]
        return R_wb.inv().apply(a + np.array([0.0, 0.0, G]))

    # ---- pass 2: stream-copy the bag, rewriting /argus/imu ----
    n_imu = n_skip = 0
    with AnyReader([src_p], default_typestore=ts) as r, \
            Writer(dst_p, version=8) as w:
        conn_map = {}
        for c in r.connections:
            conn_map[c.id] = w.add_connection(
                c.topic, c.msgtype, typestore=ts,
                serialization_format='cdr',
                offered_qos_profiles=c.ext.offered_qos_profiles)
        for conn, t_ns, raw in r.messages():
            if conn.topic == '/argus/imu':
                m = r.deserialize(raw, conn.msgtype)
                stamp = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
                f_b = specific_force(stamp)
                if f_b is None:        # outside GT coverage: drop (bag edges)
                    n_skip += 1
                    continue
                f_b = f_b + rng.normal(0.0, acc_sigma, 3)
                w_b = (np.array([m.angular_velocity.x, m.angular_velocity.y,
                                 m.angular_velocity.z])
                       + rng.normal(0.0, gyr_sigma, 3))
                m.linear_acceleration.x, m.linear_acceleration.y, \
                    m.linear_acceleration.z = map(float, f_b)
                m.angular_velocity.x, m.angular_velocity.y, \
                    m.angular_velocity.z = map(float, w_b)
                raw = ts.serialize_cdr(m, conn.msgtype)
                n_imu += 1
            w.write(conn_map[conn.id], t_ns, raw)
    print(f"[synth] rewrote {n_imu} IMU msgs ({n_skip} dropped at GT edges) -> {dst}")
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    sys.exit(main(sys.argv[1], sys.argv[2]))
