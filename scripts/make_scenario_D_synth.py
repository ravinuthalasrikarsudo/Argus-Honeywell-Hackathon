#!/usr/bin/env python3
"""ARGUS make_scenario_D_synth.py

Synthesize the Scenario D ("lights-off") sensor bag by DARKENING the stereo stream
of a known-good lit forward-flight bag (baseline_ABC) over the Zone-B window. The
physical effect on the estimator is identical to a gz light blackout -- the cameras
go dark, KLT/feature tracking starves, VINS degrades -- so the health monitor's
detection of the lights-out failure is demonstrated deterministically, without
depending on the live ogre2 render. The full live gz-blackout path
(blackout.sh + run_scenario_D.sh) remains in the tree for when the render works.

Only the cam0/cam1 image payloads inside the window are modified (multiplied by a
small factor); IMU / ground truth / clock / camera_info pass through untouched, so
the flight, timing and QoS are exactly the real recorded flight.

Run with the eval interpreter:
  ~/.venvs/argus-eval/bin/python scripts/make_scenario_D_synth.py
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS2_HUMBLE)
CAM = {"/argus/cam0/image_raw", "/argus/cam1/image_raw"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(Path.home() / "argus/data/bags/baseline_ABC"))
    ap.add_argument("--dst", default=str(Path.home() / "argus/data/bags/scenario_D"))
    ap.add_argument("--blackout-x", type=float, default=10.0, help="Zone-B entry: start darkening.")
    ap.add_argument("--restore-x", type=float, default=18.0, help="Zone-B exit: stop darkening.")
    ap.add_argument("--factor", type=float, default=0.03, help="brightness multiplier in the dark window.")
    a = ap.parse_args()
    src, dst = Path(a.src), Path(a.dst)
    if not src.is_dir():
        print(f"ERROR: src bag not found: {src}"); return 1
    if dst.exists():
        shutil.rmtree(dst)

    # 1. Find the GT time window where x in [blackout_x, restore_x].
    with Reader(src) as r:
        gt = [c for c in r.connections if c.topic == "/argus/ground_truth/pose"]
        xs = []
        for conn, t, raw in r.messages(connections=gt):
            m = TS.deserialize_cdr(raw, conn.msgtype)
            xs.append((t, m.pose.position.x))
    t_enter = next((t for t, x in xs if x >= a.blackout_x), None)
    t_exit = next((t for t, x in xs if x >= a.restore_x), None)
    if t_enter is None:
        print(f"ERROR: GT never reached blackout_x={a.blackout_x}"); return 1
    if t_exit is None:
        t_exit = xs[-1][0]
    print(f"[synth] dark window: t=[{t_enter*1e-9:.2f},{t_exit*1e-9:.2f}]s "
          f"({(t_exit-t_enter)*1e-9:.1f}s), x in [{a.blackout_x},{a.restore_x}] m, factor={a.factor}")

    # 2. Rewrite the bag, darkening cam frames inside the window.
    n_dark = 0
    with Reader(src) as r, Writer(dst, version=8) as w:
        cmap = {}
        for conn in r.connections:
            cmap[conn.id] = w.add_connection(
                conn.topic, conn.msgtype, typestore=TS,
                offered_qos_profiles=conn.ext.offered_qos_profiles)
        for conn, t, raw in r.messages():
            if conn.topic in CAM and t_enter <= t <= t_exit:
                m = TS.deserialize_cdr(raw, conn.msgtype)
                m.data = (m.data.astype(np.float32) * a.factor).astype(np.uint8)
                raw = TS.serialize_cdr(m, conn.msgtype)
                n_dark += 1
            w.write(cmap[conn.id], t, raw)
    print(f"[synth] darkened {n_dark} cam frames -> {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
