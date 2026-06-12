#!/usr/bin/env python3
"""Measure the cam0-vs-cam1 header-stamp offset in a rosbag (no VINS needed).

VINS-Fusion stereo sync discards an image when |t_cam0 - t_cam1| > 3 ms; if cam0
runs consistently ahead it throws every cam1 and never forms a stereo pair. This
prints the per-pair offset distribution so we can tell which recorded bag VINS can
actually use.

Run with the eval interpreter (has rosbags):
  ~/.venvs/argus-eval/bin/python scripts/_bag_stereo_offset.py <bag_dir> [<bag_dir> ...]
"""
from __future__ import annotations

import sys
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS2_HUMBLE)
TOL = 0.003


def stamps(reader: Reader, topic: str, maxn: int = 80) -> list[float]:
    """First `maxn` header stamps for `topic` (the offset is systematic, so a
    small prefix is enough and avoids deserializing every full 720p image)."""
    out = []
    conns = [c for c in reader.connections if c.topic == topic]
    for conn, _ts, raw in reader.messages(connections=conns):
        m = TS.deserialize_cdr(raw, conn.msgtype)
        out.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if len(out) >= maxn:
            break
    return sorted(out)


def analyze(bag: Path) -> None:
    with Reader(bag) as r:
        t0 = stamps(r, "/argus/cam0/image_raw")
        t1 = stamps(r, "/argus/cam1/image_raw")
    print(f"\n=== {bag.name} ===  cam0={len(t0)} cam1={len(t1)} frames")
    if not t0 or not t1:
        print("  missing a camera stream")
        return
    # For each cam0 stamp, nearest cam1 stamp; report signed offset cam0-cam1.
    import bisect
    offs = []
    for a in t0:
        i = bisect.bisect_left(t1, a)
        cands = []
        if i < len(t1):
            cands.append(t1[i])
        if i > 0:
            cands.append(t1[i - 1])
        b = min(cands, key=lambda x: abs(x - a))
        offs.append(a - b)
    offs.sort()
    n = len(offs)
    med = offs[n // 2]
    pairable = sum(1 for o in offs if abs(o) <= TOL)
    print(f"  offset cam0-cam1 (s): min={offs[0]:+.4f} median={med:+.4f} max={offs[-1]:+.4f}")
    print(f"  pairable (|off|<=3ms): {pairable}/{n}  ({100.0*pairable/n:.1f}%)"
          f"  -> {'OK for VINS stereo' if pairable > 0.5 * n else 'STEREO WILL FAIL'}")


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    for arg in sys.argv[1:]:
        analyze(Path(arg))
    return 0


if __name__ == "__main__":
    sys.exit(main())
