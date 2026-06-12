#!/usr/bin/env python3
"""ARGUS :: transcode_bag_mono.py — rgb8 -> mono8 stereo images in a sensor bag.

VINS converts incoming frames to MONO8 anyway (cv_bridge in getImageFromMsg);
publishing rgb8 just triples every image payload. On the 14 GB host the
estimator's image-size-proportional memory growth (day-7: ~3.6 MB/processed
frame even with show_track off) OOMs vins_node ~60 % into the 207 m Scenario E
replay. Feeding mono8 directly is bit-equivalent vision input (same standard
luma conversion VINS would apply), cuts the leak rate ~3x — comfortably past
the full lap — and shrinks the bag ~3x.

Usage (host eval venv):
  ~/.venvs/argus-eval/bin/python scripts/transcode_bag_mono.py \
      data/bags/scenario_E_tunnel_imu data/bags/scenario_E_tunnel_imu_mono
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

IMG_TOPICS = {'/argus/cam0/image_raw', '/argus/cam1/image_raw'}
# cv_bridge / OpenCV RGB->GRAY luma weights
W = np.array([0.299, 0.587, 0.114])


def main(src: str, dst: str) -> int:
    ts = get_typestore(Stores.ROS2_HUMBLE)
    src_p, dst_p = Path(src), Path(dst)
    if dst_p.exists():
        raise SystemExit(f"refusing to overwrite {dst}")
    n_img = 0
    with AnyReader([src_p], default_typestore=ts) as r, \
            Writer(dst_p, version=8) as w:
        conn_map = {c.id: w.add_connection(
            c.topic, c.msgtype, typestore=ts, serialization_format='cdr',
            offered_qos_profiles=c.ext.offered_qos_profiles)
            for c in r.connections}
        for conn, t_ns, raw in r.messages():
            if conn.topic in IMG_TOPICS:
                m = r.deserialize(raw, conn.msgtype)
                assert m.encoding == 'rgb8', m.encoding
                rgb = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                gray = (rgb @ W).round().astype(np.uint8)
                m.encoding = 'mono8'
                m.step = m.width
                m.data = gray.reshape(-1)
                raw = ts.serialize_cdr(m, conn.msgtype)
                n_img += 1
            w.write(conn_map[conn.id], t_ns, raw)
    print(f"[mono] transcoded {n_img} images -> {dst}")
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    sys.exit(main(sys.argv[1], sys.argv[2]))
