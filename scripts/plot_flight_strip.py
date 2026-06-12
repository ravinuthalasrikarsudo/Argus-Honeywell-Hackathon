#!/usr/bin/env python3
"""Drone-camera POV strip from the LIT baseline_live_day6 flight bag (headless, reliable):
6 frames along the corridor = the drone flying through the GPS-denied warehouse."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS2_HUMBLE)
BAG = Path("/home/vittal/argus/data/bags/baseline_live_day6")
OUT = "/home/vittal/argus/data/eval/flight_pov_strip.png"
want = [110, 260, 410, 560, 710, 900]
frames = {}

with AnyReader([BAG], default_typestore=TS) as r:
    conns = [c for c in r.connections if c.topic == "/argus/cam0/image_raw"]
    i = 0
    for conn, ts, raw in r.messages(connections=conns):
        if i in want:
            m = r.deserialize(raw, conn.msgtype)
            a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
            frames[i] = a
        i += 1
        if i > max(want):
            break

means = {k: float(v.mean()) for k, v in frames.items()}
print("frames:", sorted(frames), "means:", {k: round(v, 1) for k, v in means.items()})

fig, axes = plt.subplots(2, 3, figsize=(15, 6))
fig.suptitle("ARGUS — Drone camera POV flying through the GPS-denied warehouse corridor",
             fontweight="bold", fontsize=14)
for ax, idx in zip(axes.flat, want):
    if idx in frames:
        ax.imshow(frames[idx])
        ax.set_title(f"t ≈ {idx/13.2:.0f}s", fontsize=10)
    ax.axis("off")
plt.tight_layout()
plt.savefig(OUT, dpi=110, bbox_inches="tight")
print("saved", OUT)
