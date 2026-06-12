#!/usr/bin/env python3
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS2_HUMBLE)
bag = Path("/home/vittal/argus/data/bags/map_demo")
for topic in ["/argus/vio/point_cloud", "/argus/vio/margin_cloud"]:
    with AnyReader([bag], default_typestore=TS) as r:
        conns = [c for c in r.connections if c.topic == topic]
        counts = []
        for conn, ts, raw in r.messages(connections=conns):
            m = r.deserialize(raw, conn.msgtype)
            counts.append(len(m.points))
        if counts:
            nz = [c for c in counts if c > 0]
            print(f"{topic}: {len(counts)} msgs | nonempty {len(nz)} | "
                  f"pts min/med/max = {min(counts)}/{int(np.median(counts))}/{max(counts)} | "
                  f"last={counts[-1]}")
        else:
            print(f"{topic}: NO messages")
