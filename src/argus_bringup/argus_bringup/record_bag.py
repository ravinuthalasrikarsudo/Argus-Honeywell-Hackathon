#!/usr/bin/env python3
"""ARGUS record_bag helper.

Thin wrapper around ``ros2 bag record`` that captures the frozen ``/argus/*``
contract topics (plus ``/clock``) into a timestamped bag, giving the downstream
pillars a reproducible stereo+IMU+ground-truth dataset to replay offline.

``/clock`` is recorded so the bag replays correctly with ``use_sim_time`` (run
``ros2 bag play <bag> --clock`` on playback). ``/argus/cmd_vel`` is included so a
drive session is captured end to end.

Run::

    ros2 run argus_bringup record_bag                 # default contract topics
    ros2 run argus_bringup record_bag --duration 30   # stop after 30 s
    ros2 run argus_bringup record_bag --all            # everything on the graph

Stops on Ctrl-C or after ``--duration`` seconds; either way the child is sent
SIGINT so rosbag2 closes the bag cleanly (a hard kill can corrupt the metadata).
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

# Frozen-contract topic set (matches argus_bridge.yaml + the camera_info patch).
DEFAULT_TOPICS = [
    '/argus/cam0/image_raw',
    '/argus/cam0/camera_info',
    '/argus/cam1/image_raw',
    '/argus/cam1/camera_info',
    '/argus/imu',
    '/argus/ground_truth/pose',
    '/argus/cmd_vel',
    '/clock',
    '/argus/clock',
]


def _parse(argv):
    p = argparse.ArgumentParser(
        description='Record the frozen ARGUS /argus/* contract topics to a rosbag2 bag.')
    p.add_argument('-o', '--output', default=None,
                   help='Output bag dir (default: ~/argus/bags/argus_<timestamp>).')
    p.add_argument('-a', '--all', action='store_true',
                   help='Record every topic on the graph instead of the contract set.')
    p.add_argument('-d', '--duration', type=float, default=None,
                   help='Stop after N seconds (default: run until Ctrl-C).')
    p.add_argument('--topics', nargs='+', default=None,
                   help='Explicit topic list, overrides the default contract set.')
    p.add_argument('-s', '--storage', default='sqlite3',
                   help='rosbag2 storage plugin (default sqlite3; mcap if installed).')
    return p.parse_args(argv)


def main(argv=None):
    args = _parse(sys.argv[1:] if argv is None else argv)

    out = args.output
    if out is None:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out = os.path.expanduser(f'~/argus/bags/argus_{stamp}')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)

    cmd = ['ros2', 'bag', 'record', '-s', args.storage, '-o', out]
    if args.all:
        cmd.append('-a')
        selection = 'ALL topics'
    else:
        topics = args.topics if args.topics else DEFAULT_TOPICS
        cmd += topics
        selection = f'{len(topics)} contract topics'

    dur = f' for {args.duration:.0f}s' if args.duration else ' (Ctrl-C to stop)'
    print(f'[record_bag] recording {selection}{dur}\n[record_bag] -> {out}', flush=True)

    # Own process group so we can SIGINT the whole rosbag2 tree at once.
    proc = subprocess.Popen(cmd, start_new_session=True)
    try:
        if args.duration:
            deadline = time.monotonic() + args.duration
            while proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.2)
        else:
            proc.wait()
    except KeyboardInterrupt:
        pass
    finally:
        if proc.poll() is None:
            # SIGINT lets rosbag2 finalize metadata.yaml; hard kill can corrupt it.
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

    print(f'[record_bag] saved bag: {out}')
    return proc.returncode or 0


if __name__ == '__main__':
    sys.exit(main())
