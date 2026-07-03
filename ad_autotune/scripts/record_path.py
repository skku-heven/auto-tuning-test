#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""
record_path.py — record a global_path.csv by driving the course.

The last-year scenario is on the OLD map frame, so its csv/scenario don't match
R_KR_PR_K-city_2025.  Fix: drive the new map once and record a fresh path here.

  # LIVE: manually drive the course in MORAI, then Ctrl+C to save
  rosrun ad_autotune record_path.py --out $(rospack find ad_tracker)/csv/global_path.csv

  # OFFLINE: extract a path from an existing bag (also how it's tested)
  rosrun ad_autotune record_path.py --bag run.bag --out /tmp/path.csv

Output is 2 columns (x,y) on purpose — ad_tracker derives heading from
consecutive waypoints, which sidesteps bug A (col3 read as heading).
"""
from __future__ import annotations

import argparse
import math
import os
import sys

# /Ego_topic has real ground-truth position; /Competition_topic position is
# competition-zeroed (use GPS/IMU localization for the real competition run).
TOPIC = "/Ego_topic"


def _smooth(pts, window):
    """Moving-average smoothing to remove manual-driving wobble. window=odd."""
    if window < 3 or len(pts) < window:
        return pts
    h = window // 2
    out = []
    n = len(pts)
    for i in range(n):
        xs = ys = 0.0
        cnt = 0
        for j in range(i - h, i + h + 1):
            k = j % n          # wrap (closed loop); for open paths the ends average less
            xs += pts[k][0]; ys += pts[k][1]; cnt += 1
        out.append((xs / cnt, ys / cnt))
    return out


class Downsampler:
    """Keep a waypoint only when it's > spacing from the last kept one."""
    def __init__(self, spacing, smooth=0):
        self.spacing = spacing
        self.smooth = smooth
        self.pts = []

    def add(self, x, y):
        if not self.pts or math.hypot(x - self.pts[-1][0], y - self.pts[-1][1]) >= self.spacing:
            self.pts.append((x, y))

    def write(self, path):
        if self.smooth:
            self.pts = _smooth(self.pts, self.smooth)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            f.write("# x,y  recorded global path\n")
            for x, y in self.pts:
                f.write(f"{x:.4f},{y:.4f}\n")
        length = sum(math.hypot(self.pts[i][0] - self.pts[i - 1][0],
                                self.pts[i][1] - self.pts[i - 1][1])
                     for i in range(1, len(self.pts)))
        print(f"wrote {len(self.pts)} waypoints (~{length:.1f} m) -> {path}")


def from_bag(args, ds):
    import rosbag
    with rosbag.Bag(args.bag, "r") as bag:
        for _topic, msg, _ts in bag.read_messages(topics=[args.topic]):
            ds.add(msg.position.x, msg.position.y)
    ds.write(args.out)


def from_live(args, ds):
    import rospy
    from morai_msgs.msg import EgoVehicleStatus
    rospy.init_node("ad_path_recorder", anonymous=True)

    def cb(msg):
        ds.add(msg.position.x, msg.position.y)
        if len(ds.pts) % 20 == 0:
            sys.stdout.write(f"\r  recorded {len(ds.pts)} waypoints...")
            sys.stdout.flush()

    rospy.Subscriber(args.topic, EgoVehicleStatus, cb, queue_size=50)
    print(f"recording from {args.topic} — drive the course, Ctrl+C to save.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        print()
        ds.write(args.out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--bag", default=None, help="extract from a bag instead of live")
    ap.add_argument("--topic", default=TOPIC)
    ap.add_argument("--spacing", type=float, default=0.5, help="meters between waypoints")
    ap.add_argument("--smooth", type=int, default=0,
                    help="moving-average window (odd, e.g. 7) to remove driving wobble; 0=off")
    args = ap.parse_args()

    ds = Downsampler(args.spacing, smooth=args.smooth)
    if args.bag:
        from_bag(args, ds)
    else:
        from_live(args, ds)


if __name__ == "__main__":
    main()
