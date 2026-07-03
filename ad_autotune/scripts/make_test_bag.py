#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""
make_test_bag.py — write a rosbag from a simulated run, so the ROS metric node
(ad_metric/competition_score.py) can be validated WITHOUT the live sim.

  source devel/setup.bash
  python3 make_test_bag.py --track oval --out /tmp/test_run.bag
  rosrun ad_metric competition_score.py --bag /tmp/test_run.bag \
      --csv ../paths/oval_track.csv
"""
from __future__ import annotations

import argparse
import os
import sys


def _import_core():
    """Always import autotune_core from the SOURCE package dir (rosrun runs a
    stale devel copy otherwise)."""
    try:
        import rospkg
        p = os.path.join(rospkg.RosPack().get_path("ad_autotune"), "scripts")
    except Exception:
        p = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, p)
    import autotune_core
    return autotune_core


core = _import_core()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", default="oval", choices=list(core.TRACKS))
    ap.add_argument("--out", default="/tmp/test_run.bag")
    ap.add_argument("--lookahead", type=float, default=4.0)
    ap.add_argument("--velocity", type=float, default=40.0)
    ap.add_argument("--gain", type=float, default=0.8)
    args = ap.parse_args()

    import rosbag
    from rospy import Time
    from morai_msgs.msg import EgoVehicleStatus

    track = core.TRACKS[args.track]()
    params = dict(lookahead=args.lookahead, target_velocity_kph=args.velocity,
                  gain_k=args.gain, pid_kp=0.5, pid_ki=0.0, pid_kd=0.05)
    r = core.simulate(track, params, record=True)
    print(f"simulated {args.track}: done={r['completed']} score={r['driving_score']} "
          f"t={r['time_s']}s samples={len(r['traj'])}")

    base = 1000.0
    with rosbag.Bag(args.out, "w") as bag:
        for (t, x, y, v_kph) in r["traj"]:
            msg = EgoVehicleStatus()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = 0.0
            msg.velocity.x = v_kph / 3.6     # m/s forward
            stamp = Time.from_sec(base + t)
            bag.write("/Competition_topic", msg, stamp)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
