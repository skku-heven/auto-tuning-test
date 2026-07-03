#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""
reset_ego.py — teleport the MORAI ego to the start of a path (auto-reset for
tuning, instead of pressing I + driving to the start every iteration).

Publishes morai_msgs/MultiEgoSetting to /ego_setting. MORAI must be subscribed
to /ego_setting (add a "MultiEgoSetting" subscriber in the sim Network Settings).

  python3 reset_ego.py --csv ../paths/kcity_2025.csv          # teleport to path start
  python3 reset_ego.py --x 7.58 --y -279.08 --yaw 61          # explicit pose
"""
from __future__ import annotations
import argparse, math, os, sys


def _core():
    try:
        import rospkg
        p = os.path.join(rospkg.RosPack().get_path("ad_autotune"), "scripts")
    except Exception:
        p = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, p)
    import autotune_core
    return autotune_core


def start_pose_from_csv(path):
    core = _core()
    pts = core.load_track_csv(path)
    x0, y0 = pts[0]
    x1, y1 = pts[1]
    yaw = math.degrees(math.atan2(y1 - y0, x1 - x0))
    # z from the raw csv 3rd column if present (≈28.5 for K-City)
    z = 28.5
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                c = line.split(",")
                if len(c) >= 3:
                    z = float(c[2])
                break
    return x0, y0, z, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=None)
    ap.add_argument("--x", type=float); ap.add_argument("--y", type=float)
    ap.add_argument("--z", type=float, default=28.5); ap.add_argument("--yaw", type=float, default=0.0)
    args = ap.parse_args()

    if args.csv:
        x, y, z, yaw = start_pose_from_csv(args.csv)
    else:
        x, y, z, yaw = args.x, args.y, args.z, args.yaw

    import rospy
    from morai_msgs.msg import MultiEgoSetting
    rospy.init_node("reset_ego", anonymous=True)
    pub = rospy.Publisher("/ego_setting", MultiEgoSetting, queue_size=1)
    # wait for the rosbridge subscriber to actually connect (else msgs are lost)
    t0 = rospy.get_time()
    while pub.get_num_connections() == 0 and rospy.get_time() - t0 < 5.0:
        rospy.sleep(0.1)
    if pub.get_num_connections() == 0:
        print("[warn] /ego_setting 구독자 없음 — MORAI Simulator Network에서 "
              "MultiEgoSetting 토글 ON 후 Connect 필요")

    m = MultiEgoSetting()
    m.number_of_ego_vehicle = 1
    m.camera_index = 0
    m.ego_index = [0]
    m.global_position_x = [x]
    m.global_position_y = [y]
    m.global_position_z = [z]
    m.global_roll = [0.0]; m.global_pitch = [0.0]; m.global_yaw = [yaw]
    m.velocity = [0.0]
    m.gear = [4]            # Drive
    m.ctrl_mode = [16]      # auto (accepts external CtrlCmd)
    m.steering_angle = [0.0]; m.vehicle_speed = [0.0]
    m.turn_signal = [0]; m.brake_light = [False]

    for _ in range(5):
        pub.publish(m); rospy.sleep(0.1)
    print(f"teleported ego -> ({x:.2f}, {y:.2f}, z={z:.1f}, yaw={yaw:.1f} deg)")


if __name__ == "__main__":
    main()
