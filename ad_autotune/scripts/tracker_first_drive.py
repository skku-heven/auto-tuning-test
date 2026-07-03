#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""tracker_first_drive.py — ego 리셋+External 모드 후 ad_tracker가 주행하는지 N초 관찰.

ad_tracker(C++) + ego_pose_bridge가 먼저 떠 있어야 함. 우리는 제어 안 하고(ad_tracker가 /ctrl_cmd 발행)
/Ego_topic 위치·cte·수신한 /ctrl_cmd만 찍어서 path-tracking 되는지 눈으로 확인.
  python3 tracker_first_drive.py --secs 30
"""
import math, os, sys, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune_core as core


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(os.path.dirname(__file__), "..", "paths", "kcity_2025.csv"))
    ap.add_argument("--secs", type=float, default=30)
    args = ap.parse_args()

    import rospy
    from morai_msgs.msg import EgoVehicleStatus, MultiEgoSetting, CtrlCmd, EventInfo
    from morai_msgs.srv import MoraiEventCmdSrv

    track = core.load_track_csv(args.csv)
    START = (7.5766, -279.0828, 28.5, 60.9)
    rospy.init_node("tracker_first_drive", anonymous=True)

    st = {"x": None, "y": None, "v": 0.0}
    def cb(m):
        st["x"] = m.position.x; st["y"] = m.position.y
        st["v"] = math.hypot(m.velocity.x, m.velocity.y) * 3.6
    rospy.Subscriber("/Ego_topic", EgoVehicleStatus, cb, queue_size=10)
    cmd = {"steer": 0.0, "accel": 0.0, "n": 0}
    def ccb(m):
        cmd["steer"] = m.steering; cmd["accel"] = m.accel; cmd["n"] += 1
    rospy.Subscriber("/ctrl_cmd", CtrlCmd, ccb, queue_size=10)

    egopub = rospy.Publisher("/ego_setting", MultiEgoSetting, queue_size=1)
    m = MultiEgoSetting()
    m.number_of_ego_vehicle = 1; m.camera_index = 0; m.ego_index = [0]
    m.global_position_x = [START[0]]; m.global_position_y = [START[1]]; m.global_position_z = [START[2]]
    m.global_roll = [0.0]; m.global_pitch = [0.0]; m.global_yaw = [START[3]]
    m.velocity = [0.0]; m.gear = [4]; m.ctrl_mode = [16]
    m.steering_angle = [0.0]; m.vehicle_speed = [0.0]; m.turn_signal = [0]; m.brake_light = [False]
    t0 = rospy.get_time()
    while egopub.get_num_connections() == 0 and rospy.get_time() - t0 < 5:
        rospy.sleep(0.1)
    for _ in range(8):
        egopub.publish(m); rospy.sleep(0.1)
    rospy.sleep(1.0)
    try:
        rospy.wait_for_service("/Service_MoraiEventCmd", timeout=5)
        ev = EventInfo(); ev.option = 3; ev.ctrl_mode = 3; ev.gear = 4
        rospy.ServiceProxy("/Service_MoraiEventCmd", MoraiEventCmdSrv)(ev)
        print("[first_drive] External mode set")
    except Exception as e:
        print(f"[first_drive] External mode fail: {e}")
    rospy.sleep(0.5)

    print("[first_drive] 관찰 시작 (ad_tracker가 /ctrl_cmd 발행해야 함)")
    t0 = rospy.get_time(); rate = rospy.Rate(2)
    while not rospy.is_shutdown() and rospy.get_time() - t0 < args.secs:
        if st["x"] is not None:
            cte = core._path_lateral_error(track, st["x"], st["y"])
            print(f"t={rospy.get_time()-t0:4.1f} x={st['x']:7.1f} y={st['y']:7.1f} "
                  f"v={st['v']:4.1f}kph cte={cte:5.2f} | /ctrl_cmd steer={cmd['steer']:+.2f} "
                  f"accel={cmd['accel']:.2f} (n={cmd['n']})", flush=True)
        rate.sleep()
    print("[first_drive] 끝")


if __name__ == "__main__":
    main()
