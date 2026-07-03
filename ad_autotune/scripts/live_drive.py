#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""
live_drive.py — drive the MORAI ego along a path using ad_control.GovernedStanley.
Thin wrapper: pose from /Ego_topic -> GovernedStanley -> /ctrl_cmd. For manual
verification/debug. Engages External mode. Freshness + start ramp + logging.

  python3 live_drive.py --csv ../paths/kcity_2025.csv --velocity 25 --log /tmp/run
"""
from __future__ import annotations
import argparse, math, os, sys


def _mods():
    try:
        import rospkg
        p = os.path.join(rospkg.RosPack().get_path("ad_autotune"), "scripts")
    except Exception:
        p = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, p)
    import autotune_core, ad_control
    return autotune_core, ad_control


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--velocity", type=float, default=25.0, help="straight target kph")
    ap.add_argument("--lookahead", type=float, default=3.0)
    ap.add_argument("--gain", type=float, default=0.8)
    ap.add_argument("--ksoft", type=float, default=1.0)
    ap.add_argument("--kp", type=float, default=0.3)
    ap.add_argument("--a-lat", type=float, default=1.5)
    ap.add_argument("--log", default=None)
    args = ap.parse_args()

    core, ctrl = _mods()
    import rospy
    from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

    track = core.load_track_csv(args.csv); n = len(track)
    s, prof = ctrl.build_profile(track, a_lat=args.a_lat)
    gs = ctrl.GovernedStanley(track, s, prof, args.lookahead, args.gain, args.ksoft, args.kp)

    rospy.init_node("ad_live_drive", anonymous=True)
    pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
    st = {"x": None, "y": None, "th": 0.0, "v": 0.0, "t": 0.0}
    def cb(m):
        st["x"] = m.position.x; st["y"] = m.position.y; st["th"] = math.radians(m.heading)
        st["v"] = math.hypot(m.velocity.x, m.velocity.y)*3.6; st["t"] = rospy.get_time()
    rospy.Subscriber("/Ego_topic", EgoVehicleStatus, cb, queue_size=10)
    try:
        from morai_msgs.msg import EventInfo
        from morai_msgs.srv import MoraiEventCmdSrv
        rospy.wait_for_service("/Service_MoraiEventCmd", timeout=3.0)
        ev = EventInfo(); ev.option = 3; ev.ctrl_mode = 3; ev.gear = 4
        rospy.ServiceProxy("/Service_MoraiEventCmd", MoraiEventCmdSrv)(ev)
    except Exception as e:
        rospy.logwarn(f"mode set failed ({e})")
    while st["x"] is None and not rospy.is_shutdown():
        rospy.sleep(0.1)
    gs.reset(st["x"], st["y"])

    logf = None
    if args.log:
        os.makedirs(args.log, exist_ok=True)
        logf = open(os.path.join(args.log, "controller.csv"), "w")
        logf.write("t,x,y,heading_deg,s,near,cte,head_err_deg,v_ref,v,steer_norm,accel\n")

    rate = rospy.Rate(20); t0 = rospy.get_time(); prev_t = t0
    rospy.loginfo("driving. Ctrl+C to stop.")
    try:
        while not rospy.is_shutdown():
            now = rospy.get_time(); t = now-t0; dt = now-prev_t; prev_t = now
            if now - st["t"] > 0.5:                       # freshness / stall guard
                rospy.logwarn_throttle(1.0, "stale /Ego_topic")
                rospy.sleep(0.05); continue
            x, y, th, v = st["x"], st["y"], st["th"], st["v"]
            target = min(args.velocity, 5.0 + (args.velocity-5.0)*min(t/4.0, 1.0))  # start ramp
            steer, thr, brk, info = gs.step(x, y, th, v, dt, target)
            cmd = CtrlCmd(); cmd.longlCmdType = 1
            cmd.accel = float(thr); cmd.brake = float(brk)
            cmd.steering = float(max(-1.0, min(1.0, steer/0.698))); cmd.front_steer = float(steer)
            pub.publish(cmd)
            cte_abs = core._path_lateral_error(track, x, y)
            if logf:
                logf.write(f"{t:.2f},{x:.2f},{y:.2f},{math.degrees(th):.1f},{info['s']:.1f},"
                           f"{info['near']},{info['cte']:.2f},{math.degrees(info['head_err']):.1f},"
                           f"{info['v_ref']:.1f},{v:.1f},{cmd.steering:.3f},{thr:.2f}\n")
            rospy.loginfo_throttle(1.0,
                f"s={info['s']:.0f} near={info['near']} cte={cte_abs:.2f} "
                f"hErr={math.degrees(info['head_err']):.0f} vref={info['v_ref']:.0f} "
                f"v={v:.0f} steer={math.degrees(steer):.0f}deg")
            if info["near"] >= n-3:
                rospy.loginfo("REACHED END s=%.0f", info["s"]); break
            if cte_abs > 15.0:
                rospy.logwarn("DIVERGED cte=%.0f s=%.0f near=%d", cte_abs, info["s"], info["near"]); break
            rate.sleep()
    finally:
        if logf: logf.close()
        stop = CtrlCmd(); stop.longlCmdType = 1; stop.brake = 1.0
        for _ in range(5): pub.publish(stop); rospy.sleep(0.02)
        print("stopped.")


if __name__ == "__main__":
    main()
