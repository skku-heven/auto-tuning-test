#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""
auto_tune_live.py — unattended live tuning against MORAI (Optuna TPE).

Each trial: reset ego -> GovernedStanley drive over a segment (s=0..SEG_END)
-> competition-style score -> objective. SQLite storage (resumable), per-trial
metrics/log saved. Uses the shared corrected controller (ad_control).

  python3 auto_tune_live.py --hours 3 --seg 400 --timeout 120
Results in results/live_tune/ .
"""
from __future__ import annotations
import os, sys, math, time, json, argparse


def _mods():
    try:
        import rospkg
        p = os.path.join(rospkg.RosPack().get_path("ad_autotune"), "scripts")
    except Exception:
        p = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, p)
    import autotune_core, ad_control
    return autotune_core, ad_control

core, ctrl = _mods()
HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "..", "paths", "kcity_2025.csv")
OUT = os.path.join(HERE, "..", "results", "live_tune")
START = (7.5766, -279.0828, 28.5, 60.9)      # x,y,z,yaw(deg) = path[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hours", type=float, default=3.0)
    ap.add_argument("--seg", type=float, default=400.0)
    ap.add_argument("--timeout", type=float, default=120.0)
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    import rospy
    from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, MultiEgoSetting, EventInfo
    from morai_msgs.srv import MoraiEventCmdSrv
    import optuna

    track = core.load_track_csv(CSV); n = len(track)
    # arc-length s (shared across trials); profile is rebuilt per-trial (a_lat varies)
    s_arr = [0.0]
    for i in range(1, n):
        s_arr.append(s_arr[-1] + math.hypot(track[i][0]-track[i-1][0], track[i][1]-track[i-1][1]))

    rospy.init_node("auto_tune_live", anonymous=True)
    pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
    egopub = rospy.Publisher("/ego_setting", MultiEgoSetting, queue_size=1)
    st = {"x": None, "y": None, "th": 0.0, "v": 0.0, "t": 0.0}
    def cb(m):
        st["x"] = m.position.x; st["y"] = m.position.y
        st["th"] = math.radians(m.heading); st["v"] = math.hypot(m.velocity.x, m.velocity.y)*3.6
        st["t"] = rospy.get_time()
    rospy.Subscriber("/Ego_topic", EgoVehicleStatus, cb, queue_size=10)
    try:
        rospy.wait_for_service("/Service_MoraiEventCmd", timeout=5)
        event_srv = rospy.ServiceProxy("/Service_MoraiEventCmd", MoraiEventCmdSrv)
    except Exception:
        event_srv = None

    def fresh():
        return st["x"] is not None and (rospy.get_time() - st["t"]) < 0.5

    def reset_and_arm():
        m = MultiEgoSetting()
        m.number_of_ego_vehicle = 1; m.camera_index = 0; m.ego_index = [0]
        m.global_position_x = [START[0]]; m.global_position_y = [START[1]]; m.global_position_z = [START[2]]
        m.global_roll = [0.0]; m.global_pitch = [0.0]; m.global_yaw = [START[3]]
        m.velocity = [0.0]; m.gear = [4]; m.ctrl_mode = [16]
        m.steering_angle = [0.0]; m.vehicle_speed = [0.0]; m.turn_signal = [0]; m.brake_light = [False]
        t0 = rospy.get_time()
        while egopub.get_num_connections() == 0 and rospy.get_time()-t0 < 5: rospy.sleep(0.1)
        for _ in range(8): egopub.publish(m); rospy.sleep(0.1)
        t0 = rospy.get_time()
        while rospy.get_time()-t0 < 8:
            if fresh() and math.hypot(st["x"]-START[0], st["y"]-START[1]) < 3.0: break
            rospy.sleep(0.1)
        else:
            return False
        if event_srv:
            try:
                ev = EventInfo(); ev.option = 3; ev.ctrl_mode = 3; ev.gear = 4; event_srv(ev)
            except Exception: pass
        rospy.sleep(0.5)
        return True

    def drive(p, save_path):
        s, prof = ctrl.build_profile(track, a_lat=p["a_lat"])
        gs = ctrl.GovernedStanley(track, s, prof, p["lookahead"], p["gain_k"], p["k_soft"], p["pid_kp"])
        gs.reset(st["x"], st["y"])
        max_s = 0.0; max_cte = sum_cte = 0.0; cnt = 0
        offtrack = overspeed = 0.0; diverged = False; complete_t = None
        stall_t = None; stale_since = None; samples = []
        rate = rospy.Rate(20); t0 = rospy.get_time(); prev_t = t0
        while not rospy.is_shutdown():
            now = rospy.get_time(); t = now-t0; dt = now-prev_t; prev_t = now
            if t > args.timeout: break
            if not fresh():
                stale_since = stale_since if stale_since else now
                if now - stale_since > 3.0: return None     # SUSTAINED stale = disconnected
                rospy.sleep(0.05); continue
            stale_since = None
            x, y, th, v = st["x"], st["y"], st["th"], st["v"]
            target = min(p["target_velocity_kph"], 5.0 + (p["target_velocity_kph"]-5.0)*min(t/4.0, 1.0))
            steer, thr, brk, info = gs.step(x, y, th, v, dt, target)
            cmd = CtrlCmd(); cmd.longlCmdType = 1
            cmd.accel = float(thr); cmd.brake = float(brk)
            cmd.steering = float(max(-1.0, min(1.0, steer/0.698))); cmd.front_steer = float(steer)
            pub.publish(cmd)
            cte = core._path_lateral_error(track, x, y)
            near = info["near"]
            max_s = max(max_s, s_arr[near]); max_cte = max(max_cte, cte); sum_cte += cte; cnt += 1
            if cte > 1.5: offtrack += dt
            if v > 50: overspeed += dt
            samples.append((round(t, 2), round(x, 2), round(y, 2), round(v, 1), round(cte, 2), near))
            # stall abort (no forward progress + near-zero speed)
            if v < 1.0 and t > 5:
                stall_t = (stall_t or now)
                if now - stall_t > 4.0: diverged = True; break
            else:
                stall_t = None
            if cte > 12: diverged = True; break
            if s_arr[near] >= args.seg: complete_t = t; break
            rate.sleep()
        stop = CtrlCmd(); stop.longlCmdType = 1; stop.brake = 1.0
        for _ in range(4): pub.publish(stop); rospy.sleep(0.02)
        completed = complete_t is not None
        penalty = 5.0*int(offtrack//3.0) + (10.0 if overspeed > 0 else 0) + 10.0*int(overspeed//3.0)
        m = dict(completed=completed, diverged=diverged, progress_s=round(max_s, 1),
                 time_s=round(complete_t, 2) if completed else args.timeout,
                 driving_score=max(0.0, 100.0-penalty), max_cte=round(max_cte, 2),
                 mean_cte=round(sum_cte/max(cnt, 1), 3), offtrack_s=round(offtrack, 1),
                 overspeed_s=round(overspeed, 1))
        if save_path:
            with open(save_path, "w") as f:
                f.write("t,x,y,v_kph,cte,near\n")
                for s_ in samples: f.write(",".join(map(str, s_))+"\n")
        return m

    def objective(trial):
        p = dict(lookahead=trial.suggest_float("lookahead", 1.5, 5.5),
                 target_velocity_kph=trial.suggest_float("target_velocity_kph", 12.0, 50.0),
                 gain_k=trial.suggest_float("gain_k", 0.4, 3.0, log=True),
                 k_soft=trial.suggest_float("k_soft", 0.5, 3.0),
                 a_lat=trial.suggest_float("a_lat", 1.0, 3.0),
                 pid_kp=0.3)
        for k, v in p.items(): trial.set_user_attr(k, v)
        trial.set_user_attr("seg", args.seg)
        if not reset_and_arm():
            trial.set_user_attr("fail", "RESET_FAILED"); return -5e6
        m = drive(p, os.path.join(OUT, f"trial_{trial.number:04d}.csv"))
        if m is None:
            trial.set_user_attr("fail", "DISCONNECTED"); return -4e6
        for k, v in m.items(): trial.set_user_attr(k, v)
        with open(os.path.join(OUT, f"trial_{trial.number:04d}.json"), "w") as f:
            json.dump({**p, **m}, f)
        if m["completed"]:
            return 1e6 + m["driving_score"]*1000 - m["time_s"]
        return -1e6 + m["progress_s"]*1e3 - m["max_cte"]*50

    study = optuna.create_study(direction="maximize",
                                storage=f"sqlite:///{os.path.join(OUT, 'tune.db')}",
                                study_name=f"kcity_seg{int(args.seg)}_v3", load_if_exists=True,
                                sampler=optuna.samplers.TPESampler(seed=20260701,
                                    n_startup_trials=10, multivariate=True, group=True))
    deadline = time.time() + args.hours*3600
    print(f"[auto_tune] seg={args.seg}m timeout={args.timeout}s deadline {args.hours}h (5 params)", flush=True)
    while time.time() < deadline and not rospy.is_shutdown():
        study.optimize(objective, n_trials=1, catch=(Exception,))
        b = study.best_trial
        print(f"[auto_tune] {len(study.trials)} trials. BEST #{b.number} obj={b.value:.0f} "
              f"{ {k: round(b.user_attrs.get(k), 2) if isinstance(b.user_attrs.get(k), float) else b.user_attrs.get(k) for k in ('completed','time_s','max_cte','lookahead','target_velocity_kph','gain_k','k_soft','a_lat')} }", flush=True)
    b = study.best_trial
    with open(os.path.join(OUT, "best.json"), "w") as f:
        json.dump({"value": b.value, **b.user_attrs}, f, indent=2)
    print(f"[auto_tune] DONE. {len(study.trials)} trials. best obj={b.value:.0f}", flush=True)


if __name__ == "__main__":
    main()
