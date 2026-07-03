#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""
ad_autotune — LIVE tuning against the running MORAI simulator.

Same optimizer + scoring as the offline harness (autotune_core.score_run), but
the "plant" is the real sim:  each trial relaunches ad_tracker with a param set,
records the ego pose from /Competition_topic, scores the run, and moves on.

  # works RIGHT NOW without the sim (uses the offline bicycle plant):
  python3 run_live_tuning.py --dry-run

  # live, once MORAI Ego Network (ROS) is Connected and topics flow:
  rosrun ad_autotune run_live_tuning.py --path $(rospack find ad_tracker)/csv/global_path.csv

PREREQS for live mode (all verified present except the Connect press):
  - roscore + rosbridge_websocket up (ws://127.0.0.1:9090)
  - MORAI Ego Network = ROS, Connected  -> /Competition_topic, /ctrl_cmd flowing
  - catkin build done so `rosrun ad_tracker ad_tracker` exists

THE ONE UNVERIFIED PIECE — ego reset between trials.
  SimControl (:20000) is dead on this competition build, so there is no scripted
  scenario reset.  Options, in order of preference:
    1. /Service_MoraiEventCmd  (morai_msgs/MoraiEventCmdSrv) — set AutoMode; some
       builds also reposition. Tried automatically below (best-effort).
    2. --manual-reset : pause between trials so you reload the scenario by hand.
  Confirm which works on your build; the rest of the loop is build-agnostic.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

import autotune_core as core

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "..", "results")

# small, sensible sweep for a live run (each trial costs a real lap, so keep it tight)
SWEEP = [
    dict(lookahead=3.0, target_velocity_kph=20.0, gain_k=0.5, pid_kp=0.3),   # baseline
    dict(lookahead=4.0, target_velocity_kph=30.0, gain_k=0.8, pid_kp=0.4),
    dict(lookahead=5.0, target_velocity_kph=40.0, gain_k=1.0, pid_kp=0.5),
    dict(lookahead=6.0, target_velocity_kph=45.0, gain_k=1.5, pid_kp=0.5),
]
FIXED = dict(pid_ki=0.0, pid_kd=0.05)


# ---------------------------------------------------------------------------
# DRY RUN — reuse the offline plant so the whole loop is exercisable now.
# ---------------------------------------------------------------------------
def run_trial_offline(track, params):
    return core.simulate(track, params)


# ---------------------------------------------------------------------------
# LIVE — record ego from ROS while the real ad_tracker drives.
# ---------------------------------------------------------------------------
def run_trial_live(track, params, args, rospy, EgoMsg):
    samples = []
    t0 = [None]

    def cb(msg):
        if t0[0] is None:
            t0[0] = rospy.get_time()
        t = rospy.get_time() - t0[0]
        v_kph = (msg.velocity.x ** 2 + msg.velocity.y ** 2) ** 0.5 * 3.6
        samples.append((t, msg.position.x, msg.position.y, v_kph))

    reset_ego(args, rospy)
    sub = rospy.Subscriber("/Competition_topic", EgoMsg, cb, queue_size=20)

    # launch the real controller with this param set
    launch = ["roslaunch", "ad_tracker", "ad_tracker.launch",
              f"lookahead:={params['lookahead']}",
              f"target_velocity_kph:={params['target_velocity_kph']}",
              f"gain_k:={params['gain_k']}",
              f"pid_kp:={params['pid_kp']}",
              f"pid_ki:={params['pid_ki']}",
              f"pid_kd:={params['pid_kd']}"]
    proc = subprocess.Popen(launch, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    deadline = time.time() + core.TIME_LIMIT_S
    n = len(track)
    try:
        while time.time() < deadline and not rospy.is_shutdown():
            # stop early once a full lap of progress is recorded
            if len(samples) > 5:
                r = core.score_run(track, samples)
                if r["completed"] or r["diverged"]:
                    break
            time.sleep(0.2)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        sub.unregister()

    return core.score_run(track, samples)


def reset_ego(args, rospy):
    """Best-effort ego reset between trials. See module docstring."""
    if args.manual_reset:
        input("  [manual] 시나리오 리셋(또는 ego 시작점 복귀) 후 Enter...")
        return
    try:
        from morai_msgs.srv import MoraiEventCmdSrv
        from morai_msgs.msg import EventInfo
        rospy.wait_for_service("/Service_MoraiEventCmd", timeout=3.0)
        srv = rospy.ServiceProxy("/Service_MoraiEventCmd", MoraiEventCmdSrv)
        ev = EventInfo()
        ev.option = 3          # ctrl_mode + gear
        ev.ctrl_mode = 3       # 3 = AutoMode (auto driving)
        ev.gear = 4            # D
        srv(ev)
        time.sleep(1.0)
    except Exception as e:
        print(f"  [reset] /Service_MoraiEventCmd 실패({e}); 위치 리셋은 수동 필요할 수 있음")
    time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="offline plant, no ROS/sim")
    ap.add_argument("--manual-reset", action="store_true", help="pause for manual reset")
    ap.add_argument("--path", default=os.path.join(HERE, "..", "paths", "oval_track.csv"))
    args = ap.parse_args()

    if os.path.exists(args.path):
        track = core.load_track_csv(args.path)
    else:
        track = core.generate_oval_track()
    print(f"track: {len(track)} waypoints from {os.path.basename(args.path)}")

    rospy = EgoMsg = None
    if not args.dry_run:
        try:
            import rospy as _rospy
            from morai_msgs.msg import EgoVehicleStatus as _Ego
            rospy, EgoMsg = _rospy, _Ego
            rospy.init_node("ad_autotune_live", anonymous=True)
        except Exception as e:
            print(f"[error] ROS/morai_msgs import 실패: {e}\n  --dry-run 으로 먼저 확인하세요.")
            sys.exit(1)

    best = None
    os.makedirs(RESULTS, exist_ok=True)
    log = os.path.join(RESULTS, "live_trials.csv")
    with open(log, "w") as f:
        f.write("trial,lookahead,target_velocity_kph,gain_k,pid_kp,"
                "completed,driving_score,time_s,max_cte,objective\n")
        for i, sweep in enumerate(SWEEP):
            params = dict(sweep, **FIXED)
            print(f"\n[trial {i}] {sweep}")
            if args.dry_run:
                r = run_trial_offline(track, params)
            else:
                r = run_trial_live(track, params, args, rospy, EgoMsg)
            obj = core.objective(r)
            print(f"  -> done={int(r['completed'])} score={r['driving_score']:.0f} "
                  f"t={r['time_s']:.1f}s max_cte={r['max_cte']} obj={obj:.1f}")
            f.write(f"{i},{params['lookahead']},{params['target_velocity_kph']},"
                    f"{params['gain_k']},{params['pid_kp']},{int(r['completed'])},"
                    f"{r['driving_score']:.1f},{r['time_s']:.2f},{r['max_cte']},{obj:.1f}\n")
            if best is None or obj > best[0]:
                best = (obj, params, r)

    _, bp, br = best
    print("\n" + "=" * 56)
    print(f"BEST: LA={bp['lookahead']} V={bp['target_velocity_kph']} "
          f"k={bp['gain_k']} kp={bp['pid_kp']} "
          f"| score={br['driving_score']:.0f} t={br['time_s']:.1f}s")
    print(f"log: {log}")


if __name__ == "__main__":
    main()
