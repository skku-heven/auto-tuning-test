#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""
competition_score.py — score a recorded run by the 2025 대회 채점 규정
(완주 여부 > 주행점수[100−감점] > 완주시간), instead of raw RMSE/mean-speed.

This is the ROS/rosbag counterpart of ad_autotune's score_run — it imports the
SAME scoring so live tuning and post-hoc bag scoring agree.  Additive: leaves the
original calc_metric.py untouched.

  rosrun ad_metric competition_score.py --bag run.bag \
      --csv $(rospack find ad_tracker)/csv/global_path.csv

감점 (규정):
  경로이탈(전·후륜축 중심이 미션경로 최단거리 초과) 3초누적 -5
  속도 50kph 초과 즉시 -10, +3초누적당 -10
  정적장애물 충돌 -10 / 객체
  코스이탈(차선 이탈 지속) → 완주 무효
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

# reuse the harness scoring (single source of truth). Resolve the SOURCE
# ad_autotune/scripts via rospkg so this works from source AND via rosrun.
try:
    import rospkg
    _AUTOTUNE = os.path.join(rospkg.RosPack().get_path("ad_autotune"), "scripts")
except Exception:
    _AUTOTUNE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "..", "ad_autotune", "scripts")
sys.path.insert(0, _AUTOTUNE)
import autotune_core as core  # noqa: E402

COMPETITION_TOPIC = "/Ego_topic"  # real position; /Competition_topic is zeroed
COLLISION_TOPIC = "/CollisionData"
POSE_TOPIC = "/ad_pose_parser/pose"
COLLISION_PENALTY = 10.0   # 규정: 정적장애물 충돌 -10 / 객체


def collision_objects(msg):
    objs = []
    if hasattr(msg, "collision_object"):
        objs.extend(msg.collision_object)
    if hasattr(msg, "collision_objecta"):   # tolerate the typo'd field
        objs.extend(msg.collision_objecta)
    return objs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, type=Path)
    ap.add_argument("--csv", required=True, type=Path)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    import rosbag

    track = core.load_track_csv(str(args.csv))   # reads x,y (ignores col3) -> dodges bug A

    samples = []          # (t, x, y, v_kph)
    pose_xy = {}          # t -> (x,y) from POSE_TOPIC if available
    t0 = None
    collided_ids = set()

    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, msg, ts in bag.read_messages(
                topics=[COMPETITION_TOPIC, COLLISION_TOPIC, POSE_TOPIC]):
            t = ts.to_sec()
            if t0 is None:
                t0 = t
            rel = t - t0
            if topic == COMPETITION_TOPIC:
                v = math.hypot(getattr(msg.velocity, "x", 0.0),
                               getattr(msg.velocity, "y", 0.0)) * 3.6
                samples.append((rel, msg.position.x, msg.position.y, v))
            elif topic == POSE_TOPIC:
                pose_xy[round(rel, 2)] = (msg.x, msg.y)
            elif topic == COLLISION_TOPIC:
                for o in collision_objects(msg):
                    collided_ids.add(getattr(o, "unique_id", id(o)))

    if not samples:
        print("[error] /Competition_topic 샘플 없음 — bag/연동 확인", file=sys.stderr)
        return 1

    r = core.score_run(track, samples)

    # collision penalty (규정), applied on top of score_run's path/speed penalties
    n_collisions = len(collided_ids)
    base_penalty = 100.0 - r["driving_score"]
    total_penalty = base_penalty + COLLISION_PENALTY * n_collisions
    driving_score = max(0.0, 100.0 - total_penalty)

    lines = [
        "==== 대회 채점 (competition_score) ====",
        f"완주 여부      : {'O' if r['completed'] else 'X'}"
        + ("  (코스이탈 무효)" if r.get("off_road") else ""),
        f"주행 점수      : {driving_score:.1f} / 100",
        f"완주 시간      : {r['time_s']:.2f} s" if r["completed"] else "완주 시간      : - (미완주)",
        f"진행률         : {r['progress']*100:.1f} %",
        "---- 감점 내역 ----",
        f"경로이탈 누적  : {r['offtrack_time']:.1f} s  (-{5.0*int(r['offtrack_time']//3.0):.0f})",
        f"속도초과 누적  : {r['speeding_time']:.1f} s  (-{(10.0 if r['speeding_time']>0 else 0)+10.0*int(r['speeding_time']//3.0):.0f})",
        f"충돌 객체 수   : {n_collisions}  (-{COLLISION_PENALTY*n_collisions:.0f})",
        f"최대 횡오차    : {r['max_cte']:.2f} m   평균 횡오차: {r['mean_cte']:.2f} m",
        f"objective      : {core.objective(r):.1f}",
    ]
    out = "\n".join(lines)
    print(out)
    if args.out:
        args.out.write_text(out + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
