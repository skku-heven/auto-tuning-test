#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""auto_tune_ab — GP vs TPE 튜닝(제약목적: minimize time, 완주=제약).
기본 모드: smoke(각 n_smoke) → 완주율/median으로 승자 → 승자 full → top-K 재측정.
balanced 모드: 승자를 고르지 않고 각 sampler가 target trial 수에 도달할 때까지 실행.
장기 무인 견고화: sim순단 대기 가드(오염 대신 대기), SQLite resume(smoke 중복 안 함).
  python3 auto_tune_ab.py --hours 3 --seg 400 --timeout 120 --smoke 25
  python3 auto_tune_ab.py --mode balanced --target-trials 1000 --forever
"""
from __future__ import annotations
import os, sys, math, time, json, argparse


def _mods():
    p = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, p)
    import ab_core, autotune_core
    return ab_core, autotune_core


def enqueue_warmstart(studies):
    import ab_core
    for study in studies.values():
        for w in ab_core.WARMSTART:
            study.enqueue_trial(dict(w), skip_if_exists=True)


def wait_for_sim(runner, budget, poll=2.0):
    """sim이 신선(/Ego_topic 흐름)해질 때까지 블록. deadline/shutdown이면 False.
    장기 무인 중 MORAI 순단이 trial을 오염(disconnected)시키지 않게 대기로 흡수."""
    while budget["alive"]() and budget["time_left"]() > 0:
        if runner._fresh():
            return True
        if poll:
            time.sleep(poll)
    return False


def build_objective(runner, seg, timeout, out_dir, budget=None):
    import ab_core
    budget = budget or {"alive": lambda: True, "time_left": lambda: 1.0}

    def objective(trial):
        p = ab_core.suggest_params(trial)
        for k, v in p.items():
            trial.set_user_attr(k, v)
        trial.set_user_attr("seg", seg)
        if not wait_for_sim(runner, budget):
            trial.set_user_attr("aborted", True)
            return ab_core.objective_value({"completed": False, "progress_s": 0.0, "max_cte": 0.0}, seg, timeout)
        ok = runner.reset_and_arm() or runner.reset_and_arm()   # 1회 재시도
        if not ok:
            trial.set_user_attr("reset_failed", True)
            return ab_core.objective_value({"completed": False, "progress_s": 0.0, "max_cte": 0.0}, seg, timeout)
        save = os.path.join(out_dir, f"{trial.study.study_name}_{trial.number:04d}.csv") if out_dir else None
        m = runner.drive(p, save)
        if m is None:
            trial.set_user_attr("disconnected", True)
            return ab_core.objective_value({"completed": False, "progress_s": 0.0, "max_cte": 0.0}, seg, timeout)
        for k, v in m.items():
            trial.set_user_attr(k, v)
        if out_dir:
            with open(os.path.join(out_dir, f"{trial.study.study_name}_{trial.number:04d}.json"), "w") as f:
                json.dump({**p, **m}, f)
        return ab_core.objective_value(m, seg, timeout)
    return objective


def run_ab(studies, objective, n_smoke, budget):
    import ab_core
    alive = budget["alive"]; time_left = budget["time_left"]
    # smoke: interleave 한 라운드에 각 스터디 1 trial. resume 시 이미 돈 라운드는 건너뜀.
    done = min(len(st.get_trials(deepcopy=False)) for st in studies.values())
    for _ in range(max(0, n_smoke - done)):
        for st in studies.values():
            if not alive():
                break
            st.optimize(objective, n_trials=1, catch=(Exception,))
    stats = {name: ab_core.study_stats(st) for name, st in studies.items()}
    winner = ab_core.pick_winner(stats)
    ws = studies[winner]
    n_full = 0
    while alive() and time_left() > 0:
        ws.optimize(objective, n_trials=1, catch=(Exception,))
        n_full += 1
    return {"winner": winner, "stats": stats, "n_full": n_full}


def trial_count(study):
    return len(study.get_trials(deepcopy=False))


def run_balanced(studies, objective, target_trials, budget):
    """Run the currently-behind sampler until every study reaches target_trials.

    This resumes naturally from Optuna storage. If TPE has 590 trials and GP has
    20, GP is selected until the counts match, then the two alternate.
    """
    import ab_core
    alive = budget["alive"]; time_left = budget["time_left"]
    names = list(studies)
    added = {name: 0 for name in names}

    while alive() and time_left() > 0:
        counts = {name: trial_count(studies[name]) for name in names}
        pending = [name for name in names if counts[name] < target_trials]
        if not pending:
            break
        name = min(pending, key=lambda n: (counts[n], names.index(n)))
        studies[name].optimize(objective, n_trials=1, catch=(Exception,))
        added[name] += 1

    counts = {name: trial_count(studies[name]) for name in names}
    stats = {name: ab_core.study_stats(studies[name]) for name in names}
    return {
        "target_trials": target_trials,
        "target_reached": all(c >= target_trials for c in counts.values()),
        "counts": counts,
        "added": added,
        "stats": stats,
    }


def top_params(study, k, ab_core):
    fs = sorted(ab_core.feasible_trials(study), key=lambda t: t.value)[:k]
    return [
        {**{name: t.user_attrs[name] for name, _, _, _ in ab_core.PARAM_SPECS},
         "pid_kp": ab_core.PID_KP}
        for t in fs
    ]


def final_remeasure(runner, top_params, repeats, seg, timeout, out_dir):
    results = []
    for i, p in enumerate(top_params):
        times = []; feas = 0
        for r in range(repeats):
            if not (runner.reset_and_arm() or runner.reset_and_arm()):
                continue
            save = os.path.join(out_dir, f"final_{i}_{r}.csv") if out_dir else None
            m = runner.drive(p, save)
            if m is None:
                continue
            if m["completed"]:
                feas += 1; times.append(m["time_s"])
        results.append({"params": p, "feasible": feas, "repeats": repeats,
                        "mean_time": round(sum(times) / len(times), 2) if times else None,
                        "worst_time": max(times) if times else None})
    results.sort(key=lambda r: (-r["feasible"], r["mean_time"] if r["mean_time"] is not None else 1e9))
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hours", type=float, default=3.0)
    ap.add_argument("--seg", type=float, default=400.0)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--smoke", type=int, default=25)
    ap.add_argument("--topk", type=int, default=4)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--seed", type=int, default=20260702)
    ap.add_argument("--mode", choices=("winner", "balanced"), default="winner",
                    help="winner: smoke 후 승자만 계속. balanced: 둘 다 target trial까지 계속.")
    ap.add_argument("--target-trials", type=int, default=1000,
                    help="--mode balanced에서 sampler별 누적 목표 trial 수")
    ap.add_argument("--forever", action="store_true", help="멈출 때까지 무제한(deadline 없음)")
    ap.add_argument("--storage", default=None,
                    help="Optuna storage URL. 미지정 시 env OPTUNA_STORAGE, 그것도 없으면 sqlite. "
                         "PostgreSQL 이전 시: postgresql://user:pw@host:5432/db (psycopg2 필요)")
    args = ap.parse_args()

    ab_core, core = _mods()
    HERE = os.path.dirname(os.path.abspath(__file__))
    CSV = os.path.join(HERE, "..", "paths", "kcity_2025.csv")
    OUT = os.path.join(HERE, "..", "results", "live_tune")
    DB = os.path.join(OUT, "tune_ab.db")
    START = (7.5766, -279.0828, 28.5, 60.9)
    os.makedirs(OUT, exist_ok=True)
    # storage: Optuna RDB(SQLAlchemy) — sqlite/postgres URL만 바꾸면 백엔드 이전됨(스키마 동일).
    # 나중에 Postgres 이전: OPTUNA_STORAGE=postgresql://... 로 실행하면 그대로 동작.
    # 기존 sqlite DB → postgres 이전은 optuna.copy_study(from, to)로 옮길 수 있음.
    storage = args.storage or os.environ.get("OPTUNA_STORAGE") or f"sqlite:///{DB}"

    import rospy, optuna
    track = core.load_track_csv(CSV); n = len(track)
    s_arr = [0.0]
    for i in range(1, n):
        s_arr.append(s_arr[-1] + math.hypot(track[i][0] - track[i - 1][0], track[i][1] - track[i - 1][1]))

    rospy.init_node("auto_tune_ab", anonymous=True)
    from live_runner import LiveRunner
    runner = LiveRunner(track, s_arr, args.seg, args.timeout, START)

    sam = ab_core.make_samplers(args.seed)
    studies = {}
    for name, sn in (("tpe", f"kcity_seg{int(args.seg)}_tpe_v4"), ("gp", f"kcity_seg{int(args.seg)}_gp_v4")):
        studies[name] = optuna.create_study(direction="minimize", study_name=sn,
            storage=storage, load_if_exists=True, sampler=sam[name])
    enqueue_warmstart(studies)

    if args.forever:
        budget = {"alive": lambda: not rospy.is_shutdown(), "time_left": lambda: float("inf")}
    else:
        deadline = time.time() + args.hours * 3600
        budget = {"alive": lambda: not rospy.is_shutdown(), "time_left": lambda: deadline - time.time()}
    objective = build_objective(runner, args.seg, args.timeout, OUT, budget)
    dl = "무제한(멈출때까지)" if args.forever else f"{args.hours}h"
    print(f"[ab] mode={args.mode} seg={args.seg} smoke={args.smoke}/sampler "
          f"target={args.target_trials} deadline={dl} storage={storage.split('://')[0]}",
          flush=True)

    if args.mode == "balanced":
        res = run_balanced(studies, objective, args.target_trials, budget)
        leader = ab_core.pick_winner(res["stats"])
        print(f"[ab] balanced target_reached={res['target_reached']} "
              f"counts={res['counts']} added={res['added']} leader={leader} "
              f"stats={res['stats']}", flush=True)

        finals = {}
        if res["target_reached"]:
            for name, study in studies.items():
                finals[name] = final_remeasure(
                    runner, top_params(study, args.topk, ab_core),
                    args.repeats, args.seg, args.timeout, OUT)
        with open(os.path.join(OUT, "best_ab_balanced.json"), "w") as f:
            json.dump({"mode": "balanced", "leader": leader, **res,
                       "final": finals}, f, indent=2)
        print(f"[ab] DONE balanced. target_reached={res['target_reached']} "
              f"leader={leader}", flush=True)
        return

    res = run_ab(studies, objective, args.smoke, budget)
    print(f"[ab] winner={res['winner']} stats={res['stats']} full={res['n_full']}", flush=True)

    # 재측정: 승자 feasible 중 목적값(작을수록 좋음) 상위 topk
    ws = studies[res["winner"]]
    finals = final_remeasure(runner, top_params(ws, args.topk, ab_core),
                             args.repeats, args.seg, args.timeout, OUT)
    with open(os.path.join(OUT, "best_ab.json"), "w") as f:
        json.dump({"winner": res["winner"], "stats": res["stats"], "final": finals}, f, indent=2)
    print(f"[ab] DONE. best={finals[0] if finals else None}", flush=True)


if __name__ == "__main__":
    main()
