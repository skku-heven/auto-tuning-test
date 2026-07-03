#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""
ad_autotune — the optimizer.  Sweeps ad_tracker params to maximize the
competition objective (완주 > 주행점수 > 완주시간) and writes tuned params
that drop straight into ad_tracker.launch.

  python3 autotune.py            # full run (grid + refine) on the synthetic track
  python3 autotune.py --quick    # smaller grid, for a fast check

Output:
  results/trials.csv        every evaluated param set + score
  results/tuned_params.yaml best params + the roslaunch command to use them
"""
from __future__ import annotations

import argparse
import itertools
import os

import autotune_core as core

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "..", "results")

# tunable params (the ones in ad_tracker.launch). pid_ki/kd fixed to keep dims low.
FIXED = dict(pid_ki=0.0, pid_kd=0.05)
BOUNDS = dict(
    lookahead=(1.5, 10.0),
    target_velocity_kph=(10.0, 55.0),
    gain_k=(0.1, 4.0),
    pid_kp=(0.1, 1.0),
)
ORDER = ["lookahead", "target_velocity_kph", "gain_k", "pid_kp"]


def make_params(vec):
    p = {k: vec[i] for i, k in enumerate(ORDER)}
    p.update(FIXED)
    return p


class Trials:
    def __init__(self):
        self.rows = []
        self.best = None
        self.best_obj = -float("inf")

    def eval(self, track, vec):
        vec = [max(BOUNDS[k][0], min(BOUNDS[k][1], vec[i])) for i, k in enumerate(ORDER)]
        params = make_params(vec)
        r = core.simulate(track, params)
        obj = core.objective(r)
        self.rows.append((vec, r, obj))
        if obj > self.best_obj:
            self.best_obj = obj
            self.best = (vec, r)
            print(f"  ★ #{len(self.rows):3d} obj={obj:11.1f} "
                  f"done={int(r['completed'])} score={r['driving_score']:.0f} "
                  f"t={r['time_s']:.1f}s | LA={vec[0]:.2f} V={vec[1]:.1f} "
                  f"k={vec[2]:.2f} kp={vec[3]:.2f}")
        return obj


def grid_search(track, trials, quick=False):
    if quick:
        space = dict(lookahead=[3, 6], target_velocity_kph=[30, 48],
                     gain_k=[0.5, 1.5], pid_kp=[0.4])
    else:
        space = dict(lookahead=[2, 4, 6, 8], target_velocity_kph=[25, 35, 45, 52],
                     gain_k=[0.3, 0.8, 1.5, 3.0], pid_kp=[0.3, 0.6])
    combos = list(itertools.product(*[space[k] for k in ORDER]))
    print(f"[grid] {len(combos)} combinations")
    for c in combos:
        trials.eval(track, list(c))


def pattern_search(track, trials, start, iters=40):
    """Hooke-Jeeves coordinate pattern search from `start` vector."""
    print(f"[refine] pattern search, {iters} iterations")
    x = list(start)
    step = [(BOUNDS[k][1] - BOUNDS[k][0]) * 0.15 for k in ORDER]
    base = trials.eval(track, x)
    for _ in range(iters):
        improved = False
        for i in range(len(ORDER)):
            for s in (step[i], -step[i]):
                cand = list(x)
                cand[i] += s
                o = trials.eval(track, cand)
                if o > base:
                    base, x, improved = o, cand, True
                    break
        if not improved:
            step = [s * 0.5 for s in step]
            if max(step) < 1e-3:
                break
    return x


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--track", default="oval", choices=list(core.TRACKS),
                    help="oval=easy curves, tight=sharp corners")
    ap.add_argument("--csv", default=None,
                    help="tune on a recorded path CSV instead of a synthetic track")
    args = ap.parse_args()

    os.makedirs(RESULTS, exist_ok=True)
    if args.csv:
        track = core.load_track_csv(args.csv)
        args.track = os.path.splitext(os.path.basename(args.csv))[0]
        print(f"track: {args.csv}, {len(track)} waypoints")
    else:
        track = core.TRACKS[args.track]()
        core.write_track_csv(track, os.path.join(HERE, "..", "paths", f"{args.track}_track.csv"))
        print(f"track: {args.track}, {len(track)} waypoints")

    trials = Trials()
    # baseline (current ad_tracker.launch defaults) for comparison
    base_params = [3.0, 20.0, 0.5, 0.3]
    base_obj = trials.eval(track, base_params)
    base_row = trials.rows[-1][1]
    print(f"[baseline] obj={base_obj:.1f} done={int(base_row['completed'])} "
          f"score={base_row['driving_score']:.0f} t={base_row['time_s']:.1f}s\n")

    grid_search(track, trials, quick=args.quick)
    best_vec = trials.best[0]
    pattern_search(track, trials, best_vec, iters=15 if args.quick else 40)

    # ---- write results ----
    csv_path = os.path.join(RESULTS, "trials.csv")
    with open(csv_path, "w") as f:
        f.write("trial,lookahead,target_velocity_kph,gain_k,pid_kp,"
                "completed,driving_score,time_s,progress,penalty,max_cte,objective\n")
        for i, (vec, r, obj) in enumerate(trials.rows):
            f.write(f"{i},{vec[0]:.3f},{vec[1]:.2f},{vec[2]:.3f},{vec[3]:.3f},"
                    f"{int(r['completed'])},{r['driving_score']:.1f},{r['time_s']:.2f},"
                    f"{r['progress']:.3f},{r['penalty']:.1f},{r['max_cte']:.2f},{obj:.1f}\n")

    bvec, br = trials.best
    yaml_path = os.path.join(RESULTS, "tuned_params.yaml")
    with open(yaml_path, "w") as f:
        f.write("# ad_autotune best params (synthetic oval track)\n")
        f.write(f"# evaluated {len(trials.rows)} param sets\n")
        f.write(f"# completed={br['completed']} driving_score={br['driving_score']} "
                f"time_s={br['time_s']} max_cte={br['max_cte']}\n")
        f.write(f"lookahead: {bvec[0]:.3f}\n")
        f.write(f"target_velocity_kph: {bvec[1]:.2f}\n")
        f.write(f"gain_k: {bvec[2]:.3f}\n")
        f.write(f"pid_kp: {bvec[3]:.3f}\n")
        f.write(f"pid_ki: {FIXED['pid_ki']}\n")
        f.write(f"pid_kd: {FIXED['pid_kd']}\n")

    print("\n" + "=" * 64)
    print(f"BEST after {len(trials.rows)} evals:")
    print(f"  lookahead={bvec[0]:.2f} target_velocity_kph={bvec[1]:.1f} "
          f"gain_k={bvec[2]:.2f} pid_kp={bvec[3]:.2f}")
    print(f"  completed={br['completed']} driving_score={br['driving_score']:.0f} "
          f"time_s={br['time_s']:.1f}  (baseline t={base_row['time_s']:.1f}s)")
    speedup = (base_row['time_s'] - br['time_s']) / base_row['time_s'] * 100 \
        if br['completed'] and base_row['completed'] else 0.0
    print(f"  -> {speedup:.0f}% faster than baseline at score {br['driving_score']:.0f}")
    print(f"\nresults: {csv_path}")
    print(f"tuned:   {yaml_path}")
    print("\n[!] toy-model starting point, NOT competition params — re-tune live.")
    print("starting point for a live run:")
    print(f"  roslaunch ad_tracker ad_tracker.launch \\")
    print(f"    lookahead:={bvec[0]:.2f} target_velocity_kph:={bvec[1]:.1f} \\")
    print(f"    gain_k:={bvec[2]:.2f} pid_kp:={bvec[3]:.2f}")

    # visualize the best run
    best_params = make_params(bvec)
    rr = core.simulate(track, best_params, record=True)
    traj_path = os.path.join(RESULTS, f"{args.track}_best_traj.csv")
    with open(traj_path, "w") as f:
        f.write("t,x,y,v_kph\n")
        for s in rr["traj"]:
            f.write(",".join(str(v) for v in s) + "\n")
    print(f"\n[{args.track}] best run ('.'=track  '*'=driven):")
    print(core.ascii_plot(track, rr["traj"]))


if __name__ == "__main__":
    main()
