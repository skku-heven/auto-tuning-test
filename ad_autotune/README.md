# ad_autotune

Auto-tunes the `ad_tracker` Stanley controller params to maximize the
competition objective — **완주 > 주행점수(100−감점) > 완주시간** (2025 규정 기준).

It mirrors `ad_tracker/src/gps_tracker.cpp` exactly, so the params it finds
(`lookahead / target_velocity_kph / gain_k / pid_*`) drop straight into
`ad_tracker.launch`.

## Two modes — same controller, same scoring

| mode | "plant" | needs sim? | use |
|------|---------|-----------|-----|
| **offline** (`autotune.py`) | kinematic bicycle model | ❌ | prove the workflow, search 100s of params in seconds |
| **live** (`run_live_tuning.py`) | the running MORAI sim over ROS | ✅ | final tuning on the real map |

The scoring (`autotune_core.score_run`) is **identical** in both — only the
plant is swapped. So params validated offline transfer to live with the same
metric.

## Quick start

```bash
# EVERYTHING in one command (offline tune both tracks -> bag -> ROS 채점), no sim:
cd ad_autotune/scripts && ./demo.sh

# offline — runs now, no ROS/sim needed
python3 autotune.py            # full grid + pattern-search refine
python3 autotune.py --quick    # fast check

# live dry-run — full trial loop, still offline plant
python3 run_live_tuning.py --dry-run

# live — once MORAI Ego Network(ROS) is Connected
rosrun ad_autotune run_live_tuning.py \
    --path $(rospack find ad_tracker)/csv/global_path.csv
```

Outputs land in `ad_autotune/results/`:
- `trials.csv` — every param set evaluated + score
- `tuned_params.yaml` — best params + the exact `roslaunch` line to use them

## Results — proof the loop works AND adapts

> ⚠️ **These are toy-model numbers, NOT competition params.** The plant is a
> kinematic bicycle (no tire/actuation dynamics, made-up accel/drag). The values
> below prove the *optimization loop runs end-to-end and adapts* — they are a
> starting point, not a tuning result. Tells that they're toy artifacts:
> lookahead pins to the upper bound (10.0 = optimizer just maxed a param, no
> interior optimum) and velocity converges "just under 50" by exploiting that
> toy drag makes actual < target. **Real tuning = run this loop against live
> MORAI** (blocked on your Connect). 껀덕지(feasibility) = yes; numbers = demo.

Two synthetic tracks, ~250 evals each:

| track | params (LA / V / k / kp) | time | score | vs baseline |
|-------|--------------------------|------|-------|-------------|
| **oval** (gentle curves) | 10.0 / 45.9 / 1.5 / 0.6 | 15.3 s | 100 | 32.5→15.3 s (−53%) |
| **tight** (sharp corners) | **4.0** / 48.4 / 0.8 / 0.6 | 7.1 s | 100 | 13.9→7.1 s (−49%) |

The optimizer **adapts per track**: gentle oval → long lookahead (10) cuts time;
sharp corners → it *drops lookahead to 4* to stay on the lane. On both it pushes
velocity to *just under* the 50 kph penalty line on its own. Tuned runs hug the
path (mean lateral error < 0.7 m) — `python3 autotune.py` prints an ASCII view.

### Anti-gaming (why the scores are trustworthy)
A naive scorer let high-lookahead params *cut corners* (6 m off-path) yet score
100, because short runs never hit the 3-second-cumulative 경로이탈 penalty.
`score_run` now voids 완주 (코스이탈 실격) when the car leaves the lane
(>2 m for >0.7 s), so corner-cutting params fail completion and the optimizer
rejects them — the tuned params actually follow the course.

## Real-map workflow (the scenario-coords fix)

Last year's `global_path.csv` + scenario are on the **old** map frame, so they
don't match `R_KR_PR_K-city_2025`. Don't port coordinates — **record a fresh
path on the new map**:

```bash
# 1) drive the course once in MORAI (manual), record the path
rosrun ad_autotune record_path.py --out $(rospack find ad_tracker)/csv/global_path.csv
# 2) tune the controller on that recorded path
python3 autotune.py --csv $(rospack find ad_tracker)/csv/global_path.csv
# 3) run ad_tracker with the tuned params; 4) score the bag with competition_score.py
```

`record_path.py` also extracts a path from an existing bag (`--bag run.bag`) —
that's how the record→tune chain is tested offline. Validated: bag → 300-wpt
path → tuned params, end-to-end.

## Going live — the full workflow

```
roscore                                         # :11311
roslaunch rosbridge_server rosbridge_websocket  # :9090
# MORAI: Ego Network -> ROS -> Connect           (/Competition_topic, /ctrl_cmd flow)
catkin build && source devel/setup.bash
rosrun ad_autotune run_live_tuning.py --path <global_path.csv>
```

Each live trial: reset ego → relaunch `ad_tracker` with the param set → record
`/Competition_topic` → `score_run` → next.

### Known blockers (state as of build)

1. **MORAI "Connect" is a UI press** — can't be scripted from here. Bridge
   (roscore + rosbridge) is already up and waiting; just Connect.
2. **Ego reset between trials** — SimControl (:20000) is **dead** on this
   competition build, so there's no scripted scenario reset. `run_live_tuning.py`
   tries `/Service_MoraiEventCmd` (best-effort) and supports `--manual-reset`.
   Confirm which works on your build — it's the only unverified link.
3. **Scenario coords** — last-year scenario is on the old map frame; the
   synthetic oval track sidesteps this for proving the loop. Real-map tuning
   needs a `global_path.csv` recorded on `R_KR_PR_K-city_2025`.
4. **ad_tracker bug A** — `global_path.csv` col3 (≈28.5, the altitude) is read
   as heading. The synthetic track uses 2 columns to avoid it; for live runs on
   real csv, apply the fix (drop col3-as-heading, derive path_theta from
   consecutive waypoints — see `gps_tracker.cpp` FindTarget/Stanley).

## Scoring a recorded run (rosbag)

The same competition scoring is available as a ROS node, so a real MORAI run
(or a synthetic test bag) is scored identically to the optimizer:

```bash
# make a test bag from a sim run (no MORAI needed) and score it
python3 scripts/make_test_bag.py --track oval --out /tmp/run.bag
rosrun ad_metric competition_score.py --bag /tmp/run.bag --csv paths/oval_track.csv
```

`competition_score.py` imports `score_run` (single source of truth) and adds the
충돌 감점 (-10/객체). Validated: synthetic bag → score matches; a 60 kph bag
correctly loses 30 pts to 속도초과.

## Files

- `scripts/autotune_core.py` — track gen, bicycle plant, Stanley mirror, `score_run`
- `scripts/autotune.py` — offline optimizer (grid + Hooke-Jeeves), ASCII viz
- `scripts/run_live_tuning.py` — live/dry-run trial loop over ROS
- `scripts/make_test_bag.py` — sim run → rosbag (test the metric without MORAI)
- `../ad_metric/scripts/competition_score.py` — rosbag → 대회 채점 (additive node)
- `paths/{oval,tight}_track.csv` — synthetic tracks  ·  `results/` — logs + tuned params
