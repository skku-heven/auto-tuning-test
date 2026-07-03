#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""
Self-contained tests for the ad_autotune harness — `python3 test_autotune.py`.
No pytest needed.  Codifies the behaviours validated during the build so they
don't regress.
"""
import math
import sys

import autotune_core as core

PASS = 0
FAIL = 0


def check(name, cond):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ok   {name}")
    else:
        FAIL += 1
        print(f"  FAIL {name}")


def base(**over):
    p = dict(lookahead=3.0, target_velocity_kph=20.0, gain_k=0.5,
             pid_kp=0.3, pid_ki=0.0, pid_kd=0.05)
    p.update(over)
    return p


# --- tracks --------------------------------------------------------------
oval = core.generate_oval_track()
tight = core.generate_tight_track()
check("oval is a closed loop (>100 wpts)", len(oval) > 100)
check("oval start≈end (closed)", math.hypot(oval[0][0] - oval[-1][0],
                                            oval[0][1] - oval[-1][1]) < 1.0)
check("tight has smaller extent than oval",
      (max(p[0] for p in tight) - min(p[0] for p in tight)) <
      (max(p[0] for p in oval) - min(p[0] for p in oval)))

# --- clean run scores 100 and completes ----------------------------------
r = core.simulate(oval, base())
check("clean run completes", r["completed"])
check("clean run scores 100", r["driving_score"] == 100.0)
check("clean run stays on track (max_cte<1.5)", r["max_cte"] < 1.5)

# --- speeding is penalised ------------------------------------------------
# force high target so actual speed exceeds 50 kph
rs = core.simulate(oval, base(target_velocity_kph=70.0, pid_kp=0.6))
check("speeding accrues speeding_time", rs["speeding_time"] > 0)
check("speeding loses points (<100)", rs["driving_score"] < 100.0)

# --- corner-cutting (gaming) is rejected ---------------------------------
rg = core.simulate(tight, base(lookahead=10.0, target_velocity_kph=45.0, gain_k=0.1))
check("gaming run goes off-lane (max_cte>3)", rg["max_cte"] > 3.0)
check("gaming run flagged off_road", rg.get("off_road") is True)
check("gaming run NOT completed (실격)", not rg["completed"])

# --- objective ordering: completed >> not-completed ----------------------
good = core.objective(core.simulate(oval, base()))
bad = core.objective(rg)
check("objective: completed beats off-road", good > bad)

# --- among CLEAN (score-100) runs, faster wins; a penalised-but-faster run
#     must NOT beat a clean slower run (규정: 주행점수 먼저, 시간 나중) ----------
slow_clean = core.simulate(oval, base(target_velocity_kph=20.0))
fast_clean = core.simulate(oval, base(target_velocity_kph=45.9, lookahead=10.0, gain_k=1.5, pid_kp=0.6))
check("fast_clean also scores 100", fast_clean["driving_score"] == 100.0 and fast_clean["time_s"] < slow_clean["time_s"])
check("among clean runs, faster wins", core.objective(fast_clean) > core.objective(slow_clean))
penalised_fast = core.simulate(oval, base(target_velocity_kph=45.0, lookahead=6.0, gain_k=1.0))
check("penalised-but-faster < clean (주행점수 우선)",
      penalised_fast["driving_score"] < 100.0 and
      core.objective(penalised_fast) < core.objective(slow_clean))

# --- score_run on external samples matches simulate (offline/live parity) -
ctrl = core.StanleyController(oval, **base())
x0, y0 = oval[0]
x1, y1 = oval[1]
plant = core.BicyclePlant(x0, y0, math.atan2(y1 - y0, x1 - x0))
samples, t = [], 0.0
for _ in range(4000):
    v = plant.v * 3.6
    steer, thr, brk, _c, _n = ctrl.control(plant.x, plant.y, plant.theta, v, t)
    samples.append((t, plant.x, plant.y, v))
    plant.step(steer, thr, brk, core.DT)
    t += core.DT
    if core._path_lateral_error(oval, plant.x, plant.y) > 12 or len(samples) > 4000:
        break
rr = core.score_run(oval, samples)
check("score_run on external samples completes", rr["completed"])
check("score_run parity: scores 100 like simulate", rr["driving_score"] == 100.0)

# --- record→load round trip ----------------------------------------------
import tempfile, os
tmp = os.path.join(tempfile.gettempdir(), "test_track.csv")
core.write_track_csv(oval, tmp)
loaded = core.load_track_csv(tmp)
check("write/load round-trips waypoint count", abs(len(loaded) - len(oval)) <= 1)

print(f"\n{PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
