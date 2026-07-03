# Optuna A/B (GP vs TPE) + 제약목적 튜닝 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** MORAI 라이브에서 governed-Stanley 5파라미터를 "완주=제약, 시간=목적"으로 튜닝하되, GPSampler와 TPESampler를 interleave A/B로 검증해 승자를 채택한다.

**Architecture:** 순수 결정로직(`ab_core.py`, 시뮬 불필요·pytest 가능) / 라이브 주행 러너(`live_runner.py`, ROS+MORAI) / 오케스트레이터(`auto_tune_ab.py`, 두 스터디 interleave→승자→full→재측정)로 3분리. 기존 `auto_tune_live.py`(v3)와 `ad_control.py`(제어기)는 **건드리지 않음**.

**Tech Stack:** Python3, ROS1 Noetic (rospy, morai_msgs), Optuna (TPESampler/GPSampler + constraints_func), SQLite storage, pytest.

## Global Constraints

- 제어기 `ad_control.py` 변경 금지. 기존 `auto_tune_live.py` 변경 금지(v3 아카이브/폴백).
- 스크립트 위치: `src/heven-common-test/ad_autotune/scripts/`. 테스트: `ad_autotune/tests/`.
- 파일 상단 주석 `# [transferable-to-heven_ad]` 관례 유지(신규 스크립트).
- Optuna 스터디: `direction="minimize"`, storage=`sqlite:///results/live_tune/tune_ab.db`, `load_if_exists=True`.
- 스터디명: TPE=`kcity_seg400_tpe_v4`, GP=`kcity_seg400_gp_v4`. 시드=20260702.
- 탐색공간(고정): lookahead 1.5–5.5 / target_velocity_kph 25–55 / gain_k 0.4–3.0(**log**) / k_soft 0.5–3.0 / a_lat 1.0–3.0. pid_kp=0.3 고정.
- 실행 커맨드 관례: `PYTHONNOUSERSITE=0 python3 ...`, rosbridge PYTHONPATH에 `/home/taeyeong/heven_common_test_ws/devel/lib/python3/dist-packages`.

---

### Task 1: Preflight — Optuna/GPSampler 환경 검증

목적: 구현 전에 GPSampler가 이 머신에서 실제로 import/동작하는지(의존성 torch/scipy), constraints_func가 두 샘플러에 지원되는지 확인. 실패 시 여기서 막고 해결.

**Files:**
- Create: `ad_autotune/tests/test_preflight.py`

**Interfaces:**
- Produces: 없음(검증만). 결과를 스펙 리스크 항목에 반영.

- [ ] **Step 1: Preflight 테스트 작성**

```python
# ad_autotune/tests/test_preflight.py
import optuna

def test_optuna_version():
    major = int(optuna.__version__.split(".")[0])
    assert major >= 3, f"optuna {optuna.__version__} < 3.x"

def test_gpsampler_importable():
    # GPSampler는 torch/scipy 의존 — 없으면 여기서 실패
    s = optuna.samplers.GPSampler(seed=1)
    assert s is not None

def test_constraints_func_accepted():
    cf = lambda t: [0.0]
    tpe = optuna.samplers.TPESampler(constraints_func=cf)
    gp = optuna.samplers.GPSampler(constraints_func=cf)
    assert tpe is not None and gp is not None
```

- [ ] **Step 2: 실행해 통과/실패 확인**

Run: `cd ad_autotune && PYTHONNOUSERSITE=0 python3 -m pytest tests/test_preflight.py -v`
Expected: 3 PASS. 만약 `test_gpsampler_importable`가 ModuleNotFoundError(torch 등)로 FAIL이면 **중단하고 사용자에게 보고**(GP 못 씀 → TPE-only로 축소하거나 의존성 설치 결정 필요). constraints_func가 GPSampler에서 TypeError면 마찬가지로 보고.

- [ ] **Step 3: 커밋**

```bash
git add ad_autotune/tests/test_preflight.py
git commit -m "test: optuna/GPSampler preflight 검증"
```

---

### Task 2: `ab_core.py` — 순수 결정로직 (파라미터/목적/제약/승자선택)

**Files:**
- Create: `ad_autotune/scripts/ab_core.py`
- Test: `ad_autotune/tests/test_ab_core.py`

**Interfaces:**
- Produces:
  - `PARAM_SPECS: list[tuple[str,float,float,bool]]`, `PID_KP=0.3`
  - `WARMSTART: list[dict]` (6개, 각 dict은 5파라미터 키)
  - `suggest_params(trial) -> dict` (5파라미터 + pid_kp)
  - `objective_value(m: dict, seg: float, timeout: float) -> float` (minimize)
  - `constraints_func(trial) -> list[float]` (≤0 feasible)
  - `make_samplers(seed: int) -> dict` ({"tpe":TPESampler,"gp":GPSampler})
  - `feasible_trials(study) -> list`, `study_stats(study) -> dict`
  - `pick_winner(stats: dict) -> str` ("tpe"|"gp")

- [ ] **Step 1: 실패 테스트 작성**

```python
# ad_autotune/tests/test_ab_core.py
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import ab_core

class FakeTrial:
    def __init__(self, attrs=None): self.user_attrs = attrs or {}; self.calls = []
    def suggest_float(self, name, lo, hi, log=False):
        self.calls.append((name, lo, hi, log)); return lo
    def set_user_attr(self, k, v): self.user_attrs[k] = v

def test_param_specs_ranges():
    d = {n:(lo,hi,log) for n,lo,hi,log in ab_core.PARAM_SPECS}
    assert d["target_velocity_kph"][:2] == (25.0, 55.0)
    assert d["gain_k"][2] is True          # log scale
    assert d["lookahead"][:2] == (1.5, 5.5)

def test_warmstart_within_ranges():
    d = {n:(lo,hi) for n,lo,hi,_ in ab_core.PARAM_SPECS}
    assert len(ab_core.WARMSTART) == 6
    for w in ab_core.WARMSTART:
        for n,(lo,hi) in d.items():
            assert lo <= w[n] <= hi, f"{n}={w[n]} out of [{lo},{hi}]"

def test_suggest_params_uses_specs():
    t = FakeTrial(); p = ab_core.suggest_params(t)
    assert set(p) == {"lookahead","target_velocity_kph","gain_k","k_soft","a_lat","pid_kp"}
    assert p["pid_kp"] == 0.3
    assert ("gain_k", 0.4, 3.0, True) in t.calls

def test_objective_completed_returns_time():
    m = {"completed": True, "time_s": 48.7, "progress_s": 400.0, "max_cte": 0.5}
    assert ab_core.objective_value(m, 400, 120) == 48.7

def test_objective_incomplete_is_worse_than_timeout_and_monotonic():
    near = {"completed": False, "progress_s": 380.0, "max_cte": 1.0}
    far  = {"completed": False, "progress_s": 100.0, "max_cte": 1.0}
    v_near = ab_core.objective_value(near, 400, 120)
    v_far  = ab_core.objective_value(far, 400, 120)
    assert v_near > 120 and v_far > 120          # 완주(≤120)보다 항상 나쁨
    assert v_near < v_far                         # 더 진행할수록 좋음(작음)

def test_constraints_feasible_only_when_completed():
    assert ab_core.constraints_func(FakeTrial({"completed": True})) == [0.0]
    assert ab_core.constraints_func(FakeTrial({"completed": False})) == [1.0]
    assert ab_core.constraints_func(FakeTrial({"reset_failed": True})) == [1.0]
    assert ab_core.constraints_func(FakeTrial({"disconnected": True})) == [1.0]

def test_pick_winner_prefers_feasible_then_lower_median():
    stats = {"tpe": {"feasible": 5, "median_time": 60.0},
             "gp":  {"feasible": 8, "median_time": 70.0}}
    assert ab_core.pick_winner(stats) == "gp"            # 완주율 우선
    stats2 = {"tpe": {"feasible": 8, "median_time": 55.0},
              "gp":  {"feasible": 8, "median_time": 70.0}}
    assert ab_core.pick_winner(stats2) == "tpe"          # 동률 → median 낮은쪽
    stats3 = {"tpe": {"feasible": 0, "median_time": None},
              "gp":  {"feasible": 1, "median_time": 90.0}}
    assert ab_core.pick_winner(stats3) == "gp"

def test_make_samplers_types():
    import optuna
    s = ab_core.make_samplers(20260702)
    assert isinstance(s["tpe"], optuna.samplers.TPESampler)
    assert isinstance(s["gp"], optuna.samplers.GPSampler)
```

- [ ] **Step 2: 실행해 실패 확인**

Run: `cd ad_autotune && PYTHONNOUSERSITE=0 python3 -m pytest tests/test_ab_core.py -v`
Expected: FAIL (ModuleNotFoundError: ab_core).

- [ ] **Step 3: `ab_core.py` 구현**

```python
# ad_autotune/scripts/ab_core.py
# [transferable-to-heven_ad]
"""ab_core — Optuna A/B 튜닝의 순수 결정로직(시뮬 불필요, 테스트 가능).
minimize(time_s), 완주=제약(constraints_func), 미완주=연속 proxy(절벽 없음)."""
import statistics

PID_KP = 0.3
# (name, low, high, log)
PARAM_SPECS = [
    ("lookahead", 1.5, 5.5, False),
    ("target_velocity_kph", 25.0, 55.0, False),
    ("gain_k", 0.4, 3.0, True),
    ("k_soft", 0.5, 3.0, False),
    ("a_lat", 1.0, 3.0, False),
]
# v3 seg400 완주기록 기반 6점(공격~중속, 다양한 basin)
WARMSTART = [
    {"lookahead": 4.26, "target_velocity_kph": 49.8, "gain_k": 2.77, "k_soft": 1.72, "a_lat": 2.84},
    {"lookahead": 2.19, "target_velocity_kph": 37.4, "gain_k": 2.43, "k_soft": 0.58, "a_lat": 2.48},
    {"lookahead": 4.19, "target_velocity_kph": 41.9, "gain_k": 2.03, "k_soft": 1.83, "a_lat": 2.54},
    {"lookahead": 5.15, "target_velocity_kph": 42.6, "gain_k": 2.70, "k_soft": 0.99, "a_lat": 2.28},
    {"lookahead": 3.71, "target_velocity_kph": 35.5, "gain_k": 1.13, "k_soft": 0.77, "a_lat": 1.27},
    {"lookahead": 2.74, "target_velocity_kph": 30.0, "gain_k": 1.38, "k_soft": 2.53, "a_lat": 1.13},
]


def suggest_params(trial):
    p = {}
    for name, lo, hi, log in PARAM_SPECS:
        p[name] = trial.suggest_float(name, lo, hi, log=log)
    p["pid_kp"] = PID_KP
    return p


def objective_value(m, seg, timeout):
    """minimize. 완주 → time_s(≤timeout). 미완주 → timeout+(seg-progress)+max_cte*0.5 (>timeout, 진행할수록 작음)."""
    if m.get("completed"):
        return float(m["time_s"])
    return float(timeout + (seg - m.get("progress_s", 0.0)) + m.get("max_cte", 0.0) * 0.5)


def constraints_func(trial):
    """≤0 이면 feasible. 완주만 feasible; reset실패/disconnect는 infeasible."""
    a = trial.user_attrs
    if a.get("reset_failed") or a.get("disconnected"):
        return [1.0]
    return [0.0 if a.get("completed") else 1.0]


def make_samplers(seed):
    import optuna
    tpe = optuna.samplers.TPESampler(
        seed=seed, n_startup_trials=15, multivariate=True, group=True,
        constraints_func=constraints_func)
    gp = optuna.samplers.GPSampler(
        seed=seed, n_startup_trials=15, deterministic_objective=False,
        constraints_func=constraints_func)
    return {"tpe": tpe, "gp": gp}


def feasible_trials(study):
    import optuna
    out = []
    for t in study.trials:
        if t.state != optuna.trial.TrialState.COMPLETE:
            continue
        a = t.user_attrs
        if a.get("reset_failed") or a.get("disconnected"):
            continue
        if a.get("completed"):
            out.append(t)
    return out


def study_stats(study):
    fs = feasible_trials(study)
    times = [t.user_attrs["time_s"] for t in fs]
    return {"feasible": len(fs),
            "median_time": statistics.median(times) if times else None}


def pick_winner(stats):
    def key(name):
        s = stats[name]; mt = s["median_time"]
        return (s["feasible"], -(mt if mt is not None else 1e9))
    return max(stats, key=key)
```

- [ ] **Step 4: 실행해 통과 확인**

Run: `cd ad_autotune && PYTHONNOUSERSITE=0 python3 -m pytest tests/test_ab_core.py -v`
Expected: 8 PASS.

- [ ] **Step 5: 커밋**

```bash
git add ad_autotune/scripts/ab_core.py ad_autotune/tests/test_ab_core.py
git commit -m "feat: ab_core — 제약목적 A/B 순수 결정로직 + 테스트"
```

---

### Task 3: `live_runner.py` — 리셋/주행 러너 (기존 auto_tune_live 로직 재사용)

기존 `auto_tune_live.py`의 `reset_and_arm`/`drive` 인라인 로직을 클래스로 옮긴 것. 로직 동일, ad_control 제어기 그대로. 라이브 필요라 유닛테스트 불가 → import-smoke만.

**Files:**
- Create: `ad_autotune/scripts/live_runner.py`
- Test: `ad_autotune/tests/test_live_runner_import.py`

**Interfaces:**
- Consumes: `autotune_core`(load_track_csv, _path_lateral_error), `ad_control`(build_profile, GovernedStanley).
- Produces: `LiveRunner(track, s_arr, seg, timeout, start).reset_and_arm()->bool`, `.drive(params: dict, save_csv: str|None)->dict|None`. drive 반환 dict 키: completed, diverged, progress_s, time_s, driving_score, max_cte, mean_cte, offtrack_s, overspeed_s. (rospy.init_node는 **호출자**가 미리 함.)

- [ ] **Step 1: import-smoke 테스트 작성**

```python
# ad_autotune/tests/test_live_runner_import.py
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

def test_live_runner_has_interface():
    import live_runner
    assert hasattr(live_runner, "LiveRunner")
    for m in ("reset_and_arm", "drive"):
        assert callable(getattr(live_runner.LiveRunner, m))
```

- [ ] **Step 2: 실행해 실패 확인**

Run: `cd ad_autotune && PYTHONNOUSERSITE=0 python3 -m pytest tests/test_live_runner_import.py -v`
Expected: FAIL (No module named live_runner).

- [ ] **Step 3: `live_runner.py` 구현** (auto_tune_live.py의 reset/drive를 클래스화)

```python
# ad_autotune/scripts/live_runner.py
# [transferable-to-heven_ad]
"""live_runner — MORAI 리셋 텔레포트 + governed-Stanley 세그먼트 주행.
auto_tune_live(v3)의 reset_and_arm/drive를 클래스화(로직 동일). 호출자가 rospy.init_node 선행."""
import math, os, sys


def _mods():
    try:
        import rospkg
        p = os.path.join(rospkg.RosPack().get_path("ad_autotune"), "scripts")
    except Exception:
        p = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, p)
    import autotune_core, ad_control
    return autotune_core, ad_control


class LiveRunner:
    def __init__(self, track, s_arr, seg, timeout, start):
        self.core, self.ctrl = _mods()
        self.track = track; self.s_arr = s_arr
        self.seg = seg; self.timeout = timeout; self.start = start
        import rospy
        from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, MultiEgoSetting
        self.rospy = rospy
        self._CtrlCmd = CtrlCmd; self._MultiEgoSetting = MultiEgoSetting
        self.pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        self.egopub = rospy.Publisher("/ego_setting", MultiEgoSetting, queue_size=1)
        self.st = {"x": None, "y": None, "th": 0.0, "v": 0.0, "t": 0.0}
        def cb(m):
            self.st["x"] = m.position.x; self.st["y"] = m.position.y
            self.st["th"] = math.radians(m.heading)
            self.st["v"] = math.hypot(m.velocity.x, m.velocity.y) * 3.6
            self.st["t"] = rospy.get_time()
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, cb, queue_size=10)
        try:
            from morai_msgs.srv import MoraiEventCmdSrv
            rospy.wait_for_service("/Service_MoraiEventCmd", timeout=5)
            self.event_srv = rospy.ServiceProxy("/Service_MoraiEventCmd", MoraiEventCmdSrv)
        except Exception:
            self.event_srv = None

    def _fresh(self):
        return self.st["x"] is not None and (self.rospy.get_time() - self.st["t"]) < 0.5

    def reset_and_arm(self):
        rospy = self.rospy; S = self.start
        m = self._MultiEgoSetting()
        m.number_of_ego_vehicle = 1; m.camera_index = 0; m.ego_index = [0]
        m.global_position_x = [S[0]]; m.global_position_y = [S[1]]; m.global_position_z = [S[2]]
        m.global_roll = [0.0]; m.global_pitch = [0.0]; m.global_yaw = [S[3]]
        m.velocity = [0.0]; m.gear = [4]; m.ctrl_mode = [16]
        m.steering_angle = [0.0]; m.vehicle_speed = [0.0]; m.turn_signal = [0]; m.brake_light = [False]
        t0 = rospy.get_time()
        while self.egopub.get_num_connections() == 0 and rospy.get_time() - t0 < 5:
            rospy.sleep(0.1)
        for _ in range(8):
            self.egopub.publish(m); rospy.sleep(0.1)
        t0 = rospy.get_time()
        while rospy.get_time() - t0 < 8:
            if self._fresh() and math.hypot(self.st["x"] - S[0], self.st["y"] - S[1]) < 3.0:
                break
            rospy.sleep(0.1)
        else:
            return False
        if self.event_srv:
            try:
                from morai_msgs.msg import EventInfo
                ev = EventInfo(); ev.option = 3; ev.ctrl_mode = 3; ev.gear = 4
                self.event_srv(ev)
            except Exception:
                pass
        rospy.sleep(0.5)
        return True

    def drive(self, p, save_path):
        rospy = self.rospy; track = self.track; s_arr = self.s_arr
        s, prof = self.ctrl.build_profile(track, a_lat=p["a_lat"])
        gs = self.ctrl.GovernedStanley(track, s, prof, p["lookahead"], p["gain_k"], p["k_soft"], p["pid_kp"])
        gs.reset(self.st["x"], self.st["y"])
        max_s = max_cte = sum_cte = 0.0; cnt = 0
        offtrack = overspeed = 0.0; diverged = False; complete_t = None
        stall_t = None; stale_since = None; samples = []
        rate = rospy.Rate(20); t0 = rospy.get_time(); prev_t = t0
        while not rospy.is_shutdown():
            now = rospy.get_time(); t = now - t0; dt = now - prev_t; prev_t = now
            if t > self.timeout:
                break
            if not self._fresh():
                stale_since = stale_since or now
                if now - stale_since > 3.0:
                    return None
                rospy.sleep(0.05); continue
            stale_since = None
            x, y, th, v = self.st["x"], self.st["y"], self.st["th"], self.st["v"]
            target = min(p["target_velocity_kph"],
                         5.0 + (p["target_velocity_kph"] - 5.0) * min(t / 4.0, 1.0))
            steer, thr, brk, info = gs.step(x, y, th, v, dt, target)
            cmd = self._CtrlCmd(); cmd.longlCmdType = 1
            cmd.accel = float(thr); cmd.brake = float(brk)
            cmd.steering = float(max(-1.0, min(1.0, steer / 0.698))); cmd.front_steer = float(steer)
            self.pub.publish(cmd)
            cte = self.core._path_lateral_error(track, x, y)
            near = info["near"]
            max_s = max(max_s, s_arr[near]); max_cte = max(max_cte, cte); sum_cte += cte; cnt += 1
            if cte > 1.5: offtrack += dt
            if v > 50: overspeed += dt
            samples.append((round(t, 2), round(x, 2), round(y, 2), round(v, 1), round(cte, 2), near))
            if v < 1.0 and t > 5:
                stall_t = stall_t or now
                if now - stall_t > 4.0:
                    diverged = True; break
            else:
                stall_t = None
            if cte > 12: diverged = True; break
            if s_arr[near] >= self.seg: complete_t = t; break
            rate.sleep()
        stop = self._CtrlCmd(); stop.longlCmdType = 1; stop.brake = 1.0
        for _ in range(4):
            self.pub.publish(stop); rospy.sleep(0.02)
        completed = complete_t is not None
        penalty = 5.0 * int(offtrack // 3.0) + (10.0 if overspeed > 0 else 0) + 10.0 * int(overspeed // 3.0)
        m = dict(completed=completed, diverged=diverged, progress_s=round(max_s, 1),
                 time_s=round(complete_t, 2) if completed else self.timeout,
                 driving_score=max(0.0, 100.0 - penalty), max_cte=round(max_cte, 2),
                 mean_cte=round(sum_cte / max(cnt, 1), 3), offtrack_s=round(offtrack, 1),
                 overspeed_s=round(overspeed, 1))
        if save_path:
            with open(save_path, "w") as f:
                f.write("t,x,y,v_kph,cte,near\n")
                for s_ in samples:
                    f.write(",".join(map(str, s_)) + "\n")
        return m
```

- [ ] **Step 4: 실행해 통과 확인**

Run: `cd ad_autotune && PYTHONNOUSERSITE=0 python3 -m pytest tests/test_live_runner_import.py -v`
Expected: 1 PASS. (morai_msgs import 실패하면 devel 소싱 필요: `source /home/taeyeong/heven_common_test_ws/devel/setup.bash` 후 재실행.)

- [ ] **Step 5: 커밋**

```bash
git add ad_autotune/scripts/live_runner.py ad_autotune/tests/test_live_runner_import.py
git commit -m "feat: live_runner — 리셋/주행 러너 클래스화(로직 v3 동일)"
```

---

### Task 4: `auto_tune_ab.py` — 오케스트레이터 (interleave→승자→full→재측정)

주행 러너를 **의존성 주입**받는 `run_ab`로 오케스트레이션을 순수화 → FakeRunner+in-memory 스터디로 유닛테스트. `main()`만 실제 ROS/LiveRunner 연결.

**Files:**
- Create: `ad_autotune/scripts/auto_tune_ab.py`
- Test: `ad_autotune/tests/test_auto_tune_ab.py`

**Interfaces:**
- Consumes: `ab_core`(suggest_params, objective_value, constraints_func, make_samplers, study_stats, pick_winner, WARMSTART), `live_runner.LiveRunner`.
- Produces:
  - `build_objective(runner, seg, timeout, out_dir) -> callable(trial)->float`
  - `run_ab(studies: dict, objective, n_smoke: int, budget) -> dict` (budget: `{"alive": callable()->bool, "time_left": callable()->float}`; 반환 `{"winner": str, "stats": dict, "n_full": int}`)
  - `enqueue_warmstart(studies)`
  - `final_remeasure(runner, top_params: list[dict], repeats: int, seg, timeout, out_dir) -> list[dict]`

- [ ] **Step 1: 실패 테스트 작성** (FakeRunner로 오케스트레이션만 검증)

```python
# ad_autotune/tests/test_auto_tune_ab.py
import sys, os, optuna
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import ab_core, auto_tune_ab

class FakeRunner:
    """reset 항상 성공. drive는 target_velocity_kph 높을수록 빨리 완주(단조)."""
    def reset_and_arm(self): return True
    def drive(self, p, save):
        v = p["target_velocity_kph"]
        return dict(completed=True, diverged=False, progress_s=400.4,
                    time_s=round(3600.0 / v, 2), driving_score=100.0,
                    max_cte=0.3, mean_cte=0.1, offtrack_s=0.0, overspeed_s=0.0)

def _studies(seed=1):
    s = ab_core.make_samplers(seed)
    return {"tpe": optuna.create_study(direction="minimize", sampler=s["tpe"]),
            "gp":  optuna.create_study(direction="minimize", sampler=s["gp"])}

def test_enqueue_warmstart_seeds_both():
    st = _studies()
    auto_tune_ab.enqueue_warmstart(st)
    for study in st.values():
        assert len(study.get_trials(deepcopy=False)) >= len(ab_core.WARMSTART)

def test_run_ab_interleaves_and_picks_winner():
    st = _studies()
    obj = auto_tune_ab.build_objective(FakeRunner(), seg=400, timeout=120, out_dir=None)
    budget = {"alive": lambda: True, "time_left": _countdown(6)}  # full 6 trial 후 종료
    res = auto_tune_ab.run_ab(st, obj, n_smoke=3, budget=budget)
    assert res["winner"] in ("tpe", "gp")
    assert st["tpe"].trials and st["gp"].trials          # 둘 다 smoke 돌았음
    # 완주가 목적 최소화라 best value는 유한, feasible 존재
    assert ab_core.study_stats(st[res["winner"]])["feasible"] > 0

def _countdown(n):
    box = {"n": n}
    def f():
        box["n"] -= 1; return float(box["n"])
    return f

def test_objective_sets_attrs_and_returns_float():
    st = _studies(); study = st["tpe"]
    obj = auto_tune_ab.build_objective(FakeRunner(), seg=400, timeout=120, out_dir=None)
    study.optimize(obj, n_trials=1)
    t = study.trials[0]
    assert isinstance(t.value, float)
    assert t.user_attrs.get("completed") is True
    assert "gain_k" in t.user_attrs
```

- [ ] **Step 2: 실행해 실패 확인**

Run: `cd ad_autotune && PYTHONNOUSERSITE=0 python3 -m pytest tests/test_auto_tune_ab.py -v`
Expected: FAIL (No module named auto_tune_ab).

- [ ] **Step 3: `auto_tune_ab.py` 구현**

```python
#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""auto_tune_ab — GP vs TPE interleave A/B 튜닝(제약목적: minimize time, 완주=제약).
smoke(각 n_smoke) → 완주율/median으로 승자 → 승자 full → top-K 재측정.
  python3 auto_tune_ab.py --hours 3 --seg 400 --timeout 120 --smoke 25
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


def build_objective(runner, seg, timeout, out_dir):
    import ab_core
    def objective(trial):
        p = ab_core.suggest_params(trial)
        for k, v in p.items():
            trial.set_user_attr(k, v)
        trial.set_user_attr("seg", seg)
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
    # smoke: interleave 한 라운드에 각 스터디 1 trial
    for _ in range(n_smoke):
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


def final_remeasure(runner, top_params, repeats, seg, timeout, out_dir):
    import ab_core
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
    args = ap.parse_args()

    ab_core, core = _mods()
    HERE = os.path.dirname(os.path.abspath(__file__))
    CSV = os.path.join(HERE, "..", "paths", "kcity_2025.csv")
    OUT = os.path.join(HERE, "..", "results", "live_tune")
    DB = os.path.join(OUT, "tune_ab.db")
    START = (7.5766, -279.0828, 28.5, 60.9)
    os.makedirs(OUT, exist_ok=True)

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
            storage=f"sqlite:///{DB}", load_if_exists=True, sampler=sam[name])
    enqueue_warmstart(studies)

    deadline = time.time() + args.hours * 3600
    objective = build_objective(runner, args.seg, args.timeout, OUT)
    budget = {"alive": lambda: not rospy.is_shutdown(),
              "time_left": lambda: deadline - time.time()}
    print(f"[ab] seg={args.seg} smoke={args.smoke}/sampler deadline {args.hours}h", flush=True)
    res = run_ab(studies, objective, args.smoke, budget)
    print(f"[ab] winner={res['winner']} stats={res['stats']} full={res['n_full']}", flush=True)

    # 재측정: 승자 feasible 중 목적값(작을수록 좋음) 상위 topk
    ws = studies[res["winner"]]
    fs = sorted(ab_core.feasible_trials(ws), key=lambda t: t.value)[:args.topk]
    top_params = [{**{k: t.user_attrs[k] for k, _, _, _ in ab_core.PARAM_SPECS}, "pid_kp": ab_core.PID_KP} for t in fs]
    finals = final_remeasure(runner, top_params, args.repeats, args.seg, args.timeout, OUT)
    with open(os.path.join(OUT, "best_ab.json"), "w") as f:
        json.dump({"winner": res["winner"], "stats": res["stats"], "final": finals}, f, indent=2)
    print(f"[ab] DONE. best={finals[0] if finals else None}", flush=True)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 실행해 통과 확인**

Run: `cd ad_autotune && PYTHONNOUSERSITE=0 python3 -m pytest tests/test_auto_tune_ab.py -v`
Expected: 3 PASS. (optuna 로그 억제하려면 `OPTUNA_LOG_LEVEL` 무시하고 그냥 통과 확인.)

- [ ] **Step 5: 전체 유닛테스트 재확인 + 커밋**

Run: `cd ad_autotune && PYTHONNOUSERSITE=0 python3 -m pytest tests/ -v`
Expected: Task1~4 테스트 전부 PASS (preflight 3 + ab_core 8 + live_runner_import 1 + auto_tune_ab 3).

```bash
git add ad_autotune/scripts/auto_tune_ab.py ad_autotune/tests/test_auto_tune_ab.py
git commit -m "feat: auto_tune_ab — interleave A/B 오케스트레이터 + 재측정 + 테스트"
```

---

### Task 5: 라이브 통합 — 스택 복구 + MORAI 재연결 + 짧은 A/B 스모크

유닛으로 못 잡는 실제 시뮬 연동을 확인. MORAI가 켜져 연결돼야 함(사용자 필요).

**Files:** 없음(운영 실행). 로그는 `results/live_tune/`.

**Interfaces:** Consumes: 위 전체.

- [ ] **Step 1: ROS 스택 복구 (죽어있음)**

```bash
source /opt/ros/noetic/setup.bash
source /home/taeyeong/heven_common_test_ws/devel/setup.bash
roscore &                       # 11311
sleep 2
export PYTHONPATH=/home/taeyeong/heven_common_test_ws/devel/lib/python3/dist-packages:$PYTHONPATH
roslaunch rosbridge_server rosbridge_websocket.launch &   # 9090
sleep 2
python3 /tmp/claude-1000/-home-taeyeong-heven-common-test-ws/28ca779a-c1a4-4aa9-8e85-0b381345a639/scratchpad/primer.py &
```
Expected: `rostopic list`에 `/Ego_topic`, `/ctrl_cmd`, `/ego_setting` 보임.

- [ ] **Step 2: MORAI 재연결 확인 (사용자)**

MORAI에서 ROS Connect. 확인: `rostopic hz /Ego_topic` → ~30Hz 흐름. 안 흐르면 **"MORAI 재연결 필요"** 보고하고 대기.

- [ ] **Step 3: 짧은 A/B 스모크 (각 3 trial, ~15분)**

Run:
```bash
cd /home/taeyeong/heven_common_test_ws/src/heven-common-test/ad_autotune/scripts
PYTHONNOUSERSITE=0 python3 auto_tune_ab.py --hours 0.3 --seg 400 --timeout 120 --smoke 3 --topk 2 --repeats 2
```
Expected: warm-start 6점이 두 스터디에 enqueue되고 완주 로그가 뜸. `[ab] winner=... stats=...` 출력. `results/live_tune/tune_ab.db` 생성, `kcity_seg400_tpe_v4_0000.json` 등 생성. reset/drive 실제 동작 확인.

- [ ] **Step 4: 결과 점검**

```bash
ls -t /home/taeyeong/heven_common_test_ws/src/heven-common-test/ad_autotune/results/live_tune/*_v4_*.json | head
cat /home/taeyeong/heven_common_test_ws/src/heven-common-test/ad_autotune/results/live_tune/best_ab.json
```
Expected: 완주 trial들이 있고 winner/stats/final이 채워짐. 완주율 0이면 → reset/mode/START 좌표 점검(스펙 리스크 참고), 사용자 보고.

- [ ] **Step 5: 3시간 본run 착수 (백그라운드) + 감시 cron 재무장**

Run (백그라운드):
```bash
cd /home/taeyeong/heven_common_test_ws/src/heven-common-test/ad_autotune/scripts
PYTHONNOUSERSITE=0 python3 auto_tune_ab.py --hours 3 --seg 400 --timeout 120 --smoke 25 --topk 4 --repeats 3
```
그리고 기존 감시 패턴대로 cron 재등록(스택/프로세스 감시, MORAI client_count=0 알림). 커밋할 코드 없음(운영).

---

### Task 6: 장기 무인 실행 견고화 (오래 돌아가도 안 죽고 안 오염)

장시간 돌리면 (a) MORAI가 잠깐 끊기고, (b) roscore/rosbridge/primer가 죽고, (c) 파이썬 프로세스 자체가 크래시할 수 있음. 3중 방어: **끊기면 trial 오염 대신 대기**(인-스크립트) + **프로세스 죽으면 재시작**(래퍼) + **스택 죽으면 재시작**(cron). SQLite resume이라 전부 이어감.

**Files:**
- Modify: `ad_autotune/scripts/auto_tune_ab.py` (sim 대기 가드 추가)
- Create: `ad_autotune/scripts/run_ab_forever.sh`
- Test: `ad_autotune/tests/test_wait_for_sim.py`

**Interfaces:**
- Produces: `wait_for_sim(runner, budget, poll=2.0) -> bool` (sim 신선해질 때까지 블록, deadline/shutdown이면 False). build_objective가 reset 전에 호출.

- [ ] **Step 1: 실패 테스트 작성** (FakeRunner로 대기 로직만)

```python
# ad_autotune/tests/test_wait_for_sim.py
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import auto_tune_ab

class R:
    def __init__(self, becomes_fresh_after): self.k = 0; self.n = becomes_fresh_after
    def _fresh(self): self.k += 1; return self.k >= self.n

def test_returns_true_when_sim_becomes_fresh():
    b = {"alive": lambda: True, "time_left": lambda: 100.0}
    assert auto_tune_ab.wait_for_sim(R(3), b, poll=0.0) is True

def test_returns_false_when_deadline_passes():
    box = {"n": 3}
    def tl():
        box["n"] -= 1; return float(box["n"])
    b = {"alive": lambda: True, "time_left": tl}
    assert auto_tune_ab.wait_for_sim(R(999), b, poll=0.0) is False   # 절대 fresh 안 됨 → deadline
```

- [ ] **Step 2: 실행해 실패 확인**

Run: `cd ad_autotune && PYTHONNOUSERSITE=0 python3 -m pytest tests/test_wait_for_sim.py -v`
Expected: FAIL (wait_for_sim 없음).

- [ ] **Step 3: `wait_for_sim` 추가 + build_objective 배선**

`auto_tune_ab.py`에 함수 추가:
```python
def wait_for_sim(runner, budget, poll=2.0):
    """sim이 신선(/Ego_topic 흐름)해질 때까지 블록. deadline/shutdown이면 False.
    장기 무인 중 MORAI 순단이 trial을 오염(disconnected)시키지 않게 대기로 흡수."""
    import time as _t
    while budget["alive"]() and budget["time_left"]() > 0:
        if runner._fresh():
            return True
        if poll:
            _t.sleep(poll)
    return False
```
`build_objective`의 objective 맨 앞(suggest_params 뒤, reset 앞)에 삽입:
```python
        if not wait_for_sim(runner, objective._budget):
            trial.set_user_attr("aborted", True)
            return ab_core.objective_value({"completed": False, "progress_s": 0.0, "max_cte": 0.0}, seg, timeout)
```
`build_objective` 시그니처에 budget 전달: `def build_objective(runner, seg, timeout, out_dir, budget=None)` 로 바꾸고 `objective._budget = budget or {"alive": lambda: True, "time_left": lambda: 1.0}` 설정. `main()`에서 `build_objective(runner, args.seg, args.timeout, OUT, budget)` 로 호출(budget는 studies 생성 뒤 정의된 것 사용 — main 순서 조정: budget 먼저 정의).

- [ ] **Step 4: 통과 확인 + 회귀**

Run: `cd ad_autotune && PYTHONNOUSERSITE=0 python3 -m pytest tests/ -v`
Expected: 전체 PASS (기존 auto_tune_ab 테스트도 budget 기본값으로 계속 통과).

- [ ] **Step 5: 프로세스 재시작 래퍼 작성**

```bash
# ad_autotune/scripts/run_ab_forever.sh
#!/usr/bin/env bash
# 파이썬이 죽어도 deadline까지 재시작(SQLite resume). MORAI/스택 감시는 cron이 담당.
set -u
HOURS="${1:-3}"; SMOKE="${2:-25}"
HERE="$(cd "$(dirname "$0")" && pwd)"
END=$(( $(date +%s) + $(printf '%.0f' "$(echo "$HOURS*3600" | bc)") ))
source /opt/ros/noetic/setup.bash
source /home/taeyeong/heven_common_test_ws/devel/setup.bash
while [ "$(date +%s)" -lt "$END" ]; do
  LEFT=$(( END - $(date +%s) ))
  H=$(echo "scale=3; $LEFT/3600" | bc)
  echo "[forever] restart, ${H}h left"
  PYTHONNOUSERSITE=0 python3 "$HERE/auto_tune_ab.py" --hours "$H" --seg 400 --timeout 120 --smoke "$SMOKE" || true
  sleep 10
done
echo "[forever] deadline reached"
```
`chmod +x ad_autotune/scripts/run_ab_forever.sh`.
(주의: 재시작 시 smoke가 다시 도는 걸 피하려면, 이미 각 스터디 trial 수가 smoke 이상이면 run_ab가 smoke 루프를 건너뛰도록 Step 3에서 `for _ in range(max(0, n_smoke - min(len(s.trials) for s in studies.values())))` 로 보정 — 재시작해도 smoke 중복 안 함.)

- [ ] **Step 6: 커밋**

```bash
chmod +x ad_autotune/scripts/run_ab_forever.sh
git add ad_autotune/scripts/auto_tune_ab.py ad_autotune/scripts/run_ab_forever.sh ad_autotune/tests/test_wait_for_sim.py
git commit -m "feat: 장기 무인 견고화 — sim순단 대기 가드 + 재시작 래퍼"
```

- [ ] **Step 7: 스택 감시 cron 재무장 (운영)**

Task5 스택 복구 후, 감시 cron 등록: 주기적으로 roscore(11311)/rosbridge(9090)/primer 살아있나 확인·재시작, `run_ab_forever.sh` 프로세스 살아있나 확인·재시작, MORAI client_count=0이면 "MORAI 재연결 필요" 알림. (기존 감시 패턴 재사용, 코드 커밋 없음.)

---

## Self-Review

**Spec coverage:**
- 목적함수 재구성(minimize+proxy+constraints) → Task2 `objective_value`/`constraints_func` ✅
- GP/TPE A/B interleave → Task4 `run_ab` ✅
- smoke 25 후 완주율→median 승자 → Task4 `run_ab`+`pick_winner`(Task2) ✅
- 승자 full → Task4 `run_ab` ✅
- top 3~5 재측정 → Task4 `final_remeasure` ✅
- pruner 없음 → 전 태스크에 pruner 미사용 ✅
- warm-start 6점 enqueue 양쪽 → Task4 `enqueue_warmstart` + Task2 `WARMSTART` ✅
- 탐색공간(v 25–55, gain_k log 등) → Task2 `PARAM_SPECS` ✅
- ad_control/auto_tune_live 불변 → 신규 파일만, Global Constraints 명시 ✅
- v3 DB 보존 → 신규 `tune_ab.db` + 신규 스터디명 ✅
- 리스크(GPSampler 가용성) → Task1 preflight ✅
- MORAI 재연결 선행 → Task5 ✅
- 장기 무인 실행(오래 돌아가도 안 죽고 안 오염) → Task6: sim순단 대기 가드 + 재시작 래퍼 + 스택 감시 cron ✅

**Placeholder scan:** 코드/커맨드/기대출력 모두 구체값. 플레이스홀더 없음.

**Type consistency:** `constraints_func`/`objective_value`/`study_stats`/`pick_winner` 시그니처가 Task2 정의와 Task4 사용에서 일치. drive 반환 dict 키가 live_runner(Task3)와 objective(Task4)에서 일치. `make_samplers`가 dict{"tpe","gp"} 반환 → studies 키와 일치.
