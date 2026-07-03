# [transferable-to-heven_ad]
"""ab_core — Optuna A/B 튜닝의 순수 결정로직(시뮬 불필요, 테스트 가능).
path-tracking 중심: 시간(주) + cte²(추종 보조) + overspeed penalty.
제약(infeasible): 미완주 OR max_cte>CTE_MAX. 미완주는 연속 proxy(절벽 없음)."""
import statistics

PID_KP = 0.3
# (name, low, high, log)  — target_velocity는 50kph 넘지 않게 상한 50
PARAM_SPECS = [
    ("lookahead", 1.5, 5.5, False),
    ("target_velocity_kph", 25.0, 50.0, False),
    ("gain_k", 0.4, 3.0, True),
    ("k_soft", 0.5, 3.0, False),
    ("a_lat", 1.0, 3.0, False),
]

# --- 목적/제약 튜닝 상수 (여기만 바꾸면 됨) ---
CTE_MAX = 1.0          # m. max_cte가 이 값 초과하면 즉시 infeasible(하드 제약)
V_LIMIT = 50.0         # kph. 이 속도 초과 시간(overspeed_s)에 penalty
W_CTE = 50.0           # mean(cte²) 가중 — 추종 보조(시간 대비 부차)
W_OVERSPEED = 5.0      # overspeed_s(초) 당 penalty — 50kph 초과 강하게 억제
_PROXY_BASE = 2000.0   # 미완주 proxy 하한(어떤 feasible cost보다도 큼)
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
    """minimize (path-tracking). 완주 → time_s + W_CTE·mean(cte²) + W_OVERSPEED·overspeed_s.
    미완주 → _PROXY_BASE + (seg-progress) + max_cte·0.5 (연속, 진행할수록 작음, 모든 feasible보다 큼)."""
    if m.get("completed"):
        return float(m["time_s"]
                     + W_CTE * m.get("mean_cte_sq", 0.0)
                     + W_OVERSPEED * m.get("overspeed_s", 0.0))
    return float(_PROXY_BASE + (seg - m.get("progress_s", 0.0)) + m.get("max_cte", 0.0) * 0.5)


def constraints_func(trial):
    """≤0 이면 feasible. 완주 AND max_cte≤CTE_MAX 만 feasible.
    reset실패/disconnect/aborted/미완주/과대이탈 → infeasible."""
    a = trial.user_attrs
    if a.get("reset_failed") or a.get("disconnected") or a.get("aborted"):
        return [1.0]
    if not a.get("completed"):
        return [1.0]
    if a.get("max_cte", 1e9) > CTE_MAX:      # 누락 시 infeasible (feasible_trials와 기본값 일치)
        return [1.0]
    return [0.0]


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
    """완주 AND max_cte≤CTE_MAX 인 trial(= constraints_func feasible)."""
    import optuna
    out = []
    for t in study.trials:
        if t.state != optuna.trial.TrialState.COMPLETE:
            continue
        a = t.user_attrs
        if a.get("reset_failed") or a.get("disconnected") or a.get("aborted"):
            continue
        if a.get("completed") and a.get("max_cte", 1e9) <= CTE_MAX:
            out.append(t)
    return out


def study_stats(study):
    """feasible 개수 + feasible cost(objective, 작을수록 좋음)의 median/best."""
    fs = feasible_trials(study)
    costs = [t.value for t in fs if t.value is not None]
    return {"feasible": len(fs),
            "median_cost": statistics.median(costs) if costs else None,
            "best_cost": min(costs) if costs else None}


def pick_winner(stats):
    def key(name):
        s = stats[name]; mc = s["median_cost"]
        return (s["feasible"], -(mc if mc is not None else 1e18))
    return max(stats, key=key)
