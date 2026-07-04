import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import ab_core


class FakeTrial:
    def __init__(self, attrs=None):
        self.user_attrs = attrs or {}
        self.calls = []
    def suggest_float(self, name, lo, hi, log=False):
        self.calls.append((name, lo, hi, log)); return lo
    def set_user_attr(self, k, v):
        self.user_attrs[k] = v


def test_param_specs_ranges():
    d = {n: (lo, hi, log) for n, lo, hi, log in ab_core.PARAM_SPECS}
    assert d["target_velocity_kph"][:2] == (25.0, 48.0)   # 48 상한(50 리밋 오버슈트 마진)
    assert d["gain_k"][2] is True                          # log scale
    assert d["lookahead"][:2] == (1.5, 5.5)
    assert d["pid_kp"][:2] == (0.1, 0.8)                   # 종방향 kp도 튜닝 대상


def test_warmstart_within_ranges():
    d = {n: (lo, hi) for n, lo, hi, _ in ab_core.PARAM_SPECS}
    assert len(ab_core.WARMSTART) == 6
    for w in ab_core.WARMSTART:
        for n, (lo, hi) in d.items():
            assert lo <= w[n] <= hi, f"{n}={w[n]} out of [{lo},{hi}]"


def test_suggest_params_uses_specs():
    t = FakeTrial(); p = ab_core.suggest_params(t)
    assert set(p) == {"lookahead", "target_velocity_kph", "gain_k", "k_soft", "a_lat",
                      "pid_kp", "pid_ki", "pid_kd"}
    assert p["pid_ki"] == 0.0 and p["pid_kd"] == 0.01      # ki/kd 작게 고정
    assert ("gain_k", 0.4, 3.0, True) in t.calls
    assert any(c[0] == "pid_kp" for c in t.calls)          # pid_kp는 suggest됨


def test_objective_completed_is_time_plus_cte_and_overspeed():
    # 완주: cost = time + W_CTE*mean_cte_sq + W_OVERSPEED*overspeed_s
    m = {"completed": True, "time_s": 48.7, "mean_cte_sq": 0.09, "overspeed_s": 0.0}
    assert abs(ab_core.objective_value(m, 400, 120) - (48.7 + ab_core.W_CTE * 0.09)) < 1e-6
    # overspeed 있으면 더 나빠짐(큼)
    m2 = dict(m, overspeed_s=2.0)
    assert ab_core.objective_value(m2, 400, 120) > ab_core.objective_value(m, 400, 120)
    # 추종 나쁘면(cte² 큼) 더 나빠짐
    m3 = dict(m, mean_cte_sq=0.5)
    assert ab_core.objective_value(m3, 400, 120) > ab_core.objective_value(m, 400, 120)


def test_objective_incomplete_worse_than_feasible_and_monotonic():
    near = {"completed": False, "progress_s": 380.0, "max_cte": 1.0}
    far = {"completed": False, "progress_s": 100.0, "max_cte": 1.0}
    v_near = ab_core.objective_value(near, 400, 120)
    v_far = ab_core.objective_value(far, 400, 120)
    # 어떤 feasible cost보다도 큼(완주는 time~120 + W_CTE*1 + ... 수준)
    feasible_max = 120 + ab_core.W_CTE * ab_core.CTE_MAX ** 2 + ab_core.W_OVERSPEED * 120
    assert v_near > feasible_max and v_far > feasible_max
    assert v_near < v_far                                   # 더 진행할수록 좋음(작음)


def test_constraints_feasible_only_completed_and_within_cte():
    ok = FakeTrial({"completed": True, "max_cte": 0.5})
    assert ab_core.constraints_func(ok) == [0.0]
    over = FakeTrial({"completed": True, "max_cte": ab_core.CTE_MAX + 0.1})
    assert ab_core.constraints_func(over) == [1.0]          # cte 초과 → infeasible
    assert ab_core.constraints_func(FakeTrial({"completed": False})) == [1.0]
    assert ab_core.constraints_func(FakeTrial({"reset_failed": True})) == [1.0]
    assert ab_core.constraints_func(FakeTrial({"disconnected": True})) == [1.0]
    assert ab_core.constraints_func(FakeTrial({"aborted": True})) == [1.0]
    # max_cte 누락된 완주 trial → infeasible (feasible_trials와 기본값 일치)
    assert ab_core.constraints_func(FakeTrial({"completed": True})) == [1.0]


def test_pick_winner_prefers_feasible_then_lower_cost():
    stats = {"tpe": {"feasible": 5, "median_cost": 60.0, "best_cost": 40.0},
             "gp": {"feasible": 8, "median_cost": 70.0, "best_cost": 45.0}}
    assert ab_core.pick_winner(stats) == "gp"               # 완주율 우선
    stats2 = {"tpe": {"feasible": 8, "median_cost": 55.0, "best_cost": 40.0},
              "gp": {"feasible": 8, "median_cost": 70.0, "best_cost": 45.0}}
    assert ab_core.pick_winner(stats2) == "tpe"             # 동률 → cost 낮은쪽
    stats3 = {"tpe": {"feasible": 0, "median_cost": None, "best_cost": None},
              "gp": {"feasible": 1, "median_cost": 90.0, "best_cost": 90.0}}
    assert ab_core.pick_winner(stats3) == "gp"


def test_make_samplers_types():
    import optuna
    s = ab_core.make_samplers(20260702)
    assert isinstance(s["tpe"], optuna.samplers.TPESampler)
    assert isinstance(s["gp"], optuna.samplers.GPSampler)
