import sys, os, optuna
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import ab_core, auto_tune_ab


class FakeRunner:
    """reset 항상 성공. drive는 target_velocity_kph 높을수록 빨리 완주(단조). sim 항상 신선."""
    def _fresh(self):
        return True
    def reset_and_arm(self):
        return True
    def drive(self, p, save):
        v = p["target_velocity_kph"]
        return dict(completed=True, diverged=False, progress_s=400.4,
                    time_s=round(3600.0 / v, 2), driving_score=100.0,
                    max_cte=0.3, mean_cte=0.1, offtrack_s=0.0, overspeed_s=0.0)


def _studies(seed=1):
    s = ab_core.make_samplers(seed)
    return {"tpe": optuna.create_study(direction="minimize", sampler=s["tpe"]),
            "gp": optuna.create_study(direction="minimize", sampler=s["gp"])}


def _countdown(n):
    box = {"n": n}
    def f():
        box["n"] -= 1; return float(box["n"])
    return f


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
    assert ab_core.study_stats(st[res["winner"]])["feasible"] > 0


def test_run_balanced_catches_up_lagging_sampler():
    st = _studies()
    obj = auto_tune_ab.build_objective(FakeRunner(), seg=400, timeout=120, out_dir=None)

    # Simulate the real failure mode: TPE already has more trials than GP.
    st["tpe"].optimize(obj, n_trials=5)
    st["gp"].optimize(obj, n_trials=1)

    budget = {"alive": lambda: True, "time_left": lambda: 100.0}
    res = auto_tune_ab.run_balanced(st, obj, target_trials=6, budget=budget)

    assert res["target_reached"] is True
    assert len(st["tpe"].trials) == 6
    assert len(st["gp"].trials) == 6
    assert res["added"]["gp"] > res["added"]["tpe"]


def test_objective_sets_attrs_and_returns_float():
    st = _studies(); study = st["tpe"]
    obj = auto_tune_ab.build_objective(FakeRunner(), seg=400, timeout=120, out_dir=None)
    study.optimize(obj, n_trials=1)
    t = study.trials[0]
    assert isinstance(t.value, float)
    assert t.user_attrs.get("completed") is True
    assert "gain_k" in t.user_attrs


def test_wait_for_sim_returns_true_when_fresh():
    b = {"alive": lambda: True, "time_left": lambda: 100.0}
    assert auto_tune_ab.wait_for_sim(FakeRunner(), b, poll=0.0) is True
