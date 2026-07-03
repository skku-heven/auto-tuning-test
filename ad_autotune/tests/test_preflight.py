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
