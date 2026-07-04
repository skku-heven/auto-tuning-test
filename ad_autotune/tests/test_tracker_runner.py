import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import ab_core, auto_tune_ab
import tracker_runner  # 모듈 임포트 자체는 ROS 불필요해야 함(rospy는 __init__에서만)


def test_pgrep_pattern_does_not_self_match():
    # 브래킷 패턴이어야 pgrep -f가 자기 프로세스(패턴 문자열 포함)를 안 잡음
    assert tracker_runner.TRACKER_PGREP.startswith("[")
    assert "ad_tracker" not in tracker_runner.TRACKER_PGREP


def test_build_launch_args_maps_all_params():
    p = {"lookahead": 3.2, "target_velocity_kph": 40.0, "gain_k": 1.1,
         "k_soft": 1.5, "a_lat": 2.0, "pid_kp": 0.3, "pid_ki": 0.0, "pid_kd": 0.01}
    args = tracker_runner.build_launch_args(p, "/tmp/path.csv")
    assert args[:3] == ["roslaunch", "ad_tracker", "ad_tracker.launch"]
    assert "csv_path:=/tmp/path.csv" in args
    assert "status_topic:=/Ego_topic" in args              # 튜닝은 ground-truth 속도
    for k, v in p.items():
        assert f"{k}:={v}" in args


def test_build_launch_args_skips_missing_keys():
    args = tracker_runner.build_launch_args({"lookahead": 3.0}, "c.csv")
    assert "lookahead:=3.0" in args
    assert not any(a.startswith("gain_k:=") for a in args)


class SegAwareRunner:
    """drive가 seg/timeout override를 받는 러너(TrackerRunner 시그니처 모사)."""
    def __init__(self, max_cte=0.3):
        self.max_cte = max_cte
        self.seen_seg = None
    def _fresh(self):
        return True
    def reset_and_arm(self):
        return True
    def drive(self, p, save, seg=None, timeout=None):
        self.seen_seg = seg
        return dict(completed=True, diverged=False, progress_s=seg or 400,
                    time_s=100.0, driving_score=100.0, max_cte=self.max_cte,
                    mean_cte=0.1, mean_cte_sq=0.01, offtrack_s=0.0, overspeed_s=0.0)


def test_final_remeasure_passes_fullcourse_seg_and_rechecks_cte():
    r = SegAwareRunner(max_cte=0.3)
    p = [{"lookahead": 3.0}]
    res = auto_tune_ab.final_remeasure(r, p, repeats=2, seg=1997.0, timeout=420, out_dir=None)
    assert r.seen_seg == 1997.0                            # 풀코스 seg가 runner까지 전달됨
    assert res[0]["feasible"] == 2

    bad = SegAwareRunner(max_cte=ab_core.CTE_MAX + 0.5)    # 완주했지만 cte 초과
    res2 = auto_tune_ab.final_remeasure(bad, p, repeats=2, seg=1997.0, timeout=420, out_dir=None)
    assert res2[0]["feasible"] == 0                        # 재측정에서도 cte 재검증
    assert res2[0]["worst_max_cte"] >= ab_core.CTE_MAX


def test_final_remeasure_compatible_with_legacy_runner():
    class LegacyRunner:                                     # drive(p, save)만 받는 구 러너
        def reset_and_arm(self):
            return True
        def drive(self, p, save):
            return dict(completed=True, time_s=50.0, max_cte=0.2)
    res = auto_tune_ab.final_remeasure(LegacyRunner(), [{"x": 1}], 1, 400, 120, None)
    assert res[0]["feasible"] == 1
