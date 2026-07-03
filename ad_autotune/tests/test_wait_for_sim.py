import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import auto_tune_ab


class R:
    def __init__(self, becomes_fresh_after):
        self.k = 0; self.n = becomes_fresh_after
    def _fresh(self):
        self.k += 1; return self.k >= self.n


def test_returns_true_when_sim_becomes_fresh():
    b = {"alive": lambda: True, "time_left": lambda: 100.0}
    assert auto_tune_ab.wait_for_sim(R(3), b, poll=0.0) is True


def test_returns_false_when_deadline_passes():
    box = {"n": 3}
    def tl():
        box["n"] -= 1; return float(box["n"])
    b = {"alive": lambda: True, "time_left": tl}
    assert auto_tune_ab.wait_for_sim(R(999), b, poll=0.0) is False   # 절대 fresh 안 됨 → deadline
