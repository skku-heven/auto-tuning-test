import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))


def test_live_runner_has_interface():
    import live_runner
    assert hasattr(live_runner, "LiveRunner")
    for m in ("reset_and_arm", "drive"):
        assert callable(getattr(live_runner.LiveRunner, m))
