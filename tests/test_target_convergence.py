from ros_robot_studio.runtime.controller_core import TargetConvergence


def test_same_feedback_cannot_count_as_multiple_samples():
    check = TargetConvergence({"joint": 1.0}, 0.002)
    assert not check.observe({"joint": 1.001}, 1, 0.0)
    for _ in range(10):
        assert not check.observe({"joint": 1.001}, 1, 1.0)
    assert check.samples == 1
    for sequence in range(2, 5):
        assert not check.observe({"joint": 1.001}, sequence, 0.1 * sequence)
    assert check.observe({"joint": 1.001}, 5, 0.5)


def test_transient_target_crossing_resets_stable_interval():
    check = TargetConvergence({"joint": 0.0}, 0.002)
    for sequence in range(4):
        assert not check.observe({"joint": 0.001}, sequence, sequence * 0.1)
    assert not check.observe({"joint": 0.003}, 4, 0.4)
    assert check.samples == 0
    for sequence in range(5, 9):
        assert not check.observe({"joint": 0.001}, sequence, sequence * 0.1)
    assert check.observe({"joint": 0.001}, 9, 0.9)
    assert abs(check.maximum_error - 0.001) < 1e-12


def test_sample_count_alone_and_backwards_time_do_not_converge():
    check = TargetConvergence({"joint": 0.0}, 0.002)
    for sequence in range(20):
        assert not check.observe({"joint": 0.0}, sequence, 0.01 * sequence)
    assert not check.observe({"joint": 0.0}, 20, 0.1)
    assert check.samples == 0
