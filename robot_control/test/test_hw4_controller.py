import math

from robot_control.hw4_controller import ArriveController

DT = 0.05
HALF_TRACK = 0.3 / 2


def wheel_cmds(v, w):
    return (v - w * HALF_TRACK, v + w * HALF_TRACK)


def run(disturb=None, shift=-0.12):
    """Drive a unicycle through the controller; disturb(ctrl) returns True once per entry to the pre-verify pause."""
    ctrl = ArriveController([(0, 0, 0), (1.0, 0, 0)], wheel_cmds, 0.55, 3.49, 1.0)
    pose = [0.0, 0.0, 0.0]
    t, armed = 0.0, True
    for _ in range(3000):
        if ctrl.done:
            break
        t += DT
        in_pause = ctrl.phase == 'settle' and ctrl.after == 'verify'
        if in_pause and armed and disturb is not None and disturb():
            pose[0] += shift
        armed = not in_pause
        left, right = ctrl.step(tuple(pose), t)
        v, w = (left + right) / 2, (right - left) / 0.3
        pose[0] += v * math.cos(pose[2]) * DT
        pose[1] += v * math.sin(pose[2]) * DT
        pose[2] += w * DT
    return ctrl, pose


def test_redrives_after_a_late_correction_shifts_the_estimate():
    fired = []
    ctrl, pose = run(lambda: not fired and not fired.append(1))
    assert ctrl.done
    assert ctrl.redrives == 1
    assert math.dist(pose[:2], (1.0, 0.0)) < 0.04


def test_no_redrive_when_already_at_the_goal():
    ctrl, pose = run()
    assert ctrl.done
    assert ctrl.redrives == 0
    assert math.dist(pose[:2], (1.0, 0.0)) < 0.04


def test_gives_up_after_max_redrives():
    ctrl, _ = run(lambda: True)
    assert ctrl.done
    assert ctrl.redrives == 2


def test_no_turn_around_for_an_overshoot():
    fired = []
    ctrl, _ = run(lambda: not fired and not fired.append(1), shift=0.12)
    assert ctrl.done
    assert ctrl.redrives == 0
