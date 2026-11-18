"""Tests for final_approach.command() against a simple simulated robot.

    cd ~/mini_golf/navigation && python3 -m pytest tests
"""
import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import final_approach as fa  # noqa: E402

DT = 0.05  # s, go_to_goal.py's control period


def run(start, goal, speed_scale=1.0, noise=0.0, max_time=60.0):
    """Drive a differential-drive robot with command() until done.

    speed_scale < 1 models a base that moves slower than commanded; noise adds
    random error (m, rad) to the pose the controller sees.
    Returns (final true pose, time taken), or (pose, None) on timeout.
    """
    x, y, yaw = start
    t = 0.0
    while t < max_time:
        seen = (x + random.gauss(0, noise), y + random.gauss(0, noise),
                yaw + random.gauss(0, noise))
        cmd = fa.command(seen, goal)
        if cmd is None:
            return (x, y, yaw), t
        v, w = cmd
        v, w = v * speed_scale, w * speed_scale
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = fa.wrap(yaw + w * DT)
        t += DT
    return (x, y, yaw), None


def errors(pose, goal):
    return math.hypot(goal[0] - pose[0], goal[1] - pose[1]), abs(fa.wrap(goal[2] - pose[2]))


# Starts like those Nav2 leaves: within its 0.15 m / 0.25 rad goal tolerance,
# plus some worse ones.
STARTS = [(0.15, 0.0, 0.0), (-0.15, 0.0, 0.25), (0.0, 0.15, -0.25), (0.1, -0.1, 3.0),
          (-0.3, 0.2, 1.5), (0.0, 0.0, 2.0), (0.01, 0.01, 0.0)]


@pytest.mark.parametrize('start', STARTS)
@pytest.mark.parametrize('goal_yaw', [0.0, 1.0, -2.5])
def test_reaches_goal(start, goal_yaw):
    goal = (0.0, 0.0, goal_yaw)
    pose, t = run(start, goal)
    assert t is not None, 'timed out'
    dist, yaw_err = errors(pose, goal)
    assert dist <= fa.POS_TOL + 0.002
    assert yaw_err <= fa.YAW_TOL + 0.002


@pytest.mark.parametrize('start', STARTS)
def test_slow_base(start):
    # The simulator's base reaches ~30% of the commanded speed.
    goal = (0.0, 0.0, 0.5)
    pose, t = run(start, goal, speed_scale=0.3)
    assert t is not None, 'timed out'
    dist, yaw_err = errors(pose, goal)
    assert dist <= 0.05 and yaw_err <= math.radians(3)


@pytest.mark.parametrize('seed', range(5))
def test_noisy_pose(seed):
    # 5 mm / 0.3 deg of pose noise must still end inside ±5 cm / ±3 deg.
    random.seed(seed)
    goal = (0.0, 0.0, 1.0)
    pose, t = run((0.12, -0.08, 0.2), goal, noise=0.005)
    assert t is not None, 'timed out'
    dist, yaw_err = errors(pose, goal)
    assert dist <= 0.05 and yaw_err <= math.radians(3)


def test_reverses_to_goal_behind():
    v, w = fa.command((0.1, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert v < 0 and w == pytest.approx(0.0)


def test_done_at_goal():
    assert fa.command((0.0, 0.0, 0.0), (0.005, 0.0, 0.01)) is None
