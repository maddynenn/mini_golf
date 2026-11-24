"""Tests for plan_path.fake_scan(): the fake lidar sees obstacles where they are.

Needs the ROS environment (source ~/ament_ws/install/setup.bash).
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # plan_path.py is in tests/
plan_path = pytest.importorskip('plan_path')
from builtin_interfaces.msg import Time  # noqa: E402


def hit_points(start, obstacles):
    """Map-frame points the fake scan reports, using the scan's own conventions."""
    scan = plan_path.fake_scan(start, obstacles, Time())
    x, y, yaw = start
    lx, ly = x + plan_path.LIDAR_X * math.cos(yaw), y + plan_path.LIDAR_X * math.sin(yaw)
    lyaw = yaw + plan_path.LIDAR_YAW
    return [(lx + r * math.cos(lyaw + scan.angle_min + i * scan.angle_increment),
             ly + r * math.sin(lyaw + scan.angle_min + i * scan.angle_increment))
            for i, r in enumerate(scan.ranges) if math.isfinite(r)]


@pytest.mark.parametrize('start', [(0.0, 0.0, 0.0), (5.0, -2.0, 1.57), (1.0, 1.0, -2.5)])
def test_hits_lie_on_obstacle_surface(start):
    obstacle = (start[0] + 1.0, start[1] + 0.3, 0.2)
    points = hit_points(start, [obstacle])
    assert len(points) > 5
    for px, py in points:
        assert math.hypot(px - obstacle[0], py - obstacle[1]) == pytest.approx(0.2, abs=1e-6)


def test_nearest_hit_is_front_of_obstacle():
    points = hit_points((0.0, 0.0, 0.0), [(1.2, 0.0, 0.2)])
    nearest = min(points, key=lambda p: math.hypot(*p))
    assert nearest == pytest.approx((1.0, 0.0), abs=0.01)


def test_no_obstacles_no_returns():
    scan = plan_path.fake_scan((0.0, 0.0, 0.0), [], Time())
    assert all(math.isinf(r) for r in scan.ranges)
