"""Tests for go_to_goal.path_blocked() with a fake scan and TF.

Needs the ROS environment (source ~/ament_ws/install/setup.bash).
"""
import math
import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
go_to_goal = pytest.importorskip('go_to_goal')

LIDAR_YAW = 3.1416  # Stretch URDF joint_laser: the lidar is mounted facing backwards
LIDAR_X = 0.004


class FakeBuffer:
    def lookup_transform(self, target, source, time):
        q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(LIDAR_YAW / 2), w=math.cos(LIDAR_YAW / 2))
        tr = SimpleNamespace(x=LIDAR_X, y=0.0, z=0.1664)
        return SimpleNamespace(transform=SimpleNamespace(translation=tr, rotation=q))


def fake_node(points):
    """Node whose scan contains one return per (x, y) point given in base_link."""
    n = 360
    ranges = [math.inf] * n
    for x, y in points:
        # base_link -> lidar frame
        lx, ly = x - LIDAR_X, y
        a = math.atan2(ly, lx) - LIDAR_YAW
        i = round(((a + math.pi) % (2 * math.pi)) / (2 * math.pi / n))
        ranges[i % n] = math.hypot(lx, ly)
    scan = SimpleNamespace(header=SimpleNamespace(frame_id='laser'), ranges=ranges,
                           angle_min=-math.pi, angle_increment=2 * math.pi / n,
                           range_min=0.15, range_max=12.0)
    logger = SimpleNamespace(warn=print, error=print)
    return SimpleNamespace(nav_scan=scan, tf2_buffer=FakeBuffer(), get_logger=lambda: logger)


def test_obstacle_ahead_blocks_forward_only():
    node = fake_node([(0.3, 0.0)])
    assert go_to_goal.path_blocked(node, 0.05)
    assert not go_to_goal.path_blocked(node, -0.05)


def test_obstacle_behind_blocks_reverse_only():
    node = fake_node([(-0.3, 0.05)])
    assert go_to_goal.path_blocked(node, -0.05)
    assert not go_to_goal.path_blocked(node, 0.05)


def test_obstacles_outside_zone_ignored():
    node = fake_node([(0.6, 0.0), (0.2, 0.4), (-0.2, -0.4)])
    assert not go_to_goal.path_blocked(node, 0.05)
    assert not go_to_goal.path_blocked(node, -0.05)


def test_no_scan_counts_as_blocked():
    node = fake_node([])
    node.nav_scan = None
    assert go_to_goal.path_blocked(node, 0.05)
