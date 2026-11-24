#!/usr/bin/env python3
"""Moving: drive the base so the robot ends up lined up to hit the ball.

This file should take a ball position (from detection.py) and drive the robot
so the gripper sits next to the ball, facing the hole, then stop completely so
the swing isn't thrown off by a drifting base. drive_to() and stop_base() are
copied from mini-golf1.py and work as-is; line_up_with_ball() is the part that
still needs writing. Nav2 (see the parcs_stretch_nav2_WVH starter code) is
another option for longer moves.
"""
import math
import os
import sys
import time

from geometry_msgs.msg import Twist
import hello_helpers.hello_misc as hm

sys.path.insert(0, os.path.expanduser('~/mini_golf/navigation'))
import go_to_goal   # Nav2 goals in the map frame (same frame as detection.FLOOR_FRAME)

STOP_DISTANCE = 0.6     # m, how far from the ball to stop (tune with the arm's reach)


def setup(node):
    node.cmd_vel = node.create_publisher(Twist, '/stretch/cmd_vel', 10)
    go_to_goal.setup(node)


def robot_map_pose(node):
    """Robot (x, y, yaw) in the map frame, or None if localization isn't running."""
    return go_to_goal.map_pose(node)


def go_to_ball(node, ball_xy, stop_dist=STOP_DISTANCE):
    """Nav2 to stop_dist short of the ball, facing it. ball_xy is in the map frame.

    Returns True if Nav2 reached the goal.
    """
    log = node.get_logger()
    pose = robot_map_pose(node)
    if pose is None:
        log.error('go_to_ball: no robot pose in the map frame (is the navigation launch running?)')
        return False
    bx, by = ball_xy
    dx, dy = bx - pose[0], by - pose[1]
    dist = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)                 # face the ball
    if dist <= stop_dist:
        gx, gy = pose[0], pose[1]            # already close enough: just turn to face it
    else:
        gx, gy = bx - stop_dist * dx / dist, by - stop_dist * dy / dist
    log.info(f'go_to_ball: ball at ({bx:.2f}, {by:.2f}), going to ({gx:.2f}, {gy:.2f}, '
             f'{yaw:.2f}) in map')
    return go_to_goal.go_to(node, gx, gy, yaw)


SHOT_DISTANCE = 0.5     # m, how far behind the ball (on the hole -> ball line) the robot stops


def shot_pose(ball_xy, hole_xy, dist=SHOT_DISTANCE):
    """(x, y, yaw) dist behind the ball on the line through the hole and the ball,
    on the side away from the hole, facing the hole. All in the map frame."""
    dx, dy = hole_xy[0] - ball_xy[0], hole_xy[1] - ball_xy[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        raise ValueError('ball is on the hole')
    ux, uy = dx / d, dy / d                  # unit vector ball -> hole
    return (ball_xy[0] - dist * ux, ball_xy[1] - dist * uy, math.atan2(uy, ux))


def line_up_with_ball(node, ball_xy, hole_xy=None):
    """Drive to the far side of the ball from the hole, facing the hole, so the
    swing sends the ball toward the hole. ball_xy, hole_xy: map frame.

    Returns True if Nav2 reached the pose. Without a hole, falls back to driving
    up to the ball (go_to_ball).
    """
    if hole_xy is None:
        return go_to_ball(node, ball_xy, STOP_DISTANCE)
    x, y, yaw = shot_pose(ball_xy, hole_xy)
    node.get_logger().info(
        f'line_up_with_ball: ball ({ball_xy[0]:.2f}, {ball_xy[1]:.2f}), hole '
        f'({hole_xy[0]:.2f}, {hole_xy[1]:.2f}) -> going to ({x:.2f}, {y:.2f}, {yaw:.2f}) in map, '
        f'{SHOT_DISTANCE:.2f} m behind the ball')
    return go_to_goal.go_to(node, x, y, yaw)


def turn_to(node, heading, timeout=20.0):
    """Turn in place until facing heading (rad, floor frame).

    Pseudocode:
        loop until timeout:
            err = hm.angle_diff_rad(heading, current angle)
            if abs(err) < ~0.02: break
            publish Twist with angular.z = clamp(1.5 * err, -0.5, 0.5)
        stop_base(node)
    """
    pass


def drive_to(node, goal_xy, stop_dist, timeout=60.0):
    """Turn toward goal_xy and drive until within stop_dist."""
    gx, gy = goal_xy
    start = time.time()
    while time.time() - start < timeout:
        (x, y, a), _ = node.get_robot_floor_pose_xya()
        dist = math.hypot(gx - x, gy - y)
        heading_err = hm.angle_diff_rad(math.atan2(gy - y, gx - x), a)

        if dist <= stop_dist:
            break

        cmd = Twist()
        if abs(heading_err) > 0.1:
            # Mostly off-target: turn in place first
            cmd.angular.z = max(-0.5, min(0.5, 1.5 * heading_err))
        else:
            # Roughly facing it: drive forward, correcting the heading as we go
            cmd.linear.x = min(0.2, 0.5 * (dist - stop_dist) + 0.05)
            cmd.angular.z = 1.5 * heading_err
        node.cmd_vel.publish(cmd)
        time.sleep(0.05)
    else:
        node.get_logger().warn('drive_to timed out')

    stop_base(node)


def stop_base(node):
    """Keep sending stop commands until the base has really stopped."""
    for _ in range(20):                      # about 1 second of stop messages
        node.cmd_vel.publish(Twist())
        time.sleep(0.05)

    (px, py, pa), _ = node.get_robot_floor_pose_xya()
    start = time.time()
    while time.time() - start < 10.0:
        node.cmd_vel.publish(Twist())
        time.sleep(0.3)
        (x, y, a), _ = node.get_robot_floor_pose_xya()
        if math.hypot(x - px, y - py) < 0.002 and abs(a - pa) < 0.002:
            node.get_logger().info('Base stopped')
            return
        px, py, pa = x, y, a
    node.get_logger().warn('Base still moving after 10 s!')
