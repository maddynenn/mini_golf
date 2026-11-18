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
import time

from geometry_msgs.msg import Twist
import hello_helpers.hello_misc as hm

STOP_DISTANCE = 0.6     # m, how far from the ball to stop (tune with the arm's reach)


def setup(node):
    node.cmd_vel = node.create_publisher(Twist, '/stretch/cmd_vel', 10)


def line_up_with_ball(node, ball_xy, hole_xy=None):
    """Drive so the robot is next to the ball and the swing sends it toward the hole.

    Pseudocode:
        if hole_xy is None:
            # No hole yet: just drive toward the ball like mini-golf1.py did
            drive_to(node, ball_xy, STOP_DISTANCE)
            return

        direction = unit vector from ball to hole
        # The swing hits sideways/behind the robot, so work out where the base
        # has to be for the gripper to sit behind the ball on that line:
        target_xy = ball_xy - direction * (offset from base to gripper)
        target_heading = heading that points the swing along direction
        drive_to(node, target_xy, stop_dist=0.02)
        turn_to(node, target_heading)
        stop_base(node)
    """
    node.switch_to_navigation_mode()
    drive_to(node, ball_xy, STOP_DISTANCE)


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
