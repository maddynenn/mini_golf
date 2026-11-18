#!/usr/bin/env python3
"""Runs the full shot: detect the ball, drive to it, swing.

    python3 ~/mini_golf/organization/main.py
"""
import math

import rclpy
import hello_helpers.hello_misc as hm

import detection
import moving
import swing

GOAL_DISTANCE = 1.5   # m, fallback ball position (straight ahead) until detection works


class MiniGolf(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'mini_golf', 'mini_golf', wait_for_first_pointcloud=False)
        detection.setup(self)
        moving.setup(self)

        self.stow_the_robot()

        # 1. Detection
        ball = detection.find_ball(self)
        hole = detection.find_hole(self)
        if ball is None:
            (x0, y0, a0), _ = self.get_robot_floor_pose_xya()
            ball = (x0 + GOAL_DISTANCE * math.cos(a0), y0 + GOAL_DISTANCE * math.sin(a0))
            self.get_logger().info(f'No ball detected, assuming it is at ({ball[0]:.2f}, {ball[1]:.2f})')

        # 2. Moving
        moving.line_up_with_ball(self, ball, hole)

        # 3. Swing
        swing.swing(self)

        (x, y, a), _ = self.get_robot_floor_pose_xya()
        self.get_logger().info(f'Shot done. Now at ({x:.2f}, {y:.2f}), angle {a:.2f}')
        rclpy.shutdown()


if __name__ == '__main__':
    node = MiniGolf()
    node.main()
