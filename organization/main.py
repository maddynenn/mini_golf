#!/usr/bin/env python3
"""Runs the full shot: detect the ball, drive to it, swing.

    python3 ~/mini_golf/organization/main.py
"""
import math
import os

import cv2
import rclpy
from cv_bridge import CvBridge
import hello_helpers.hello_misc as hm

import detection
import moving
import scan
import startup
import swing

GOAL_DISTANCE = 1.5   # m, fallback ball position (straight ahead) until detection works
OUT_DIR = os.path.dirname(os.path.abspath(__file__))   # where debug_color/mask.png go
_bridge = CvBridge()

def save_debug_images(node):
    """Save what the head camera sees now + the white mask (same as detection_test.py)."""
    if node.latest_color is None:
        node.get_logger().error('No color frame to save. Is the camera running? '
                                '(ros2 launch stretch_core d435i_low_resolution.launch.py)')
        return
    bgr = _bridge.imgmsg_to_cv2(node.latest_color, desired_encoding='bgr8')
    cv2.imwrite(os.path.join(OUT_DIR, 'debug_color.png'), bgr)
    cv2.imwrite(os.path.join(OUT_DIR, 'debug_mask.png'), detection.white_mask(bgr))
    node.get_logger().info(f'Saved debug_color.png and debug_mask.png to {OUT_DIR}')


class MiniGolf(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'mini_golf', 'mini_golf', wait_for_first_pointcloud=False)
        detection.setup(self)
        moving.setup(self)

        startup.run(self)       # home, stow, claw open/close, arm up, gripper to the front

        # 1. Detection
        ball = scan.scan_for_ball(self)   # sweeps the head (and base) until it sees the ball
        save_debug_images(self)           # head is still pointed at the ball (or the last view)
        self.get_logger().info(f'Detection result: ball = {ball}')
        hole = detection.find_hole(self)
        if ball is None:
            x0, y0, a0 = moving.robot_map_pose(self)
            ball = (x0 + GOAL_DISTANCE * math.cos(a0), y0 + GOAL_DISTANCE * math.sin(a0))
            self.get_logger().info(f'No ball detected, assuming it is at ({ball[0]:.2f}, {ball[1]:.2f})')

        # 2. Moving
        if not moving.line_up_with_ball(self, ball, hole):
            self.get_logger().error('Could not drive to the ball, not swinging')
            rclpy.shutdown()
            return

        # 3. Swing
        swing.swing(self)

        (x, y, a), _ = self.get_robot_floor_pose_xya()
        self.get_logger().info(f'Shot done. Now at ({x:.2f}, {y:.2f}), angle {a:.2f}')
        rclpy.shutdown()


if __name__ == '__main__':
    node = MiniGolf()
    node.main()
