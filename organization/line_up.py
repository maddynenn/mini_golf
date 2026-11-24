#!/usr/bin/env python3
"""Line-up test: find the ball, then drive behind it facing the hole. No startup
routine and no swing.

Needs the navigation launch running (start pose set in RViz) and the head camera:

    python3 ~/mini_golf/organization/line_up.py

Skip detection and use a ball position you give (map frame, m), e.g. one read
with RViz "Publish Point":

    python3 ~/mini_golf/organization/line_up.py --ball 28.5 10.8

The hole is detection.HOLE; the robot stops moving.SHOT_DISTANCE behind the ball
on the hole -> ball line (see moving.line_up_with_ball).
"""
import argparse
import math

import rclpy
import hello_helpers.hello_misc as hm

import detection
import moving
import scan
from main import save_debug_images


class LineUpTest(hm.HelloNode):
    def __init__(self, ball=None):
        hm.HelloNode.__init__(self)
        self.given_ball = ball

    def main(self):
        hm.HelloNode.main(self, 'line_up_test', 'line_up_test', wait_for_first_pointcloud=False)
        moving.setup(self)
        log = self.get_logger()

        if self.given_ball is not None:
            ball = self.given_ball
            log.info(f'Using the given ball position ({ball[0]:.2f}, {ball[1]:.2f})')
        else:
            detection.setup(self)
            self.stow_the_robot()            # arm in before the head sweep and driving
            ball = scan.scan_for_ball(self)
            save_debug_images(self)
            log.info(f'Detection result: ball = {ball}')
            if ball is None:
                log.error('No ball found, not moving')
                rclpy.shutdown()
                return False

        hole = detection.find_hole(self)
        ok = moving.line_up_with_ball(self, ball, hole)

        pose = moving.robot_map_pose(self)
        if pose is not None:
            x, y, yaw = pose
            to_hole = math.atan2(hole[1] - y, hole[0] - x)
            off = math.degrees((to_hole - yaw + math.pi) % (2 * math.pi) - math.pi)
            log.info(f'{"Lined up" if ok else "FAILED to line up"}: robot at ({x:.2f}, {y:.2f}, '
                     f'{yaw:.2f}), {math.hypot(ball[0] - x, ball[1] - y):.2f} m from the ball, '
                     f'heading {off:+.0f} deg off pointing at the hole')
        rclpy.shutdown()
        return ok


if __name__ == '__main__':
    p = argparse.ArgumentParser(description='Find the ball and line up behind it (no swing).')
    p.add_argument('--ball', nargs=2, type=float, metavar=('X', 'Y'),
                   help='skip detection: ball position in the map frame (m)')
    a = p.parse_args()
    LineUpTest(tuple(a.ball) if a.ball else None).main()
