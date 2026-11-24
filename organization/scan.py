#!/usr/bin/env python3
"""Scan: look around the room with the head camera until the ball is found.

detection.find_ball() only sees what the camera is pointed at right now.
scan_for_ball() sweeps the head through a grid of pan/tilt poses (pan covers
about 310 degrees, three tilts cover near, mid and far floor), calling
find_ball() at each one. A hit is confirmed on a couple more frames before it
is trusted. If the whole sweep finds nothing, the base turns 90 degrees (to
cover the head's blind spot) and sweeps again.

Test it on its own (driver + camera running, robot stowed):

    python3 ~/mini_golf/organization/scan.py

From other code (after detection.setup(node) and moving.setup(node)):

    ball = scan.scan_for_ball(node)   # (x, y) in detection.FLOOR_FRAME, or None

The head is left pointing at the ball.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
import hello_helpers.hello_misc as hm

import detection
import moving

# --- Head sweep ---
PAN_MIN, PAN_MAX = -3.9, 1.5   # rad, joint_head_pan limits (stretch_description URDF)
PAN_STEP = 0.8                 # rad (~46 deg) max between pans; camera FOV ~69 deg, so views overlap
# Head camera is ~1.3 m up. Tilt -1.1 sees floor ~0.4-1.2 m away, -0.75 ~0.8-2.6 m,
# -0.45 ~1.4 m to the far wall.
TILTS = [-1.1, -0.75, -0.45]   # rad
SETTLE_TIME = 1.5              # s after each head move (camera shake, TF catching up).
                               # The sim runs slow: raise to ~4 if views look blurred/wrong.
FIRST_MOVE_TIME = 4.0          # s, the first move can be a big swing
FRAME_TIMEOUT = 3.0            # s to wait for a fresh camera frame

# --- Confirming a hit (rejects one-frame false positives) ---
CONFIRM_FRAMES = 3             # frames looked at in total, including the first hit
CONFIRM_HITS = 2               # how many of them must agree
CONFIRM_DIST = 0.15            # m, hits further apart than this don't agree

# --- Base turn when the sweep finds nothing ---
BASE_TURN = math.pi / 2        # rad, CCW
MAX_BASE_TURNS = 1             # 0 = head sweep only, never move the base
TURN_SPEED = 0.4               # rad/s


def scan_for_ball(node):
    """Sweep the head (and turn the base if needed). Return the ball's (x, y) or None."""
    log = node.get_logger()
    for turn in range(MAX_BASE_TURNS + 1):
        if turn > 0:
            log.info(f'scan: no ball found, turning the base {math.degrees(BASE_TURN):.0f} deg')
            _turn_base(node, BASE_TURN)
        node.switch_to_position_mode()   # head moves are only honoured in position mode
        time.sleep(1.0)
        ball = _sweep_head(node)
        if ball is not None:
            log.info(f'scan: ball at ({ball[0]:.2f}, {ball[1]:.2f}) in {detection.FLOOR_FRAME}')
            return ball
    log.warn('scan: ball not found anywhere')
    return None


def _sweep_head(node):
    """Visit every pan/tilt pose; return the first confirmed ball (x, y), or None."""
    log = node.get_logger()
    first = True
    for i, pan in enumerate(_pan_positions()):
        # Serpentine order (tilt up, then down at the next pan) keeps head moves short.
        tilts = TILTS if i % 2 == 0 else TILTS[::-1]
        for tilt in tilts:
            log.info(f'scan: looking at pan {pan:.2f}, tilt {tilt:.2f}')
            node.move_to_pose({'joint_head_pan': pan, 'joint_head_tilt': tilt}, blocking=True)
            time.sleep(FIRST_MOVE_TIME if first else SETTLE_TIME)
            first = False

            if not _fresh_frames(node):
                log.warn('scan: no fresh camera frame, skipping this view')
                continue
            ball = detection.find_ball(node)
            if ball is None:
                continue
            ball = _confirm(node, ball)
            if ball is not None:
                return ball
            log.info('scan: hit not confirmed on later frames, ignoring it')
    return None


def _pan_positions():
    """Evenly spaced pans from PAN_MAX to PAN_MIN, at most PAN_STEP apart."""
    n = math.ceil((PAN_MAX - PAN_MIN) / PAN_STEP)
    return [PAN_MAX - k * (PAN_MAX - PAN_MIN) / n for k in range(n + 1)]


def _fresh_frames(node, timeout=FRAME_TIMEOUT):
    """Drop the frames we have and wait for new ones taken after the head stopped."""
    node.latest_color = None
    node.latest_depth = None
    start = time.time()
    while time.time() - start < timeout:
        if node.latest_color is not None and node.latest_depth is not None:
            return True
        time.sleep(0.05)
    return False


def _confirm(node, first_hit):
    """Look at more frames; return the average position if enough agree, else None."""
    hits = [first_hit]
    for _ in range(CONFIRM_FRAMES - 1):
        if not _fresh_frames(node):
            continue
        p = detection.find_ball(node)
        if p is not None and math.dist(p, first_hit) <= CONFIRM_DIST:
            hits.append(p)
    if len(hits) < CONFIRM_HITS:
        return None
    return (sum(p[0] for p in hits) / len(hits), sum(p[1] for p in hits) / len(hits))


def _turn_base(node, angle, timeout=30.0):
    """Turn the base in place by angle (rad, CCW positive) using cmd_vel."""
    if not hasattr(node, 'cmd_vel'):
        moving.setup(node)
    node.switch_to_navigation_mode()
    time.sleep(1.0)

    (_, _, prev), _ = node.get_robot_floor_pose_xya()
    turned = 0.0
    start = time.time()
    while abs(angle - turned) > 0.05 and time.time() - start < timeout:
        (_, _, a), _ = node.get_robot_floor_pose_xya()
        turned += hm.angle_diff_rad(a, prev)
        prev = a
        err = angle - turned
        cmd = Twist()
        cmd.angular.z = math.copysign(min(TURN_SPEED, max(0.15, 1.5 * abs(err))), err)
        node.cmd_vel.publish(cmd)
        time.sleep(0.05)
    moving.stop_base(node)   # must fully stop before switching back to position mode


class ScanTest(hm.HelloNode):
    """Scan for the ball and print where it is."""

    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'scan_test', 'scan_test', wait_for_first_pointcloud=False)
        detection.setup(self)
        moving.setup(self)
        ball = scan_for_ball(self)
        if ball is None:
            self.get_logger().error('Ball not found')
        else:
            self.get_logger().info(f'Ball found at ({ball[0]:.2f}, {ball[1]:.2f}) '
                                   f'in {detection.FLOOR_FRAME}')
        rclpy.shutdown()


if __name__ == '__main__':
    ScanTest().main()
