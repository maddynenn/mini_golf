#!/usr/bin/env python3
"""Swing: hit the ball with the Stretch's wrist/gripper.

Pulled out of backswing() in mini-golf1.py and split into stages so each one
can be tuned on its own:

    get_ready()       extend the arm, rotate the club out, then lower to swing height
    backswing()       rotate the gripper back to one side (wrist yaw)
    downswing()       swing side to side through the ball (this is the hit)
    follow_through()  bring the wrist back to a safe pose

The swing only uses joint_wrist_yaw (side to side). Wrist pitch (up/down) is
not moved.

The functions take the node as their first argument so main.py (or any other
HelloNode) can call them. Run this file on its own to test just the swing,
with the robot already parked next to the ball:

    python3 ~/mini_golf/organization/swing.py
"""
import time

import hello_helpers.hello_misc as hm

# --- Tunables (start from the values in mini-golf1.py) ---
HOME_LIFT = 0.6             # m, joint_lift home height (from mini-golf1.py)
INCH = 0.0254               # m
LIFT_HEIGHT = HOME_LIFT - 2 * INCH  # m, 2 in below home = 0.5492
ARM_EXTENSION = 0.2         # m, wrist_extension
BACKSWING_YAW = -1.57       # rad, joint_wrist_yaw at the top of the backswing
WRIST_YAW_OUT = 0.0         # rad, joint_wrist_yaw straight out from the arm (club clear of the robot)
IMPACT_YAW = 1.0            # rad, joint_wrist_yaw at the end of the swing (past center = follow-through)
DOWNSWING_DURATION = 0.5    # s, shorter = harder hit (if your hello_helpers supports it)
SETTLE_TIME = 1.0           # s, pause between stages so the arm stops shaking

# --- Basket limits ---
# The basket on the base is in the way, so the arm can't go all the way down.
# Placeholders: find the real values on the robot by lowering slowly
# (stretch_xbox_controller / small steps) and noting where it gets close.
# Every move goes through move(), which clamps to these.
MIN_LIFT = 0.4              # m, lowest joint_lift that clears the basket (TODO: measure)
MIN_WRIST_PITCH = -1.57     # rad, lowest joint_wrist_pitch at LIFT_HEIGHT (TODO: measure)


def clamp_to_limits(node, pose):
    """Keep the arm out of the basket. Logs a warning if anything gets clamped."""
    safe = dict(pose)
    if 'joint_lift' in safe and safe['joint_lift'] < MIN_LIFT:
        safe['joint_lift'] = MIN_LIFT
    if 'joint_wrist_pitch' in safe and safe['joint_wrist_pitch'] < MIN_WRIST_PITCH:
        safe['joint_wrist_pitch'] = MIN_WRIST_PITCH
    if safe != pose:
        node.get_logger().warn(f'Clamped {pose} -> {safe} to avoid the basket')
    return safe


def move(node, pose, wait=SETTLE_TIME, duration=None):
    """Blocking move, then pause. duration speeds up/slows down the move if supported."""
    pose = clamp_to_limits(node, pose)
    node.get_logger().info(f'Moving: {pose}')
    if duration is None:
        node.move_to_pose(pose, blocking=True)
    else:
        try:
            node.move_to_pose(pose, blocking=True, duration=duration)
        except TypeError:
            node.get_logger().warn('move_to_pose has no duration arg; using default speed')
            node.move_to_pose(pose, blocking=True)
    time.sleep(wait)


def get_ready(node, extend_first=True):
    """Lift to LIFT_HEIGHT and extend to ARM_EXTENSION.

    extend_first=True (the swing): the arm comes in high from startup with the
    club tucked inside, so go out first (extend the arm, rotate the club out),
    then down. startup.py passes False: right after stow the lift is low, over
    the basket, so it has to go up before the arm extends.
    """
    node.switch_to_position_mode()
    time.sleep(SETTLE_TIME)  # let the base settle after driving
    if extend_first:
        move(node, {'wrist_extension': ARM_EXTENSION})
        move(node, {'joint_wrist_yaw': WRIST_YAW_OUT})   # club out from inside the robot
        move(node, {'joint_lift': LIFT_HEIGHT})
    else:
        move(node, {'joint_lift': LIFT_HEIGHT})
        move(node, {'wrist_extension': ARM_EXTENSION})


def backswing(node):
    move(node, {'joint_wrist_yaw': BACKSWING_YAW})


def downswing(node):
    # TODO: this is the part that needs work. The original used a plain
    # blocking move with the default speed, which is too slow to hit well.
    move(node, {'joint_wrist_yaw': IMPACT_YAW}, duration=DOWNSWING_DURATION)


def follow_through(node):
    # Center the wrist before anything else moves.
    move(node, {'joint_wrist_yaw': 0.0})
    # TODO: stow the robot here if moving.py needs the arm out of the way


def swing(node):
    """Full swing. Assumes the robot is already lined up with the ball."""
    get_ready(node)
    backswing(node)
    downswing(node)
    follow_through(node)


class SwingTest(hm.HelloNode):
    """Run only the swing, for tuning."""

    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'swing_test', 'swing_test', wait_for_first_pointcloud=False)
        swing(self)
        self.get_logger().info('Swing done')


if __name__ == '__main__':
    node = SwingTest()
    node.main()
