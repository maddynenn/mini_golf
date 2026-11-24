#!/usr/bin/env python3
"""Startup: everything main.py does before detection.

    home -> stow -> initial swing position -> gripper outward -> open the claw
    for 2 s -> close it -> raise the arm to the top -> gripper to the front
    -> another ~85 deg inward so the club is inside

Run it on its own to test just this part (driver running, nothing else needed):

    python3 ~/mini_golf/organization/startup.py

main.py calls startup.run(node).
"""
import rclpy
import hello_helpers.hello_misc as hm

import swing

GRIPPER_OPEN = 0.4        # rad, joint_gripper_finger_left (bigger = wider; ~0.6 is fully open)
GRIPPER_CLOSED = 0.0      # rad, fingers together without squeezing (negative squeezes)
GRIPPER_OPEN_TIME = 2.0   # s to hold the claw open
LIFT_TOP = 1.05           # m, joint_lift near the top (max ~1.10)
# joint_wrist_yaw directions (checked against the robot's URDF; the arm sticks out the robot's right side):
WRIST_YAW_OUTWARD = 0.0   # rad, gripper straight out from the arm, away from the robot (club has room)
WRIST_YAW_FRONT = 1.57    # rad, gripper toward the robot's front (3.4 = stow, all the way in)
WRIST_YAW_INSIDE = WRIST_YAW_FRONT + 1.48   # rad, another ~85 deg inward (= 3.05) so the club is inside


def startup_routine(node):
    """Initial swing position, gripper outward, open the claw 2 s, close it, raise the arm,
    rotate the gripper to the front, then another ~85 deg inward."""
    log = node.get_logger()
    log.info('Startup: going to the initial swing position')
    swing.get_ready(node, extend_first=False)   # just stowed (lift low): lift first, then extend

    # Stow leaves the gripper tucked in, where the robot blocks the club: face it out first.
    log.info('Startup: turning the gripper outward')
    swing.move(node, {'joint_wrist_yaw': WRIST_YAW_OUTWARD})

    log.info(f'Startup: opening the claw for {GRIPPER_OPEN_TIME:.0f} s')
    swing.move(node, {'joint_gripper_finger_left': GRIPPER_OPEN}, wait=GRIPPER_OPEN_TIME)
    log.info('Startup: closing the claw')
    swing.move(node, {'joint_gripper_finger_left': GRIPPER_CLOSED})

    log.info('Startup: raising the arm to the top')
    swing.move(node, {'joint_lift': LIFT_TOP})
    log.info('Startup: rotating the gripper to the front')
    swing.move(node, {'joint_wrist_yaw': WRIST_YAW_FRONT})
    log.info('Startup: rotating the gripper another ~85 deg inward (club inside)')
    swing.move(node, {'joint_wrist_yaw': WRIST_YAW_INSIDE})


def run(node):
    """Home, stow, then the startup routine."""
    node.get_logger().info('Startup: homing (re-homes every joint, ~30 s)')
    node.home_the_robot()
    node.get_logger().info('Startup: stowing')
    node.stow_the_robot()
    startup_routine(node)
    node.get_logger().info('Startup: done')


class StartupTest(hm.HelloNode):
    """Run only the startup part, for testing."""

    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'startup_test', 'startup_test', wait_for_first_pointcloud=False)
        run(self)
        rclpy.shutdown()


if __name__ == '__main__':
    StartupTest().main()
