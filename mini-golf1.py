#!/usr/bin/env python3
import math
import time
import rclpy
from geometry_msgs.msg import Twist
import hello_helpers.hello_misc as hm

GOAL_DISTANCE = 1.5   # must match the cube's pos in your scene XML
STOP_DISTANCE = 0.6           # stop this far from the cube (meters)

class MiniGolf(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'mini_golf', 'mini_golf', wait_for_first_pointcloud=False)
        self.cmd_vel = self.create_publisher(Twist, '/stretch/cmd_vel', 10)

        self.stow_the_robot()
        self.switch_to_navigation_mode()

        # Set the goal relative to where the robot is right now
        (x0, y0, a0), _ = self.get_robot_floor_pose_xya()
        goal_x = x0 + GOAL_DISTANCE * math.cos(a0)
        goal_y = y0 + GOAL_DISTANCE * math.sin(a0)
        self.get_logger().info(f'Start at ({x0:.2f}, {y0:.2f}), goal at ({goal_x:.2f}, {goal_y:.2f})')

        self.drive_to(goal_x, goal_y, STOP_DISTANCE)
        self.backswing()

        (x, y, a), _ = self.get_robot_floor_pose_xya()
        self.get_logger().info(f'Done. Now at ({x:.2f}, {y:.2f}), angle {a:.2f}')
        rclpy.shutdown()
        
    def drive_to(self, gx, gy, stop_dist, timeout=60.0):
        """Turn toward (gx, gy) and drive until within stop_dist."""
        start = time.time()
        while time.time() - start < timeout:
            (x, y, a), _ = self.get_robot_floor_pose_xya()
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
            self.cmd_vel.publish(cmd)
            time.sleep(0.05)
        else:
            self.get_logger().warn('drive_to timed out')

        self.stop_base()

    def stop_base(self):
        """Keep sending stop commands until the base has really stopped."""
        for _ in range(20):                      # about 1 second of stop messages
            self.cmd_vel.publish(Twist())
            time.sleep(0.05)

        (px, py, pa), _ = self.get_robot_floor_pose_xya()
        start = time.time()
        while time.time() - start < 10.0:
            self.cmd_vel.publish(Twist())
            time.sleep(0.3)
            (x, y, a), _ = self.get_robot_floor_pose_xya()
            if math.hypot(x - px, y - py) < 0.002 and abs(a - pa) < 0.002:
                self.get_logger().info('Base stopped')
                return
            px, py, pa = x, y, a
        self.get_logger().warn('Base still moving after 10 s!')

    def backswing(self):
        self.switch_to_position_mode()
        time.sleep(1.0)  # let the base settle after driving

        # Raise and extend the arm so the wrist has room to move
        self.move({'joint_lift': 0.6})
        self.move({'wrist_extension': 0.2})

        # Backswing: gripper pointing back, raised to horizontal
        self.move({'joint_wrist_yaw': -1.57}, wait=4.0)
        self.move({'joint_wrist_pitch': -0.5}, wait=4.0)

        # Downswing: swing down to vertical (this is where the ball gets hit)
        self.move({'joint_wrist_pitch': -2}, wait=4.0)

    def move(self, pose, wait=2.0):
        self.get_logger().info(f'Moving: {pose}')
        self.move_to_pose(pose, blocking=True)
        time.sleep(wait)
if __name__ == '__main__':
    node = MiniGolf()
    node.main()
