"""Simple Stretch routine: drive forward, drive backward, spin 360 deg CCW, lift the arm.

Base motion uses /stretch/cmd_vel (sim must be in navigation mode).
The 360 turn is timed: turn CCW for TURN_TIME seconds, then move on.
The lift uses the /stretch_controller/follow_joint_trajectory action.
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

LINEAR_SPEED = 0.15      # m/s
DRIVE_TIME = 3.0         # s  (~0.45 m each way)
ANGULAR_SPEED = 0.4      # rad/s, positive = counterclockwise
TURN_TIME = 15.0         # s
LIFT_TARGET = 0.9        # m, joint_lift position (range ~0.0 - 1.1)


class StretchRoutine(Node):

    def __init__(self):
        super().__init__('stretch_routine')
        self.cmd_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.traj_client = ActionClient(
            self, FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory')

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_for(self, twist, seconds):
        # Publish at 10 Hz; the driver stops the base if commands go stale.
        end = time.time() + seconds
        while time.time() < end:
            self.cmd_pub.publish(twist)
            self.spin_for(0.1)

    def stop(self):
        self.publish_for(Twist(), 1.0)

    def drive(self, speed, seconds):
        t = Twist()
        t.linear.x = speed
        self.publish_for(t, seconds)
        self.stop()

    def turn_ccw(self, seconds):
        t = Twist()
        t.angular.z = ANGULAR_SPEED
        self.publish_for(t, seconds)
        self.stop()

    def lift_arm(self, target):
        self.get_logger().info('Waiting for trajectory action server...')
        if not self.traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('follow_joint_trajectory server not available')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_lift']
        pt = JointTrajectoryPoint()
        pt.positions = [target]
        pt.time_from_start = Duration(sec=3)
        goal.trajectory.points = [pt]

        send_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error('Lift goal rejected')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        self.get_logger().info('Lift complete')
        return True

    def run(self):
        self.spin_for(1.0)  # let publishers/subscribers connect
        self.get_logger().info('Driving forward')
        self.drive(LINEAR_SPEED, DRIVE_TIME)
        self.get_logger().info('Driving backward')
        self.drive(-LINEAR_SPEED, DRIVE_TIME)
        self.get_logger().info(f'Turning counterclockwise for {TURN_TIME:.0f} s')
        self.turn_ccw(TURN_TIME)
        self.get_logger().info(f'Lifting arm to {LIFT_TARGET} m')
        self.lift_arm(LIFT_TARGET)
        self.get_logger().info('Routine finished')


def main():
    rclpy.init()
    node = StretchRoutine()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
