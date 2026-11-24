#!/usr/bin/env python3
"""Drive the robot to (x, y, yaw) in the map frame: Nav2, then a precise final approach.

Needs mini_golf_nav.launch.py running and the start pose set in RViz.

    python3 ~/mini_golf/navigation/go_to_goal.py X Y YAW     (m, m, rad)

From other code (node is a running hm.HelloNode):

    import go_to_goal
    go_to_goal.setup(node)
    ok = go_to_goal.go_to(node, x, y, yaw)

1. Nav2 (Regulated Pure Pursuit) plans a path and follows it to within its
   goal tolerance (0.15 m / 0.25 rad, config/nav2_params.yaml), avoiding
   obstacles the lidar sees.
2. final_approach.py drives the last few cm and turns to the goal yaw, using
   the robot's pose from AMCL (TF map -> base_link). It stops and waits while
   something is in the way (gives up after BLOCKED_TIMEOUT).
3. If the robot then isn't within POS_REQ / YAW_REQ (e.g. AMCL corrected its
   estimate), the final approach is repeated, up to MAX_ATTEMPTS times.
"""
import math
import sys
import time

import rclpy
import hello_helpers.hello_misc as hm
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

import final_approach

POS_REQ = 0.05                 # m, required accuracy (relative to AMCL's estimate)
YAW_REQ = math.radians(3)      # rad
MAX_ATTEMPTS = 3               # final approach passes
# False = stop after Nav2 (within its 0.15 m / 0.25 rad goal tolerance) and skip
# final_approach.py. Disabled for now: on the robot it drove away from the goal.
USE_FINAL_APPROACH = False
NAV2_TIMEOUT = 300.0           # s
APPROACH_TIMEOUT = 60.0        # s, per final approach pass
BLOCKED_TIMEOUT = 10.0         # s, give up if the way stays blocked this long
POSE_MAX_AGE = 0.5             # s, stop if the pose is older than this
RATE = 20.0                    # Hz, final approach control rate

# Stop zone for the final approach: lidar points within STOP_DIST of base_link
# in the direction of travel and within STOP_HALF_WIDTH to the side.
STOP_DIST = 0.35               # m
STOP_HALF_WIDTH = 0.22         # m

CMD_VEL_TOPIC = '/stretch/cmd_vel'
SCAN_TOPIC = '/scan_filtered'


def setup(node):
    """Create the publisher, subscriber and action client go_to() needs."""
    node.nav_cmd_vel = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
    node.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    node.nav_scan = None
    node.create_subscription(LaserScan, SCAN_TOPIC,
                             lambda msg: setattr(node, 'nav_scan', msg),
                             qos_profile_sensor_data)


def go_to(node, x, y, yaw):
    """Drive to (x, y, yaw) in the map frame. Returns True on success."""
    log = node.get_logger()
    goal = (x, y, yaw)
    node.switch_to_navigation_mode()
    try:
        if not nav2_go_to(node, goal):
            return False
        if not USE_FINAL_APPROACH:
            stop(node)
            pose = map_pose(node)
            if pose is not None:
                dist = math.hypot(x - pose[0], y - pose[1])
                yaw_err = abs(final_approach.wrap(yaw - pose[2]))
                log.info(f'Nav2 done (final approach disabled): {dist * 100:.1f} cm, '
                         f'{math.degrees(yaw_err):.1f} deg from goal')
            return True
        for attempt in range(1, MAX_ATTEMPTS + 1):
            if not final_approach_pass(node, goal):
                return False
            stop(node)
            pose = map_pose(node)
            if pose is None:
                return False
            dist = math.hypot(x - pose[0], y - pose[1])
            yaw_err = abs(final_approach.wrap(yaw - pose[2]))
            log.info(f'Final approach {attempt}: {dist * 100:.1f} cm, '
                     f'{math.degrees(yaw_err):.1f} deg from goal')
            if dist <= POS_REQ and yaw_err <= YAW_REQ:
                return True
        log.error(f'Not within {POS_REQ * 100:.0f} cm / {math.degrees(YAW_REQ):.0f} deg '
                  f'after {MAX_ATTEMPTS} attempts')
        return False
    finally:
        stop(node)


def nav2_go_to(node, goal):
    """Send the goal to Nav2 and wait for the result. Returns True on success."""
    log = node.get_logger()
    if not node.nav_client.wait_for_server(timeout_sec=10.0):
        log.error('Nav2 navigate_to_pose action server not available')
        return False

    msg = NavigateToPose.Goal()
    msg.pose = PoseStamped()
    msg.pose.header.frame_id = 'map'
    msg.pose.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x, msg.pose.pose.position.y = goal[0], goal[1]
    msg.pose.pose.orientation.z = math.sin(goal[2] / 2)
    msg.pose.pose.orientation.w = math.cos(goal[2] / 2)

    log.info(f'Nav2: going to ({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f})')
    goal_handle = wait_for(node.nav_client.send_goal_async(msg), 10.0)
    if goal_handle is None or not goal_handle.accepted:
        log.error('Nav2 rejected the goal')
        return False
    try:
        result = wait_for(goal_handle.get_result_async(), NAV2_TIMEOUT)
    except KeyboardInterrupt:
        goal_handle.cancel_goal_async()
        raise
    if result is None:
        log.error('Nav2 timed out, cancelling')
        goal_handle.cancel_goal_async()
        return False
    if result.status != GoalStatus.STATUS_SUCCEEDED:
        log.error(f'Nav2 failed (status {result.status})')
        return False
    return True


def final_approach_pass(node, goal):
    """Run final_approach.command() until done. Returns False on a problem."""
    log = node.get_logger()
    start = time.time()
    blocked_since = None
    while time.time() - start < APPROACH_TIMEOUT:
        pose = map_pose(node)
        if pose is None:
            return False
        cmd = final_approach.command(pose, goal)
        if cmd is None:
            return True
        v, w = cmd
        if v != 0.0 and path_blocked(node, v):
            if blocked_since is None:
                log.warn('Obstacle in the way, waiting')
                blocked_since = time.time()
            elif time.time() - blocked_since > BLOCKED_TIMEOUT:
                log.error('Still blocked, giving up')
                return False
            v, w = 0.0, 0.0
        else:
            blocked_since = None
        publish(node, v, w)
        time.sleep(1.0 / RATE)
    log.error('Final approach timed out')
    return False


def map_pose(node):
    """Robot (x, y, yaw) in the map frame from TF, or None if unavailable/stale."""
    try:
        t = node.tf2_buffer.lookup_transform('map', 'base_link', Time())
    except Exception as e:  # tf2 lookup/extrapolation errors
        node.get_logger().error(f'No map -> base_link transform: {e}')
        return None
    age = (node.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
    if age > POSE_MAX_AGE:
        node.get_logger().error(f'Robot pose is {age:.1f} s old')
        return None
    q = t.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return t.transform.translation.x, t.transform.translation.y, yaw


def path_blocked(node, v):
    """True if a lidar point is in the stop zone ahead (v > 0) or behind (v < 0)."""
    scan = node.nav_scan
    if scan is None:
        node.get_logger().warn('No lidar scan received, treating the way as blocked')
        return True
    try:
        t = node.tf2_buffer.lookup_transform('base_link', scan.header.frame_id, Time())
    except Exception as e:
        node.get_logger().warn(f'No base_link -> {scan.header.frame_id} transform: {e}')
        return True
    q = t.transform.rotation
    lidar_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    lx, ly = t.transform.translation.x, t.transform.translation.y
    direction = 1.0 if v > 0 else -1.0
    for i, r in enumerate(scan.ranges):
        if not scan.range_min <= r <= scan.range_max:
            continue  # inf / nan / out of range
        a = lidar_yaw + scan.angle_min + i * scan.angle_increment
        px, py = lx + r * math.cos(a), ly + r * math.sin(a)  # point in base_link
        if 0.0 < direction * px <= STOP_DIST and abs(py) <= STOP_HALF_WIDTH:
            return True
    return False


def publish(node, v, w):
    cmd = Twist()
    cmd.linear.x, cmd.angular.z = float(v), float(w)
    node.nav_cmd_vel.publish(cmd)


def stop(node, duration=1.0):
    """Send zero velocity for a while so the base comes to a full stop."""
    end = time.time() + duration
    while time.time() < end:
        publish(node, 0.0, 0.0)
        time.sleep(1.0 / RATE)


def wait_for(future, timeout):
    """Wait for a future completed by the HelloNode's executor thread."""
    end = time.time() + timeout
    while not future.done():
        if time.time() > end:
            return None
        time.sleep(0.05)
    return future.result()


class GoToGoal(hm.HelloNode):
    def main(self, x, y, yaw):
        hm.HelloNode.main(self, 'go_to_goal', 'go_to_goal', wait_for_first_pointcloud=False)
        setup(self)
        ok = go_to(self, x, y, yaw)
        self.get_logger().info('Reached goal' if ok else 'FAILED to reach goal')
        rclpy.shutdown()
        return ok


if __name__ == '__main__':
    if len(sys.argv) != 4:
        sys.exit('usage: go_to_goal.py X Y YAW   (map frame: m, m, rad)')
    x, y, yaw = (float(a) for a in sys.argv[1:])
    sys.exit(0 if GoToGoal().main(x, y, yaw) else 1)
