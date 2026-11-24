#!/usr/bin/env python3
"""Ask Nav2's planner for a path from a start pose to a goal pose and save a picture.

Needs plan_only.launch.py running (no robot or simulator).

    python3 ~/mini_golf/navigation/plan_path.py SX SY SYAW GX GY GYAW \
        [--obstacle X Y R]... [--png FILE]

Poses are in the map frame (m, m, rad). --obstacle adds a round obstacle that is
NOT on the map (centre X, Y, radius R in m); it is reported by a fake lidar scan
from the robot at the start pose, the way the real lidar would see it, so it
must be within 1.5 m of the start (the costmap's obstacle_max_range).

Prints whether a path was found, its length, how much longer it is than the
straight line, the planning time and the highest costmap cost along it, and
saves a picture of the costmap, path, start, goal and obstacles.

How it works (plan()):
  1. Publishes a static TF odom -> base_link at the start pose (map -> odom is
     fixed by the launch file), so the costmap's robot is at the start. Static,
     not a moving transform: the costmap's lidar input waits for a moving
     transform at each scan's timestamp and was seen to stall on it, reusing
     one old scan for every later case.
  2. Clears the global costmap (removes obstacles from earlier runs), then
     publishes fake lidar scans on /scan that hit the --obstacle circles, and
     waits for a costmap update that shows them (wait_for_costmap()).
  3. Calls Nav2's compute_path_to_pose with the start pose given explicitly -
     the same planner, costmap and settings the robot uses.
"""
import argparse
import math
import os
import threading
import time
import warnings

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

NAV_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(NAV_DIR, 'planning_results')

LIDAR_X, LIDAR_YAW = 0.004, 3.1416   # base_link -> laser, as in plan_only.launch.py
OBSTACLE_MAX_RANGE = 1.5             # m, global costmap obstacle_layer.scan.obstacle_max_range
ROBOT_RADIUS = 0.27                  # m, costmap robot_radius
INSCRIBED, LETHAL = 99, 100          # costmap values (OccupancyGrid scale) = collision
COSTMAP_UPDATE_PERIOD = 1.2          # s, a bit over the global costmap's 1 Hz update
COSTMAP_TIMEOUT = 10.0               # s, max wait for added obstacles to appear


class PlanTester(Node):
    def __init__(self):
        super().__init__('plan_path')
        self.start = (0.0, 0.0, 0.0)
        self.obstacles = []
        self.costmap = None
        self.tf_pub = StaticTransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', lambda m: setattr(self, 'costmap', m),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.clear = self.create_client(ClearEntireCostmap,
                                        '/global_costmap/clear_entirely_global_costmap')
        self.set_start(self.start)
        self.create_timer(0.1, self.publish_scan)

    def set_start(self, start):
        """Put the robot at start: static TF odom -> base_link (map -> odom is identity)."""
        self.start = tuple(float(v) for v in start)
        x, y, yaw = self.start
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf_pub.sendTransform(t)

    def publish_scan(self):
        self.scan_pub.publish(fake_scan(self.start, self.obstacles,
                                        self.get_clock().now().to_msg()))


def fake_scan(start, obstacles, stamp, beams=720):
    """LaserScan (frame 'laser') from the robot at start that hits the circles in obstacles."""
    x, y, yaw = start
    lx, ly = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
    scan = LaserScan()
    scan.header.stamp, scan.header.frame_id = stamp, 'laser'
    scan.angle_min, scan.angle_max = -math.pi, math.pi
    scan.angle_increment = 2 * math.pi / beams
    scan.range_min, scan.range_max = 0.15, 12.0
    scan.ranges = [math.inf] * beams           # inf = no return (clears the costmap)
    for i in range(beams):
        a = yaw + LIDAR_YAW + scan.angle_min + i * scan.angle_increment
        dx, dy = math.cos(a), math.sin(a)
        for cx, cy, r in obstacles:
            # Ray-circle intersection: nearest t >= 0 with |l + t*d - c| = r
            fx, fy = lx - cx, ly - cy
            b = fx * dx + fy * dy
            disc = b * b - (fx * fx + fy * fy - r * r)
            if disc >= 0:
                t = -b - math.sqrt(disc)
                if t > scan.range_min:
                    scan.ranges[i] = min(scan.ranges[i], t)
    return scan


def pose_msg(node, pose):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = pose[0], pose[1]
    p.pose.orientation.z, p.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
    return p


def wait_for(future, timeout):
    end = time.time() + timeout
    while not future.done():
        if time.time() > end:
            return None
        time.sleep(0.05)
    return future.result()


def cost_at(costmap, x, y):
    """Costmap value at map point (x, y): 0 free .. 99 inscribed, 100 lethal, -1 unknown."""
    info = costmap.info
    c = int((x - info.origin.position.x) / info.resolution)
    r = int((y - info.origin.position.y) / info.resolution)
    if not (0 <= c < info.width and 0 <= r < info.height):
        return -1
    return costmap.data[r * info.width + c]


def wait_for_costmap(node, start, obstacles):
    """Wait for a costmap made after the obstacles were added, showing all of them.

    Returns (costmap, seen) where seen[i] says if obstacle i is in it. Gives up
    waiting for the obstacles after COSTMAP_TIMEOUT and returns what it has.
    """
    # Only costmaps from at least one full update after now count (older
    # messages may predate the clear or the fake scans).
    after = node.get_clock().now().nanoseconds + COSTMAP_UPDATE_PERIOD * 1e9
    deadline = time.time() + COSTMAP_TIMEOUT
    while True:
        costmap = node.costmap
        if costmap is not None and Time.from_msg(costmap.header.stamp).nanoseconds > after:
            # The lidar only sees the side facing the robot: check that surface point.
            seen = []
            for cx, cy, r in obstacles:
                d = math.hypot(cx - start[0], cy - start[1])
                fx, fy = cx - r * (cx - start[0]) / d, cy - r * (cy - start[1]) / d
                seen.append(cost_at(costmap, fx, fy) >= INSCRIBED)
            if all(seen) or time.time() > deadline:
                return costmap, seen
        elif time.time() > deadline:
            raise RuntimeError('no new /global_costmap/costmap received')
        time.sleep(0.1)


def plan(node, start, goal, obstacles=()):
    """Plan from start to goal with the given extra obstacles. Returns a result dict."""
    log = node.get_logger()
    obstacles = list(obstacles)
    for cx, cy, r in obstacles:
        if math.hypot(cx - start[0], cy - start[1]) - r > OBSTACLE_MAX_RANGE:
            log.warn(f'Obstacle at ({cx}, {cy}) is more than {OBSTACLE_MAX_RANGE} m from the '
                     'start: the lidar would not report it, so it will not be in the costmap')

    # 1-2. Robot at the start, costmap cleared, then the fake obstacles seen.
    node.obstacles = []
    node.set_start(start)
    time.sleep(0.5)
    if not node.clear.wait_for_service(timeout_sec=10.0):
        raise RuntimeError('costmap clear service not available - is plan_only.launch.py running?')
    wait_for(node.clear.call_async(ClearEntireCostmap.Request()), 10.0)
    node.obstacles = obstacles
    costmap, seen = wait_for_costmap(node, start, obstacles)

    # 3. Ask the planner.
    if not node.planner.wait_for_server(timeout_sec=30.0):
        raise RuntimeError('compute_path_to_pose not available - is plan_only.launch.py running?')
    goal_msg = ComputePathToPose.Goal()
    goal_msg.start, goal_msg.goal = pose_msg(node, start), pose_msg(node, goal)
    goal_msg.use_start = True
    handle = wait_for(node.planner.send_goal_async(goal_msg), 10.0)
    response = wait_for(handle.get_result_async(), 60.0) if handle and handle.accepted else None
    path = [(p.pose.position.x, p.pose.position.y) for p in response.result.path.poses] \
        if response is not None else []

    result = {'start': start, 'goal': goal, 'obstacles': obstacles, 'obstacles_seen': seen,
              'found': len(path) > 1, 'path': path, 'costmap': costmap}
    if result['found']:
        result['length'] = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        result['straight'] = math.dist(start[:2], goal[:2])
        t = response.result.planning_time
        result['planning_time'] = t.sec + t.nanosec * 1e-9
        result['max_cost'] = max(cost_at(costmap, px, py) for px, py in path)
        result['start_gap'] = math.dist(path[0], start[:2])
        result['end_gap'] = math.dist(path[-1], goal[:2])
        if obstacles:
            result['obstacle_clearance'] = min(math.dist(p, (cx, cy)) - r
                                               for p in path for cx, cy, r in obstacles)
    return result


def summary(result):
    s, g = result['start'], result['goal']
    lines = [f"start ({s[0]:.2f}, {s[1]:.2f}, {s[2]:.2f})  goal ({g[0]:.2f}, {g[1]:.2f}, {g[2]:.2f})"]
    for (cx, cy, r), seen in zip(result['obstacles'], result['obstacles_seen']):
        lines.append(f"obstacle ({cx:.2f}, {cy:.2f}) r={r:.2f}: "
                     f"{'in costmap' if seen else 'NOT in costmap'}")
    if not result['found']:
        lines.append('NO PATH FOUND')
        return '\n'.join(lines)
    lines.append(f"path found: {result['length']:.2f} m "
                 f"(straight line {result['straight']:.2f} m, "
                 f"x{result['length'] / max(result['straight'], 1e-6):.2f}), "
                 f"{len(result['path'])} poses, planned in {result['planning_time'] * 1000:.0f} ms")
    lines.append(f"highest cost along path: {result['max_cost']} "
                 f"({'COLLISION' if result['max_cost'] >= INSCRIBED else 'ok'}; "
                 f"{INSCRIBED}+ = robot would touch an obstacle)")
    lines.append(f"path ends {result['end_gap']:.2f} m from the goal "
                 f"(starts {result['start_gap']:.2f} m from the start)")
    if 'obstacle_clearance' in result:
        lines.append(f"closest the path centre gets to an added obstacle: "
                     f"{result['obstacle_clearance']:.2f} m (robot radius {ROBOT_RADIUS} m)")
    return '\n'.join(lines)


def save_png(result, filename, title=''):
    """Picture of the costmap around the start/goal/path, with the path drawn on."""
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np

    cm = result['costmap']
    info = cm.info
    data = np.array(cm.data, dtype=np.int16).reshape(info.height, info.width)
    rgb = np.ones(data.shape + (3,))
    inflated = (data > 0) & (data < INSCRIBED)
    # inflation: pale yellow (low cost) -> orange (high cost)
    rgb[inflated] = np.array([1.0, 0.97, 0.75]) - (data[inflated, None] / 98.0) * [0.0, 0.42, 0.75]
    rgb[data == INSCRIBED] = [0.75, 0.1, 0.1]
    rgb[data == LETHAL] = [0.0, 0.0, 0.0]
    rgb[data < 0] = [0.75, 0.75, 0.75]
    x0, y0, res = info.origin.position.x, info.origin.position.y, info.resolution

    pts = [result['start'][:2], result['goal'][:2]] + result['path'] + \
        [(cx, cy) for cx, cy, _ in result['obstacles']]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    pad = 2.0
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(rgb, origin='lower', interpolation='nearest',
              extent=[x0, x0 + info.width * res, y0, y0 + info.height * res])
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    if result['path']:
        ax.plot([p[0] for p in result['path']], [p[1] for p in result['path']], 'r-', lw=2,
                label='planned path')
    for pose, colour, label in ((result['start'], 'green', 'start'),
                                (result['goal'], 'blue', 'goal')):
        ax.add_patch(plt.Circle(pose[:2], ROBOT_RADIUS, fill=False, color=colour, lw=1.5))
        ax.arrow(pose[0], pose[1], 0.45 * math.cos(pose[2]), 0.45 * math.sin(pose[2]),
                 width=0.04, color=colour, length_includes_head=True, label=label)
    for cx, cy, r in result['obstacles']:
        ax.add_patch(plt.Circle((cx, cy), r, fill=False, color='magenta', ls='--', lw=1.5))
    ax.plot([], [], 'm--', label='added obstacle (true shape)')
    ax.set_title(title + ('\n' if title else '') +
                 (f"path {result['length']:.2f} m, max cost {result['max_cost']}"
                  if result['found'] else 'NO PATH FOUND'))
    ax.set_xlabel('map x (m)\n\nblack: wall / obstacle / keepout    dark red: robot centre here = '
                  'collision\nyellow-orange: near obstacles (higher cost)    grey: unknown',
                  fontsize=9)
    ax.set_ylabel('map y (m)')
    ax.legend(loc='upper right', fontsize=8)
    os.makedirs(os.path.dirname(os.path.abspath(filename)), exist_ok=True)
    fig.savefig(filename, dpi=100, bbox_inches='tight')
    plt.close(fig)


def start_node():
    """Create the node and spin it in a background thread."""
    rclpy.init()
    node = PlanTester()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    return node


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    for name in ('sx', 'sy', 'syaw', 'gx', 'gy', 'gyaw'):
        parser.add_argument(name, type=float)
    parser.add_argument('--obstacle', nargs=3, type=float, action='append', default=[],
                        metavar=('X', 'Y', 'R'), help='extra round obstacle (repeatable)')
    parser.add_argument('--png', default=os.path.join(RESULTS_DIR, 'plan.png'))
    args = parser.parse_args()

    node = start_node()
    try:
        result = plan(node, (args.sx, args.sy, args.syaw), (args.gx, args.gy, args.gyaw),
                      [tuple(o) for o in args.obstacle])
        print(summary(result))
        save_png(result, args.png)
        print(f'picture: {os.path.abspath(args.png)}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
