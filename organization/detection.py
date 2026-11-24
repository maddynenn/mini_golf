#!/usr/bin/env python3
"""Detection: find the ball with the robot's head camera.

Uses the head D435i camera (same topic names in sim and on the real robot):
    RGB:        /camera/color/image_raw        (bgr8)
    depth:      /camera/depth/image_rect_raw   (32FC1 metres in sim, 16UC1 mm real)
    depth info: /camera/depth/camera_info      (intrinsics for deprojection)

find_ball() isolates the *yellow* ball by colour (HSV) + shape, gets its pixel,
reads the depth there, deprojects to a 3D point in the camera frame (pushed back
by one ball radius so it is the ball's centre, not its front surface), and
transforms it into FLOOR_FRAME -- the same frame get_robot_floor_pose_xya()
returns -- so moving.py can drive to it.

The hole is NOT detected: its location is predetermined from the map (CLAUDE.md),
so find_hole() just returns the HOLE constant.
"""
import time

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import tf2_geometry_msgs  # registers do_transform for PointStamped
from tf2_geometry_msgs import do_transform_point

# --- Frames / topics ---
FLOOR_FRAME = 'odom'        # 'odom' in sim (no map); switch to 'map' with localization
COLOR_TOPIC = '/camera/color/image_raw'
DEPTH_TOPIC = '/camera/depth/image_rect_raw'
DEPTH_INFO_TOPIC = '/camera/depth/camera_info'
# On the real robot the depth and colour sensors are offset with different
# fields of view, so a colour pixel doesn't line up with the raw depth image.
# realsense publishes depth re-projected onto the colour image; when that topic
# exists we use it (with the colour intrinsics) instead.
ALIGNED_DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'
ALIGNED_DEPTH_INFO_TOPIC = '/camera/color/camera_info'

# --- Head pose used to look at the floor ahead of the robot ---
HEAD_TILT = -0.8            # rad, look down (CLAUDE.md)
HEAD_PAN = 0.0             # rad, look forward (+x)

# --- Yellow-ball thresholds (HSV; OpenCV hue is 0-179, so yellow ~ 20-35) ---
# Deliberately generous so a shadowed, far-away or partly hidden ball still
# counts. False positives are filtered out afterwards by depth and height above
# the floor (see find_ball), not by being strict here.
YELLOW_LOWER = (18, 60, 60)    # (H, S, V) lower bound (was (20, 80, 80))
YELLOW_UPPER = (45, 255, 255)  # (H, S, V) upper bound (was (40, 255, 255))
BALL_RADIUS = 0.04445      # m, 3.5 in diameter ball (depth sees the front face;
                           # we add this along the camera ray to get the centre)
MIN_BALL_AREA = 15         # px^2, ignore specks (was 50; a ball ~4 m away is only ~30 px^2)
MAX_BALL_AREA = 150000     # px^2, ignore huge regions (a 3.5 in ball up close is big)
MIN_CIRCULARITY = 0.4      # 4*pi*area / perimeter^2 (1.0 = perfect circle; was 0.6).
                           # Small/partly hidden blobs score low even when round.
MAX_CANDIDATES = 5         # try this many blobs (biggest first) before giving up
# Ball centre height above the floor must be in this range, or the blob is
# rejected (a yellow thing on a table/wall, or a bad depth reading). Expected
# value is BALL_RADIUS (~0.044 m); FLOOR_FRAME's z = 0 is the floor.
MIN_BALL_HEIGHT = -0.10    # m
MAX_BALL_HEIGHT = 0.20     # m
DEPTH_PATCH = 5            # NxN patch for a robust (median) depth reading

# --- Predetermined hole location in FLOOR_FRAME (read from the map in RViz) ---
# Matches the sim goal_cube at (1.5, 0). Set the real value on the robot.
HOLE = (1.5, 0.0)

_bridge = CvBridge()


def setup(node):
    """Subscribe to the head camera and tilt the head down to see the floor."""
    node.latest_color = None
    node.latest_depth = None
    node.depth_info = None

    depth_topic, info_topic = DEPTH_TOPIC, DEPTH_INFO_TOPIC
    topics = [name for name, _ in node.get_topic_names_and_types()]
    if ALIGNED_DEPTH_TOPIC in topics:
        depth_topic, info_topic = ALIGNED_DEPTH_TOPIC, ALIGNED_DEPTH_INFO_TOPIC
    if COLOR_TOPIC not in topics:
        node.get_logger().error(
            f'detection: {COLOR_TOPIC} is not published. On the real robot start the '
            'camera: ros2 launch stretch_core d435i_low_resolution.launch.py')
    node.get_logger().info(f'detection: depth from {depth_topic}')

    # The sim (and real) camera topics publish BEST_EFFORT, so subscribe with
    # sensor-data QoS or the callbacks never fire (QoS-incompatible otherwise).
    node.create_subscription(Image, COLOR_TOPIC,
                             lambda m: _set(node, 'latest_color', m), qos_profile_sensor_data)
    node.create_subscription(Image, depth_topic,
                             lambda m: _set(node, 'latest_depth', m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, info_topic,
                             lambda m: _set(node, 'depth_info', m), qos_profile_sensor_data)

    # Head move_to_pose is only honored in position mode (the sim often starts in
    # navigation mode), so switch first or the head won't actually tilt down.
    node.switch_to_position_mode()
    time.sleep(1.0)

    # Point the head down-and-forward so the ball is in view, then let frames arrive.
    node.move_to_pose({'joint_head_pan': HEAD_PAN, 'joint_head_tilt': HEAD_TILT}, blocking=True)
    time.sleep(2.0)
    node.get_logger().info('detection: head tilted down, waiting for camera frames')
    _wait_for_frames(node, timeout=10.0)


def find_ball(node):
    """Return the yellow ball's (x, y) in FLOOR_FRAME, or None if not seen."""
    color_msg = node.latest_color
    depth_msg = node.latest_depth
    info = node.depth_info
    if color_msg is None or depth_msg is None or info is None:
        node.get_logger().warn('find_ball: no camera data yet')
        return None

    bgr = _bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
    depth = _bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

    # Colour thresholds are loose, so several blobs may match. Try them biggest
    # first and keep the first one that has valid depth and sits on the floor.
    sx = depth.shape[1] / bgr.shape[1]
    sy = depth.shape[0] / bgr.shape[0]
    for u, v, radius_px in _find_ball_blobs(node, bgr):
        # The blob pixel is in the colour image; sample depth at the matching
        # pixel (scaled in case depth and colour differ in resolution). Shrink
        # the patch for small (far) blobs so it doesn't read the floor behind.
        du, dv = int(round(u * sx)), int(round(v * sy))
        r = max(1, min(DEPTH_PATCH // 2, int(radius_px * sx / 2)))
        z = _depth_at(depth, du, dv, r)
        if z is None:
            node.get_logger().info(f'find_ball: blob at ({u}, {v}) has no valid depth, skipping')
            continue

        point_cam = _deproject(info, du, dv, z)
        point_cam = _surface_to_center(point_cam, BALL_RADIUS)   # front face -> ball centre
        point_floor = _to_floor_frame(node, point_cam, depth_msg.header.frame_id)
        if point_floor is None:
            return None                          # TF is broken; other blobs won't help

        height = point_floor[2]
        if not MIN_BALL_HEIGHT <= height <= MAX_BALL_HEIGHT:
            node.get_logger().info(
                f'find_ball: blob at ({u}, {v}) is {height:.2f} m above the floor, not a ball')
            continue

        node.get_logger().info(
            f'find_ball: ball at ({point_floor[0]:.2f}, {point_floor[1]:.2f}) in {FLOOR_FRAME}')
        return (point_floor[0], point_floor[1])

    return None


def find_hole(node):
    """Return the predetermined hole (x, y) in FLOOR_FRAME (not detected)."""
    return HOLE


# --- helpers ---------------------------------------------------------------

def _set(node, attr, msg):
    setattr(node, attr, msg)


def _wait_for_frames(node, timeout):
    start = time.time()
    while time.time() - start < timeout:
        if node.latest_color is not None and node.latest_depth is not None \
                and node.depth_info is not None:
            return
        time.sleep(0.1)
    node.get_logger().warn('detection: timed out waiting for camera frames '
                           '(real robot: is d435i_low_resolution.launch.py running? '
                           'sim: launched with use_cameras:=true?)')


def _find_ball_blobs(node, bgr):
    """Sufficiently-circular yellow blobs -> [(u, v, radius_px), ...], biggest first."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_BALL_AREA or area > MAX_BALL_AREA:
            continue
        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        if circularity < MIN_CIRCULARITY:
            continue
        m = cv2.moments(c)
        if m['m00'] == 0:
            continue
        u = int(m['m10'] / m['m00'])
        v = int(m['m01'] / m['m00'])
        blobs.append((area, u, v, float(np.sqrt(area / np.pi))))

    if not blobs:
        node.get_logger().warn('find_ball: no yellow circular blob found')
        return []

    blobs.sort(reverse=True)
    return [(u, v, rad) for _, u, v, rad in blobs[:MAX_CANDIDATES]]


def _depth_at(depth, u, v, r=DEPTH_PATCH // 2):
    """Median depth (metres) over a (2r+1)^2 patch at (u, v); None if no valid reading.

    Handles both sim (float32 metres) and real robot (uint16 millimetres).
    """
    h, w = depth.shape[:2]
    u0, u1 = max(0, u - r), min(w, u + r + 1)
    v0, v1 = max(0, v - r), min(h, v + r + 1)
    patch = depth[v0:v1, u0:u1].astype(np.float64)

    if np.issubdtype(depth.dtype, np.integer):
        patch = patch / 1000.0          # uint16 millimetres -> metres

    valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 10.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def _deproject(info, u, v, z):
    """Pixel (u, v) + depth z -> 3D point in the camera optical frame."""
    fx, fy = info.k[0], info.k[4]
    cx, cy = info.k[2], info.k[5]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return (x, y, z)


def _surface_to_center(point_cam, radius):
    """Push a point on the ball's near surface back to its centre, along the
    camera ray (camera is at the optical-frame origin)."""
    x, y, z = point_cam
    d = (x * x + y * y + z * z) ** 0.5
    if d == 0:
        return point_cam
    s = (d + radius) / d
    return (x * s, y * s, z * s)


def _to_floor_frame(node, point_cam, camera_frame):
    """Transform a camera-frame point into FLOOR_FRAME via TF, with retries."""
    ps = PointStamped()
    ps.header.frame_id = camera_frame
    ps.header.stamp = Time().to_msg()      # latest available transform
    ps.point.x, ps.point.y, ps.point.z = point_cam

    for _ in range(30):
        try:
            tf = node.tf2_buffer.lookup_transform(
                FLOOR_FRAME, camera_frame, Time(), timeout=Duration(seconds=1.0))
            out = do_transform_point(ps, tf)
            return (out.point.x, out.point.y, out.point.z)
        except Exception as e:               # TF not ready yet -> retry
            node.get_logger().warn(f'find_ball: TF {camera_frame}->{FLOOR_FRAME} not ready ({e})')
            time.sleep(0.1)
    node.get_logger().error(f'find_ball: gave up on TF {camera_frame}->{FLOOR_FRAME}')
    return None
