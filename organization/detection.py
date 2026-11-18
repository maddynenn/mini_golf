#!/usr/bin/env python3
"""Detection: find the ball (and the hole) with the robot's cameras.

This file should use the Stretch's cameras (see
`ros2 launch stretch_core multi_camera.launch.py`) to locate the golf ball
and the hole, and turn those detections into positions the rest of the code
can use: (x, y) in the robot's floor/odom frame, the same frame
get_robot_floor_pose_xya() returns. moving.py uses the ball position to drive
next to it and the hole position to decide which way to face before swing.py
takes the shot.

Until this is written, find_ball() and find_hole() return None and main.py
falls back to what mini-golf1.py did: ball GOAL_DISTANCE straight ahead.
"""


def setup(node):
    """Subscribe to whatever camera topics detection needs.

    Pseudocode:
        node.latest_image = None
        node.latest_depth = None
        node.create_subscription(Image, '<color topic>', callback that saves to node.latest_image, 10)
        node.create_subscription(Image, '<depth topic>', callback that saves to node.latest_depth, 10)
        node.create_subscription(CameraInfo, '<camera info topic>', ...)  # needed to go pixel -> 3D
    """
    pass


def find_ball(node):
    """Return the ball's (x, y) in the floor frame, or None if not seen.

    Pseudocode:
        image = node.latest_image
        if image is None: return None
        mask = threshold image on the ball's color (e.g. HSV range for a white/orange ball)
        find the biggest blob in mask -> pixel (u, v)
        depth = node.latest_depth at (u, v)
        point_in_camera = deproject (u, v, depth) with the camera intrinsics
        point_in_floor = transform point_in_camera into the odom/base frame with tf
        return (point_in_floor.x, point_in_floor.y)
    """
    return None


def find_hole(node):
    """Return the hole's (x, y) in the floor frame, or None if not seen.

    Pseudocode: same idea as find_ball(), but for the hole (or a marker/flag
    next to it, which is probably easier to see than a hole in the floor).
    """
    return None
