"""Final approach: the velocity command that moves the robot the last few cm.

No ROS here, so it can be unit tested (tests/test_final_approach.py).
go_to_goal.py calls command() in a loop with the robot's map-frame pose.

The robot drives straight to the goal point (backwards if the goal is behind
it), turning in place first if it is pointing the wrong way, then turns in
place to the goal yaw.
"""
import math

# Tolerances the final approach aims for. They are tighter than the ±5 cm / ±3°
# requirement (checked in go_to_goal.py) so a small AMCL update doesn't push the
# result outside it.
POS_TOL = 0.02                   # m
YAW_TOL = math.radians(1.5)      # rad
ALIGN_TOL = math.radians(10)     # rad, turn in place before driving if worse than this

# Speeds. MIN_* stop the real base stalling on tiny commands - tune on the robot.
MAX_V, MIN_V = 0.08, 0.02        # m/s
MAX_W, MIN_W = 0.4, 0.08         # rad/s
K_V, K_W = 1.0, 1.5              # proportional gains


def wrap(angle):
    """Angle in [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def clamp_speed(value, min_speed, max_speed):
    """Keep |value| within [min_speed, max_speed], keeping its sign."""
    return math.copysign(min(max(abs(value), min_speed), max_speed), value)


def command(pose, goal):
    """Return (v, w) to send to the base, or None once the goal is reached.

    pose, goal: (x, y, yaw) in the same frame. v in m/s (negative = reverse),
    w in rad/s.
    """
    x, y, yaw = pose
    gx, gy, gyaw = goal
    dist = math.hypot(gx - x, gy - y)

    if dist > POS_TOL:
        heading_err = wrap(math.atan2(gy - y, gx - x) - yaw)
        direction = 1.0
        if abs(heading_err) > math.pi / 2:  # goal is behind: reverse instead of turning around
            direction = -1.0
            heading_err = wrap(heading_err + math.pi)
        if abs(heading_err) > ALIGN_TOL:
            return 0.0, clamp_speed(K_W * heading_err, MIN_W, MAX_W)
        return direction * clamp_speed(K_V * dist, MIN_V, MAX_V), K_W * heading_err

    yaw_err = wrap(gyaw - yaw)
    if abs(yaw_err) > YAW_TOL:
        return 0.0, clamp_speed(K_W * yaw_err, MIN_W, MAX_W)
    return None
