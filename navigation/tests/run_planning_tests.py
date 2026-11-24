#!/usr/bin/env python3
"""Run a set of path-planning cases on the WVH map and check the results.

Needs plan_only.launch.py running (no robot or simulator).

    python3 ~/mini_golf/navigation/tests/run_planning_tests.py [CASE_NAME ...]

Saves one picture per case to navigation/tests/planning_results/<name>.png and
prints a PASS/FAIL/INFO table. Poses are map frame (m, m, rad); obstacles are
(x, y, radius) not on the map, seen by a fake lidar at the start pose.

What each expectation checks:
  reach      a path is found, it ends at the goal (within 5 cm), and no
             pose on it is in a collision cell (cost >= 99)
  avoid      reach, and the path keeps at least the robot radius
             (minus one 5 cm cell) from every added obstacle
  avoid_or_none   either no path, or one that satisfies avoid
  not_reach  no path, or the path does not end at the goal (NavFn plans
             to the nearest free point within 0.5 m if the goal is blocked)
  report     no check, just look at the picture
"""
import os
import sys

import rclpy

import plan_path

# Blocking row across the south corridor (walls at x ~19.0 and ~22.5 at y 10.5):
# r = 0.2 circles every 0.3 m, so there is no gap.
ROW = [(round(19.2 + 0.3 * i, 2), 10.3, 0.2) for i in range(11)]

CASES = [
    # name, start, goal, obstacles, expectation, what it shows
    ('corridor_straight', (14.0, 20.5, 0.0), (18.0, 20.3, 0.0), [], 'reach',
     'Open corridor: near-straight path'),
    ('corner_to_south_corridor', (19.5, 20.0, 0.0), (21.5, 10.0, -1.57), [], 'reach',
     'Turn from the main corridor into the south corridor'),
    ('long_to_south_west', (19.5, 20.0, 0.0), (3.0, 0.5, 3.14), [], 'reach',
     'Long trip across the building, round two corners'),
    ('long_to_north_east', (19.5, 20.0, 0.0), (50.0, 29.5, 0.5), [], 'reach',
     'Long trip the other way, past keepout zones'),
    ('start_near_wall', (19.0, 19.05, 0.0), (19.0, 20.5, 1.57), [], 'reach',
     'Start 0.35 m from a wall (inside the cost zone, outside the collision zone)'),
    ('start_touching_wall', (19.0, 18.9, 0.0), (19.0, 20.5, 1.57), [], 'report',
     'Start ~0.2 m from a wall: the robot centre is in the collision zone'),
    ('goal_in_keepout', (19.5, 20.0, 0.0), (20.0, 22.5, 1.57), [], 'not_reach',
     'Goal inside a keepout zone'),
    ('goal_in_wall', (16.0, 20.3, 0.0), (16.0, 19.1, -1.57), [], 'not_reach',
     'Goal on a wall'),
    ('goal_outside_building', (19.5, 20.0, 0.0), (9.0, 15.0, 0.0), [], 'report',
     'Goal outside the building (free space, allowed): is there a way out?'),
    ('obstacle_in_corridor', (14.0, 20.5, 0.0), (18.0, 20.3, 0.0), [(15.2, 20.5, 0.2)], 'avoid',
     'Fake obstacle 1.2 m ahead in the corridor: path goes around it'),
    ('corridor_blocked', (20.75, 10.9, -1.57), (20.75, 8.5, -1.57), ROW, 'avoid_or_none',
     'Fake obstacles across the whole south corridor: no way through'),
]

GOAL_TOL = 0.05        # m
CELL = 0.05            # m, costmap resolution


def check(result, expect):
    """Return (verdict, reason)."""
    found = result['found']
    reaches = found and result['end_gap'] <= GOAL_TOL
    safe = found and result['max_cost'] < plan_path.INSCRIBED
    clear = found and result.get('obstacle_clearance', 99.0) >= plan_path.ROBOT_RADIUS - CELL
    if expect == 'report':
        return 'INFO', 'look at the picture'
    if expect == 'not_reach':
        return ('PASS', 'goal not reached') if not reaches else ('FAIL', 'path reaches the goal')
    if expect == 'avoid_or_none' and not found:
        return 'PASS', 'no path'
    if not found:
        return 'FAIL', 'no path found'
    if not reaches:
        return 'FAIL', f"path ends {result['end_gap']:.2f} m from the goal"
    if not safe:
        return 'FAIL', f"collision cell on the path (cost {result['max_cost']})"
    if expect in ('avoid', 'avoid_or_none') and not clear:
        return 'FAIL', f"passes {result['obstacle_clearance']:.2f} m from an added obstacle"
    if expect in ('avoid', 'avoid_or_none') and not all(result['obstacles_seen']):
        return 'FAIL', 'an added obstacle did not appear in the costmap'
    return 'PASS', 'ok'


def main():
    wanted = set(sys.argv[1:])
    cases = [c for c in CASES if not wanted or c[0] in wanted]
    if wanted - {c[0] for c in CASES}:
        sys.exit(f"unknown case(s): {', '.join(wanted - {c[0] for c in CASES})}")

    node = plan_path.start_node()
    rows = []
    try:
        for name, start, goal, obstacles, expect, description in cases:
            print(f'\n=== {name}: {description}')
            result = plan_path.plan(node, start, goal, obstacles)
            print(plan_path.summary(result))
            verdict, reason = check(result, expect)
            png = os.path.join(plan_path.RESULTS_DIR, f'{name}.png')
            plan_path.save_png(result, png, f'{name} [{verdict}]: {description}')
            print(f'{verdict} ({expect}): {reason}   picture: {png}')
            rows.append((name, expect, verdict, reason))
    finally:
        rclpy.shutdown()

    print('\n' + '-' * 78)
    for name, expect, verdict, reason in rows:
        print(f'{verdict:4s}  {name:26s} {expect:14s} {reason}')
    failed = sum(v == 'FAIL' for _, _, v, _ in rows)
    print(f'{len(rows) - failed} of {len(rows)} passed or informational, {failed} failed')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
