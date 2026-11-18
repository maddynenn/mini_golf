# navigation: drive to a point on the map

Give the robot `(x, y, yaw)` in the WVH map frame and it drives there:

1. **Nav2** plans a path and follows it with Regulated Pure Pursuit, avoiding
   obstacles the lidar sees (including ones not on the map). It stops within
   0.15 m / 0.25 rad of the goal.
2. **Final approach** (`final_approach.py`) drives the last few cm and turns to
   the goal yaw. It stops and waits if something is in the way, and gives up
   after 10 s.
3. The result is checked against **±5 cm / ±3°** of where AMCL thinks the robot
   is. If it's outside that, the final approach is repeated (up to 3 times).

## Running it

```bash
# Terminal 1: driver, lidar, AMCL, Nav2, RViz
source ~/ament_ws/install/setup.bash
ros2 launch ~/mini_golf/navigation/launch/mini_golf_nav.launch.py

# In RViz: "2D Pose Estimate" -> click and drag where the robot actually is.
# Check the laser scan lines up with the map walls before sending goals.

# Terminal 2: send a goal (map frame: metres, metres, radians)
source ~/ament_ws/install/setup.bash
python3 ~/mini_golf/navigation/go_to_goal.py 20.0 19.5 1.57
```

Exit code 0 means the goal was reached. Ctrl-C cancels the goal and stops the base.
Stow the arm before driving: Nav2 plans with the base footprint only.

From Python (`node` is a running `hm.HelloNode`, like `MiniGolf` in `organization/main.py`):

```python
sys.path.insert(0, os.path.expanduser('~/mini_golf/navigation'))
import go_to_goal
go_to_goal.setup(node)
ok = go_to_goal.go_to(node, x, y, yaw)
```

Launch options: `map:=<yaml>`, `params_file:=<yaml>`, `use_rviz:=false`,
`use_sim:=true` (MuJoCo simulator, see below).

## Files

| File | What it is |
|---|---|
| `go_to_goal.py` | `go_to()`: Nav2 goal, final approach, lidar stop check, accuracy check; command line tool |
| `final_approach.py` | The final-approach controller (no ROS, unit tested). Tolerances, speeds and gains at the top |
| `launch/mini_golf_nav.launch.py` | Starts everything. Based on the starter repo's navigation launch files |
| `config/nav2_params.yaml` | Nav2 params (copy of the starter's `nav2_voxel_params.yaml`, changes below) |
| `maps/` | WVH map + keepout zones, and the simulator's map (copies from the starter repo) |
| `tests/` | `python3 -m pytest tests` (after sourcing ROS) |

## Changes from the starter repo (parcs_stretch_nav2_WVH)

- **Controller:** DWB replaced by Regulated Pure Pursuit, `desired_linear_vel`
  0.2 m/s (starter max was 0.3). In `config/nav2_params.yaml`, `controller_server.FollowPath`.
- **Goal tolerance:** `xy_goal_tolerance` 0.25 -> 0.15 m. Yaw stays 0.25 rad.
- **cmd_vel remap:** Nav2 publishes on `/cmd_vel` but the Stretch driver listens on
  `/stretch/cmd_vel`. The starter had this remap commented out; the launch file adds it.
- **Removed:** joystick teleop and the SLAM option. Everything else (AMCL, costmaps,
  3D obstacle layer, keepout zones, planner, recovery behaviours) is the same.

## Testing status

- **Unit tests:** pass (final approach against a simulated base, including slow and
  noisy; lidar stop zone with the real lidar's backwards mounting).
- **Simulator:** Nav2 + Regulated Pure Pursuit reached a goal 0.85 m away
  (64 s). The final approach could **not** be tested there: the simulated base only
  achieves ~11–15% of small commanded speeds, so it timed out. It has not
  run end to end anywhere yet.
- **Real robot:** not tested yet. See the checklist below.

Simulator (testing only, needs the starter repo's `parcs_stretch_nav2` package
built in `~/ament_ws`; uses the simulator's small map, not WVH):

```bash
ros2 launch ~/mini_golf/navigation/launch/mini_golf_nav.launch.py use_sim:=true
python3 ~/mini_golf/navigation/go_to_goal.py 0.6 0.6 1.57
```

## First real-robot test checklist

Someone on the e-stop for all of this, with clear space around the robot.

1. **Launch and localize.** Start the launch file, set the pose in RViz, and check
   the scan lines up with the walls. `ros2 topic info /stretch/cmd_vel` should
   list the driver as a subscriber and controller_server as a publisher.
2. **Nav2 only, short goal.** Send a goal ~1 m away in open space. Watch that it
   turns, then drives smoothly along the green path in RViz. If it doesn't move at
   all, check the driver is in navigation mode (`/mode` topic).
3. **Final approach.** Watch the log lines `Final approach N: X cm, Y deg`.
   - Robot hums but doesn't move on small corrections -> raise `MIN_V` / `MIN_W`
     in `final_approach.py`.
   - Overshoots or oscillates -> lower `K_V` / `K_W` or `MAX_V` / `MAX_W`.
   - Stops with "Obstacle in the way" when nothing is there -> the lidar sees part
     of the robot or nearby walls: reduce `STOP_DIST` / `STOP_HALF_WIDTH` in
     `go_to_goal.py`.
4. **Measure accuracy.** Mark the floor at a goal; send it 3–5 times from different
   starts. The logs report error vs AMCL; the tape measure shows error vs the
   real floor (expected to be larger, AMCL is typically good to ~5–10 cm).
5. **Obstacle not on the map.** Put a box on the path: Nav2 should go around it.
   Put one right next to the goal: the final approach should stop and wait, then
   fail after 10 s.
6. **Speed.** Once 1–5 work, raise `desired_linear_vel` in `config/nav2_params.yaml`
   (the starter used up to 0.3 m/s).
