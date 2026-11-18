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
python3 ~/mini_golf/navigation/go_to_goal.py X Y YAW
```

To find a goal's coordinates, hover over a free spot in RViz (shown at the
bottom of the window) or use "Publish Point".

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
- **Real robot:** not tested yet. See "Running on the robot: step by step" below.

Simulator (testing only, needs the starter repo's `parcs_stretch_nav2` package
built in `~/ament_ws`; uses the simulator's small map, not WVH):

```bash
ros2 launch ~/mini_golf/navigation/launch/mini_golf_nav.launch.py use_sim:=true
python3 ~/mini_golf/navigation/go_to_goal.py 0.6 0.6 1.57
```

## Running on the robot: step by step

### Step 1: check the robot's software (once)

```bash
source ~/ament_ws/install/setup.bash

# All three must be listed
ros2 pkg list | grep -E 'nav2_regulated_pure_pursuit_controller|nav2_bringup|spatio_temporal_voxel_layer'

# Note the stretch_ros2 version (this code was written against the humble branch, May 2026)
git -C ~/ament_ws/src/stretch_ros2 log -1 --oneline

# Unit tests: expect "39 passed"
cd ~/mini_golf/navigation && python3 -m pytest -q tests
```

If a package is missing, stop here: Nav2 won't start without it.

### Step 2: prepare the robot (every session)

```bash
stretch_free_robot_process.py     # free the hardware from any other process using it
stretch_robot_home.py             # home the joints (needed after every power-on)
stretch_robot_stow.py             # stow the arm before driving
```

Put the robot somewhere in open space in WVH that you can find on the map.

### Step 3: launch navigation (terminal 1, leave it running)

```bash
source ~/ament_ws/install/setup.bash
ros2 launch ~/mini_golf/navigation/launch/mini_golf_nav.launch.py
```

Wait until the log shows `Managed nodes are active` twice (localization and
navigation), about 10–20 s. Messages about waiting for transforms before that are
normal.

RViz opens on the robot's screen. If you're working over SSH without a screen,
launch with `use_rviz:=false` and run RViz on a laptop on the same network with the
same `ROS_DOMAIN_ID`:

```bash
# On the laptop
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup rviz_launch.py
```

### Step 4: check the wiring (terminal 2)

```bash
source ~/ament_ws/install/setup.bash

ros2 topic echo /mode --once                 # expect: data: navigation
ros2 topic hz /scan_filtered                 # expect ~10 Hz; Ctrl-C to stop
ros2 topic info -v /stretch/cmd_vel          # expect subscriber stretch_driver,
                                             # publishers controller_server and behavior_server
```

If `/mode` isn't `navigation`, run `ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger`.

### Step 5: localize

1. In RViz click **2D Pose Estimate**, then click where the robot really is on the
   map and drag in the direction it faces.
2. Check the red laser points line up with the map's walls. If not, set the pose again.
3. Read back the pose AMCL now has (terminal 2):

   ```bash
   ros2 run tf2_ros tf2_echo map base_link     # Ctrl-C after a few lines
   ```

   `Translation` is x, y; the last number of `RPY (radian)` is the yaw. Write these
   down: the start pose is a handy known-good goal to return to.

### Step 6: choose test goals

Pick goals 1–2 m away in open floor. To read a point's coordinates, click
**Publish Point** in RViz, click the spot, and read it from:

```bash
ros2 topic echo /clicked_point --once
```

Choose the yaw yourself (radians, 0 = map +x, 1.57 = +y, 3.14 = -x, -1.57 = -y).
Mark the goal spot on the floor with tape (a cross under the robot's centre and a
line for the heading) so you can measure the real error later.

### Step 7: Nav2 on its own (no final approach)

In RViz click **Nav2 Goal** and click/drag a goal ~1 m away. Check:

- the green planned path appears, and the robot turns to face it, then drives
  along it smoothly without stopping and starting;
- it stops within ~15 cm of the goal (Nav2's tolerance).

If it plans but doesn't move, go back to step 4 (`/mode` and `/stretch/cmd_vel`).
Pressing the runstop at any time stops it.

### Step 8: full go_to_goal (terminal 2)

```bash
python3 ~/mini_golf/navigation/go_to_goal.py X Y YAW
```

For example, after a Nav2-only move, send the start pose from step 5 to come back.
Watch for, in order:

```
Nav2: going to (X, Y, YAW)
Final approach 1: 1.3 cm, 0.8 deg from goal
Reached goal
```

`echo $?` afterwards prints `0` on success, `1` on failure. Ctrl-C cancels the goal
and stops the base.

What to change if it misbehaves (constants at the top of the files; restart
`go_to_goal.py` after editing, no need to restart the launch):

| What you see | Change |
|---|---|
| Robot hums or sits still during small corrections | Raise `MIN_V` / `MIN_W` in `final_approach.py` |
| Overshoots or wobbles back and forth near the goal | Lower `K_V` / `K_W`, or `MAX_V` / `MAX_W` in `final_approach.py` |
| `Obstacle in the way, waiting` with nothing there | Lower `STOP_DIST` / `STOP_HALF_WIDTH` in `go_to_goal.py` |
| `Final approach timed out` | Robot moving too slowly: raise `MIN_V` / `MIN_W` |
| `Not within 5 cm / 3 deg after 3 attempts` | Note the numbers in the `Final approach N` lines and report them |
| `Nav2 failed (status 6)` | Nav2 aborted: goal blocked/unreachable or robot stuck; check RViz |
| `Robot pose is ... s old` / `No map -> base_link transform` | Localization or driver stopped; check terminal 1 |

### Step 9: measure accuracy

Send the taped goal 3–5 times, from different start positions (use Nav2 Goal in RViz
to move away in between). Each time record:

- the last `Final approach N:` line (error relative to AMCL, should be ≤ 5 cm / 3°);
- the tape-measured error from the floor mark (error relative to the real floor;
  expect this to be bigger, AMCL is typically good to ~5–10 cm);
- the time taken and how many final approach attempts it needed.

### Step 10: obstacles not on the map

1. Put a box in the middle of a path the robot will take, then send a goal past it.
   Nav2 should plan around it (RViz shows the box in the costmap).
2. Put a box ~25 cm in front of where the robot will stop, with the goal set so the
   final approach drives towards it. It should log `Obstacle in the way, waiting`,
   continue if you remove the box within 10 s, and otherwise fail with
   `Still blocked, giving up`.

### Step 11: speed up (only once steps 7–10 work)

Edit `desired_linear_vel` in `~/mini_golf/navigation/config/nav2_params.yaml`
(0.2 now; the starter used up to 0.3 m/s), then restart the launch (Ctrl-C in
terminal 1 and run step 3 again, then set the pose again).

### Stopping

Ctrl-C in terminal 1. If it doesn't exit after ~10 s (Nav2's lifecycle manager
sometimes hangs), press Ctrl-C again. Check nothing is left running with
`ps aux | grep -E 'nav2|amcl|stretch_driver' | grep -v grep`.
