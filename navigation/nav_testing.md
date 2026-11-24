# Navigation testing on the robot

All commands run on the robot's computer. T1 / T2 / T3 = terminal 1 / 2 / 3.

---

## A. Checks

### 1. Software packages (T1)
```bash
source ~/ament_ws/install/setup.bash && ros2 pkg list | grep -E 'nav2_regulated_pure_pursuit_controller|nav2_bringup|spatio_temporal_voxel_layer'
```
Expect: all 3 listed.

### 2. Unit tests (T1)
```bash
cd ~/mini_golf/navigation && python3 -m pytest -q tests
```
Expect: `44 passed`.

### 3. Free, home, stow (T1)
```bash
stretch_free_robot_process.py && stretch_robot_home.py && stretch_robot_stow.py
```

### 4. Launch navigation (T1, leave running)
```bash
source ~/ament_ws/install/setup.bash && ros2 launch ~/mini_golf/navigation/launch/mini_golf_nav.launch.py
```
Expect: `Managed nodes are active` twice.

### 5. Driver mode (T2)
```bash
source ~/ament_ws/install/setup.bash && ros2 topic echo /mode --once
```
Expect: `data: navigation`.

### 6. Lidar (T2)
```bash
ros2 topic hz /scan_filtered
```
Expect: ~10 Hz. Ctrl-C.

### 7. Velocity command wiring (T2)
```bash
ros2 topic info -v /stretch/cmd_vel
```
Expect: subscriber `stretch_driver`; publishers `controller_server`, `behavior_server`.

### 8. Depth camera topic (T2)
```bash
ros2 topic hz /camera/depth/color/points
```
If nothing arrives, start the camera in T3:
```bash
source ~/ament_ws/install/setup.bash && ros2 launch stretch_core d435i_low_resolution.launch.py
```

### 9. Stop (T1)
Ctrl-C.

---

## B. Full navigation (Nav2 + final approach)

### 1. Free, home, stow (T1)
```bash
stretch_free_robot_process.py && stretch_robot_home.py && stretch_robot_stow.py
```

### 2. Launch navigation (T1, leave running)
```bash
source ~/ament_ws/install/setup.bash && ros2 launch ~/mini_golf/navigation/launch/mini_golf_nav.launch.py
```

### 3. Set start pose (RViz)
**2D Pose Estimate** → click the robot's position, drag towards its heading.

### 4. Send goal (T2)
```bash
source ~/ament_ws/install/setup.bash && python3 ~/mini_golf/navigation/go_to_goal.py X Y YAW
```
X, Y in m, YAW in rad (map frame). Ctrl-C to cancel.

### 5. Result (T2)
```bash
echo $?
```
Expect: `0` (reached).

### 6. Stop (T1)
Ctrl-C.

---

## C. Navigation without the final approach (Nav2 only)

### 1. Free, home, stow (T1)
```bash
stretch_free_robot_process.py && stretch_robot_home.py && stretch_robot_stow.py
```

### 2. Launch navigation (T1, leave running)
```bash
source ~/ament_ws/install/setup.bash && ros2 launch ~/mini_golf/navigation/launch/mini_golf_nav.launch.py
```

### 3. Set start pose (RViz)
**2D Pose Estimate** → click the robot's position, drag towards its heading.

### 4. Send goal (T2)
```bash
source ~/ament_ws/install/setup.bash && ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: X, y: Y}, orientation: {z: QZ, w: QW}}}}"
```
QZ = sin(YAW/2), QW = cos(YAW/2). Ctrl-C to cancel.

### 5. Stop (T1)
Ctrl-C.
