# mini_golf

## Good commands to have

### Start the robot with it broadcasting its location

ros2 launch stretch_core stretch_driver.launch.py broadcast_odom_tf:=True

### Start the head camera (second terminal; needed for detection)

ros2 launch stretch_core d435i_low_resolution.launch.py

Check it's up with `ros2 topic list | grep camera`. (`use_cameras:=true` is a
sim-only argument; the real driver ignores it.)

### Make the helper modules available

source ~/ament_ws/install/setup.bash

### Start the script

python3 ~/mini_golf/mini-golf1.py

## Organized version (organization/)

Same first two steps as above (start the driver, then source), then:

### Run the full shot (detection -> moving -> swing)

python3 ~/mini_golf/organization/main.py

### Test only the swing (robot already parked next to the ball)

python3 ~/mini_golf/organization/swing.py

## Navigation (navigation/)

Drive to (x, y, yaw) on the WVH map with Nav2 + a precise final approach.
See navigation/README.md.

### Start the robot with navigation (driver, lidar, AMCL, Nav2, RViz)
ros2 launch ~/mini_golf/navigation/launch/mini_golf_nav.launch.py

### Set the start pose in RViz ("2D Pose Estimate"), then send a goal (m, m, rad)
python3 ~/mini_golf/navigation/go_to_goal.py X Y YAW

### Test path planning without a robot (planner only, pictures in navigation/planning_results/)
ros2 launch ~/mini_golf/navigation/launch/plan_only.launch.py   # add use_rviz:=true for --interactive
python3 ~/mini_golf/navigation/plan_path.py SX SY SYAW GX GY GYAW [--obstacle X Y R]   # or --interactive (click in RViz)
python3 ~/mini_golf/navigation/run_planning_tests.py
