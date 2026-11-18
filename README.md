# mini_golf

## Good commands to have

### Start the robot with it broadcasting its location

ros2 launch stretch_core stretch_driver.launch.py broadcast_odom_tf:=True use_cameras:=true

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
