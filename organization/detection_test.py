#!/usr/bin/env python3
r"""Detection-only test. No driving, no swinging.

Tilts the head down, saves what the camera sees (and the white-threshold mask)
as PNG files, then runs find_ball() a few times and prints the (x, y) it reports.

Run with the sim up:
    ros2 launch stretch_simulation stretch_mujoco_driver.launch.py \
        mode:=navigation use_cameras:=true use_robocasa:=false
    source ~/ament_ws/install/setup.bash
    python3 ~/mini_golf/organization/detection_test.py

If the GUI image viewers are flaky in WSL, just open the saved PNGs from Windows:
    \\wsl.localhost\Ubuntu-22.04\home\confi\mini_golf\organization\debug_color.png
    \\wsl.localhost\Ubuntu-22.04\home\confi\mini_golf\organization\debug_mask.png

What good output looks like:
    - debug_color.png shows the white golf ball somewhere in the frame.
    - debug_mask.png shows the ball as a white blob on a black background
      (the mask is the white threshold; if the ball is missing/partial, lower
       WHITE_LOWER's V or raise WHITE_UPPER's S in detection.py; other white
       things showing up is OK -- the size and floor-height checks reject them).
    - find_ball -> (~0.8, ~0.0), matching the ball's spot in scene.xml.
"""
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
import hello_helpers.hello_misc as hm

import detection

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
_bridge = CvBridge()


class DetectionTest(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'detection_test', 'detection_test',
                          wait_for_first_pointcloud=False)
        detection.setup(self)

        # Save the current frame + white mask so we can eyeball them from Windows.
        if self.latest_color is not None:
            bgr = _bridge.imgmsg_to_cv2(self.latest_color, desired_encoding='bgr8')
            mask = detection.white_mask(bgr)
            cv2.imwrite(os.path.join(OUT_DIR, 'debug_color.png'), bgr)
            cv2.imwrite(os.path.join(OUT_DIR, 'debug_mask.png'), mask)
            self.get_logger().info(f'Saved debug_color.png and debug_mask.png to {OUT_DIR}')
        else:
            self.get_logger().error('No color frame received. Real robot: run '
                                    'ros2 launch stretch_core d435i_low_resolution.launch.py. '
                                    'Sim: launch with use_cameras:=true.')

        # Run detection a few times (the ball/robot are still, so it should be steady).
        for i in range(5):
            ball = detection.find_ball(self)
            self.get_logger().info(f'[{i}] find_ball -> {ball}')
            time.sleep(1.0)

        self.get_logger().info('Done. Ball should read about (0.8, 0.0) in odom.')
        rclpy.shutdown()


if __name__ == '__main__':
    DetectionTest().main()
