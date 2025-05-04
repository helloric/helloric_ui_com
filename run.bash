#! /bin/bash

. /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
source install/setup.bash
ros2 run helloric_ui_com helloric_ui_com