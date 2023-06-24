#!/bin/bash
source /opt/ros/humble/setup.bash
source /opt/ros2_ws/install/setup.bash
#colcon build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


