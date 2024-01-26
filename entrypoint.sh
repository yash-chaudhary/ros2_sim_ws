#!/bin/bash
set -e

# source ros2
source /opt/ros/humble/setup.bash

# source install
source install/setup.bash

# launch navigation controller in another terminal
ros2 launch sam_bot_bringup navigation.launch.py 

# Start an interactive shell
/bin/bash
