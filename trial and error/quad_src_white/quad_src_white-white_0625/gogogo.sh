#!/bin/bash

set -e

ROS_DISTRO=humble
WS=~/hitcrt_quad2026_ws

echo "Loading ROS2 environment..."

# source ROS2
if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
else
    echo "ROS2 not found!"
    exit 1
fi

# source workspace
if [ -f $WS/install/setup.bash ]; then
    source $WS/install/setup.bash
else
    echo "Workspace not built!"
    exit 1
fi

echo "Starting launch..."

ros2 launch quad run.launch.py
