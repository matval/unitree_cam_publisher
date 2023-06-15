#!/bin/bash
#set -e

ros_source_env="$ROS_ROOT/install/setup.bash"
source "$ros_source_env"

echo "sourcing   $ros_source_env"

echo "ROS_DISTRO $ROS_DISTRO"
echo "ROS_ROOT   $ROS_ROOT"

source ~/ros2_ws/install/setup.bash

exec "$@"