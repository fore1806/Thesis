#!/bin/bash
set -e

# Setup ROS 2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source the workspace if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
fi

# Optimization: Start udev to help with hot-plugging sensors
if [ -x "$(command -v service)" ]; then
    service udev start > /dev/null 2>&1 || true
fi

exec "$@"
