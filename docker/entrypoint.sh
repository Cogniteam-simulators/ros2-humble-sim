#!/bin/bash
# Source the ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Source the workspace setup (if it exists)
if [ -f /ros2_humble_sim_ws/install/setup.bash ]; then
    source /ros2_humble_sim_ws/install/setup.bash
fi

# Execute the command passed to the container
exec "$@"