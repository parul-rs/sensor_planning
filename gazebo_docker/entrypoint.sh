#!/bin/bash

# Save the command we were actually asked to run (e.g. "bash") BEFORE
# sourcing anything. ROS/colcon setup.bash scripts touch the shell's
# positional parameters internally — sourcing them without first clearing
# "$@" can silently wipe out our real command, causing the final `exec`
# to do nothing and the container to exit silently with status 0.
# See: https://github.com/osrf/docker_images/issues/626
args="$@"
set --

source /opt/ros/jazzy/setup.bash

echo "export GZ_SIM_RESOURCE_PATH=/ws/src:\$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc
echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/ws/install/nbv_space_plugins/lib:/opt/ros/jazzy/lib:\$GZ_SIM_SYSTEM_PLUGIN_PATH" >> ~/.bashrc
source ~/.bashrc

# Source built workspace if it exists
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

exec $args
