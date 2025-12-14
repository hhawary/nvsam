#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
export LD_LIBRARY_PATH=/usr/local/cuda-12.0/targets/x86_64-linux/lib:$LD_LIBRARY_PATH
exec "$@"
