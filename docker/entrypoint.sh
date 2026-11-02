#!/bin/bash
# Source ROS, the endpoint and this workspace, then run the given command.
set -e
source /opt/ros/humble/setup.bash
source /opt/endpoint_ws/install/setup.bash
source /ws/install/setup.bash
exec "$@"
