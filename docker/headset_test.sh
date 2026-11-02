#!/bin/bash
# Default container command: the real Quest against the simulated UR5 (vr_sim.launch.py quest:=real).
# The Quest app connects on port 10000; Foxglove (browser 3D view) on port 8765.
# Tag map: TAG_MAP, default /root/.ros/handeye/tag_map.yaml (mount the host's folder on /root/.ros).
set -e
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 > /tmp/foxglove_bridge.log 2>&1 &
exec ros2 launch ur5_bringup vr_sim.launch.py quest:=real rviz:=false \
    tag_map_file:="${TAG_MAP:-/root/.ros/handeye/tag_map.yaml}" "$@"
