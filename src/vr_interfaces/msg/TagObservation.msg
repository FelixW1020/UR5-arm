# One ArUco tag as measured by the Quest.
# Coordinates are in vr_world: the Unity calibration root, converted to the ROS
# convention (x forward, y left, z up; ROS-TCP-Connector's FLU) on the Unity side.
int32 id
geometry_msgs/Point position   # tag centre, metres
