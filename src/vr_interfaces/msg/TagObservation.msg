# One ArUco tag as measured by the Quest.
# Coordinates are in vr_world: the Unity calibration root, converted to the ROS
# convention (x forward, y left, z up; ROS-TCP-Connector's FLU) on the Unity side.
int32 id
geometry_msgs/Point position            # tag centre, metres
bool has_orientation                    # false for controller-tip touches (position only)
geometry_msgs/Quaternion orientation    # tag frame (x right, y up, z out of the printed face) in
                                        # vr_world; only meaningful if has_orientation
