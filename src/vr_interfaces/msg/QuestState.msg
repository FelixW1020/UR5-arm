# Headset and both controllers, sent by the Quest app at a fixed rate on /quest/state.
# Every pose is in vr_world (RobotWorldFrame, FLU). header.stamp is the headset clock and is
# not comparable to ROS time; vr_tracking restamps on receipt.
std_msgs/Header header        # frame_id: vr_world
bool head_tracked
geometry_msgs/Pose head       # CenterEyeAnchor (x = gaze direction, y left, z up)
ControllerState left
ControllerState right
