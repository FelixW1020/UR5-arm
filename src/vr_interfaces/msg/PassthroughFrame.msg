# One Quest passthrough-camera image plus what is needed to locate ArUco tags in vr_world.
# Sent on /quest/passthrough by PassthroughFrameStreamer.cs (or by vr_sim's quest_sim).
std_msgs/Header header                # frame_id: vr_world; stamp = headset clock (informational)
sensor_msgs/CompressedImage image     # format "jpeg"; upright (row 0 = top), OpenCV convention
sensor_msgs/CameraInfo camera_info    # width/height/K/D for THIS image, OpenCV pixel convention
                                      # (origin at the top-left pixel)
geometry_msgs/Pose camera_pose        # camera pose in vr_world at capture time, "link" convention:
                                      # x = viewing direction, y = image-left, z = image-up.
                                      # The ROS side derives the optical frame (T_LINK_OPTICAL).
