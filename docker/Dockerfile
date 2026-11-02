# The headset side of this repo (vr_* packages, the RViz-free simulation, ROS-TCP-Endpoint) on
# ROS 2 Humble, for machines without Ubuntu. No robot driver, cameras or YOLO. See docker/README.md.
#   docker build -f docker/Dockerfile -t ur5-vr .        (from the repo root)
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3-pip python3-yaml \
        ros-humble-ur-description ros-humble-xacro ros-humble-interactive-markers \
        ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*
# OpenCV 4.x with the ArUco/AprilTag module (OpenCV 5 dropped the hand-eye solvers); NumPy 1.x for Humble.
RUN pip3 install --no-cache-dir "opencv-contrib-python-headless>=4.7,<4.11" "numpy<2"

# The endpoint the Quest app connects to (port 10000), in its own workspace so that mounting
# this repo's src/ over /ws/src doesn't hide it.
RUN git clone --depth 1 -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git \
        /opt/endpoint_ws/src/ros_tcp_endpoint \
    && cd /opt/endpoint_ws && . /opt/ros/humble/setup.sh && colcon build

WORKDIR /ws
COPY src/ src/
# Builds every package (ur5_bringup depends on all of them), but only the headset side has its
# runtime dependencies here. --symlink-install: with the repo's src/ mounted over /ws/src, Python
# and config edits on the host apply on the next launch, without rebuilding the image.
RUN . /opt/ros/humble/setup.sh && . /opt/endpoint_ws/install/setup.sh && colcon build --symlink-install

COPY docker/entrypoint.sh docker/headset_test.sh /usr/local/bin/
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["headset_test.sh"]
