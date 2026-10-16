"""RealSense driver with depth aligned to colour (topics under /camera/camera/...)."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
            launch_arguments={
                'camera_namespace': 'camera',
                'camera_name': 'camera',
                'align_depth.enable': 'true',
                'enable_sync': 'true',
                'pointcloud.enable': 'false',
            }.items()),
    ])
