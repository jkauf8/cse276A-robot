"""Start localization_control with config/localization.yaml (or your own copy via params_file:=...).

    ros2 launch robot_control localization_launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('robot_control'), 'config', 'localization.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        Node(package='robot_control', executable='localization_control', name='localization_control',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
