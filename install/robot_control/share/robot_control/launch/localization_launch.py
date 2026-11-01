"""Launch localization_control with localization_full.yaml (override with params_file:=...)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('robot_control'), 'config', 'localization_full.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        Node(package='robot_control', executable='localization_control', name='localization_control',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
