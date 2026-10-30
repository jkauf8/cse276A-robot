"""Rectify the camera image, then run apriltag_node. The camera and motor_control are started separately.

    ros2 launch robot_control tag_detection_launch.py tag_size:=0.16
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('robot_control'), 'config', 'apriltag_hw2.yaml')
    rectify = LaunchConfiguration('rectify')
    image_topic = PythonExpression(["'/camera/image_rect' if '", rectify, "' == 'true' else '/camera/image_raw'"])

    return LaunchDescription([
        DeclareLaunchArgument('tag_size', default_value='0.15', description='tag edge length in meters'),
        DeclareLaunchArgument('rectify', default_value='true', description='rectify the image before apriltag_node'),
        DeclareLaunchArgument('raw_image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info', default_value='/camera/camera_info'),
        # image_proc rectify: /camera/image_raw + camera_info -> /camera/image_rect
        ComposableNodeContainer(
            name='rectify_container', namespace='', package='rclcpp_components', executable='component_container',
            condition=IfCondition(rectify),
            composable_node_descriptions=[ComposableNode(
                package='image_proc', plugin='image_proc::RectifyNode', name='rectify_node',
                remappings=[('image', LaunchConfiguration('raw_image')), ('camera_info', LaunchConfiguration('camera_info')),
                            ('image_rect', '/camera/image_rect')])],
            output='screen'),
        Node(package='apriltag_ros', executable='apriltag_node', name='apriltag', output='screen',
             remappings=[('image_rect', image_topic), ('camera_info', LaunchConfiguration('camera_info'))],
             parameters=[params, {'size': ParameterValue(LaunchConfiguration('tag_size'), value_type=float)}]),
    ])
