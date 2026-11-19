# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([
                FindPackageShare('tf_buffer_server'), 'config', 'server.yaml'])),
        ComposableNodeContainer(
            name='tf_buffer_container', namespace='',
            package='rclcpp_components', executable='component_container',
            composable_node_descriptions=[ComposableNode(
                package='tf_buffer_server', plugin='tf_buffer_server::BufferServer',
                name='tf_buffer_server',
                parameters=[LaunchConfiguration('params_file')])],
            output='screen'),
    ])
