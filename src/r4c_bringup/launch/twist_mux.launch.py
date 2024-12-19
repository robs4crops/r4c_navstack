#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        # No default argument provided. It is required to pass a config file.
        DeclareLaunchArgument(
            'config_file',
            description='Topics and locks config file'),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='twist_mux/cmd_vel',
            description='cmd vel output topic'),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name="twist_mux",
            parameters=[LaunchConfiguration('config_file')],
            remappings={('cmd_vel_out', LaunchConfiguration('cmd_vel_out'))},
            arguments=["--ros-args", "--log-level", "info"],
            output='screen',
        )]
    )
