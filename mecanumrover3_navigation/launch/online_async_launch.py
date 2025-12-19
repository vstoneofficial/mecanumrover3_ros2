#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# English comments only in code.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Use simulation time (passed from slam.launch.py → gazebo_slam.launch.py)
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_file = LaunchConfiguration('slam_params_file')

    # use_sim_time must be TRUE when running in Gazebo
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )

    # SLAM parameters file
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory("mecanumrover3_navigation"),
            'config', 'mapper_params_online_async.yaml'),
        description='Full path to the parameters file for slam_toolbox'
    )

    # slam_toolbox main node
    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld

