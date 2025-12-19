#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# English comments only in code.

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Use sim time (from gazebo_slam.launch.py)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # RViz config
    rviz_config_path = get_package_share_path(
        'mecanumrover3_navigation'
    ) / 'rviz/slam.rviz'

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(rviz_config_path),
        description='Absolute path to rviz config file'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Launch slam_toolbox with correct use_sim_time
    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mecanumrover3_navigation'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        rviz_node,
        launch_slam,
    ])

