#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# English comments only in code.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui          = LaunchConfiguration('gui', default='true')
    rover        = LaunchConfiguration('rover', default='mecanum3')
    wall         = LaunchConfiguration('wall', default='Wall.stl')

    # --------------------------------------------------
    # Package shares
    # --------------------------------------------------
    pkg_gzb = FindPackageShare('mecanumrover3_gazebo')
    nav_pkg = FindPackageShare('mecanumrover3_navigation')

    # --------------------------------------------------
    # RViz config (passed to SLAM launch)
    # --------------------------------------------------
    rviz_config = LaunchConfiguration(
        'rvizconfig',
        default=PathJoinSubstitution([
            nav_pkg, 'rviz', 'slam.rviz'
        ])
    )

    # --------------------------------------------------
    # Gazebo bringup (world + robot only)
    # --------------------------------------------------
    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_gzb,
                'launch',
                'gazebo_bringup.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rover': rover,
            'gui': gui,
        }.items()
    )

    # --------------------------------------------------
    # Wall spawn
    # --------------------------------------------------
    spawn_wall = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_gzb,
                'launch',
                'spawn_wall.launch.py'
            ])
        ),
        launch_arguments={
            'wall': wall,
        }.items()
    )

    # --------------------------------------------------
    # SLAM (shared lower-level launch, do not modify)
    # --------------------------------------------------
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav_pkg,
                'launch',
                'slam.launch.py'
            ])
        ),
        launch_arguments={
            'rvizconfig': rviz_config,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'rover',
            default_value='mecanum3',
            description='Rover type'
        ),

        DeclareLaunchArgument(
            'wall',
            default_value='Wall.stl',
            description='Wall STL filename under mecanumrover3_gazebo/models'
        ),

        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                nav_pkg, 'rviz', 'slam.rviz'
            ])
        ),

        gazebo_bringup,
        spawn_wall,
        slam,
    ])
