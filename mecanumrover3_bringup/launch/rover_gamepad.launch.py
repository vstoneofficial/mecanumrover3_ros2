#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            {'deadzone': 0.05},
            {'autorepeat_rate': 20.0},
            {'coalesce_interval': 0.001},
        ]
    )

    rover_gamepad = Node(
        package='mecanumrover3_bringup',
        executable='rover_gamepad',
        name='rover_gamepad',
        emulate_tty=True,
        parameters=[
            {'publish_rate': 100.0},
        ]
    )

    return LaunchDescription([joy_node, rover_gamepad])

