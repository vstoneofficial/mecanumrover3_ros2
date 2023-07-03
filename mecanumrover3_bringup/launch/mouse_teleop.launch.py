from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mouse_teleop_node = Node( 
        package='mouse_teleop', 
        executable='mouse_teleop', 
        name='mouse_teleop',
        remappings=[('/mouse_vel', '/rover_twist')]
    )

    return LaunchDescription([
        mouse_teleop_node
    ])