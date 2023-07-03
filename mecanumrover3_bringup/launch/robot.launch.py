from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_package_path = get_package_share_path('mecanumrover_description')
    bringup_package_path = get_package_share_path('mecanumrover3_bringup')
    default_model_path = description_package_path / 'urdf/mecanum3.xacro'
    default_rviz_config_path = bringup_package_path / 'rviz/mecanum3.rviz'
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    pub_odom_node = Node(
        package='mecanumrover3_bringup',
        executable='pub_odom',
        name='pub_odom'
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        pub_odom_node,
        rviz_node,
    ])