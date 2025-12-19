from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    Command,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # --------------------------------------------------
    # Package paths (string paths)
    # --------------------------------------------------
    description_pkg = get_package_share_directory('mecanumrover_description')
    bringup_pkg     = get_package_share_directory('mecanumrover3_bringup')

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if available'
    )

    rover_arg = DeclareLaunchArgument(
        'rover',
        default_value='mecanum3',
        description='Rover type: mecanum3 or g120a'
    )

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([
            bringup_pkg, 'rviz', 'mecanum3.rviz'
        ]),
        description='Absolute path to rviz config file'
    )

    rover = LaunchConfiguration('rover')

    # --------------------------------------------------
    # Select xacro by rover type
    # --------------------------------------------------
    model_path = PythonExpression([
        '"', description_pkg, '/urdf/" + ',
        '("mecanum3.xacro" if "', rover,
        '" == "mecanum3" else "g120a.xacro")'
    ])

    # --------------------------------------------------
    # Robot description
    # --------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', model_path]),
        value_type=str
    )

    # --------------------------------------------------
    # Robot state publisher
    # --------------------------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description,
        }],
    )

    # --------------------------------------------------
    # Odometry publisher
    # --------------------------------------------------
    pub_odom_node = Node(
        package='fwdsrover_xna_bringup',
        executable='pub_odom',
        name='pub_odom',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # --------------------------------------------------
    # Static TF (odom -> base_footprint)
    # --------------------------------------------------
    odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_static',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )

    # --------------------------------------------------
    # RViz
    # --------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    return LaunchDescription([
        use_sim_time_arg,
        rover_arg,
        rviz_arg,
        robot_state_publisher_node,
        pub_odom_node,
        odom_static_tf,
        rviz_node,
    ])

