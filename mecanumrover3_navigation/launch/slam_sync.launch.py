from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node


from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rviz_config_path = get_package_share_path('mecanumrover3_navigation') / 'rviz/slam.rviz'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', default_value=str(rviz_config_path),
        description='Absolute path to rviz config file')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    launch_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mecanumrover3_navigation'),
                    'launch',
                    'online_sync_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        )
    
    pub_odom_node = Node(
        package='megarover3_bringup',
        executable='pub_odom',
        name='pub_odom'
    )

    return LaunchDescription([
        rviz_arg,
        sim_time_arg,
        rviz_node,
        launch_slam,
    ])
