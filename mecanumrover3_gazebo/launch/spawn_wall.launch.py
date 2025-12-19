import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _gen_sdf_and_spawn(context, *args, **kwargs):
    # Read launch arguments
    entity = LaunchConfiguration('entity').perform(context)
    wall_arg = LaunchConfiguration('wall').perform(context)

    # Fixed pose (world-fixed wall)
    x = float(LaunchConfiguration('x').perform(context))
    y = float(LaunchConfiguration('y').perform(context))
    z = float(LaunchConfiguration('z').perform(context))
    roll  = float(LaunchConfiguration('roll').perform(context))
    pitch = float(LaunchConfiguration('pitch').perform(context))
    yaw   = float(LaunchConfiguration('yaw').perform(context))

    # Package paths
    pkg_share = get_package_share_directory('mecanumrover3_gazebo')
    models_dir = os.path.join(pkg_share, 'models')

    # Resolve wall mesh URI
    # Priority:
    # 1) empty or "auto" -> Wall.stl
    # 2) filename only  -> <pkg>/models/<filename>
    # 3) file:// URI    -> use as-is
    if wall_arg == '' or wall_arg.lower() == 'auto':
        mesh_path = os.path.join(models_dir, 'Wall.stl')
        mesh_uri = f'file://{mesh_path}'
    elif wall_arg.startswith('file://'):
        mesh_uri = wall_arg
    else:
        mesh_path = os.path.join(models_dir, wall_arg)
        mesh_uri = f'file://{mesh_path}'

    # Fixed scale (mm -> m)
    sx, sy, sz = '0.001', '0.001', '0.001'

    # Generate minimal SDF (static wall)
    sdf = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{entity}">
    <static>true</static>
    <link name="wall_link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
            <scale>{sx} {sy} {sz}</scale>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
            <scale>{sx} {sy} {sz}</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
'''

    # Write temporary SDF file
    tmpdir = tempfile.mkdtemp(prefix='spawn_wall_')
    sdf_path = os.path.join(tmpdir, f'{entity}.sdf')
    with open(sdf_path, 'w') as f:
        f.write(sdf)

    # Spawn entity in Gazebo
    return [
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', entity,
                '-file', sdf_path,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
                '-R', str(roll),
                '-P', str(pitch),
                '-Y', str(yaw),
            ],
            output='screen'
        )
    ]


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'wall',
            default_value='',
            description='Wall STL filename or URI. Empty or "auto" uses Wall.stl'
        ),

        DeclareLaunchArgument(
            'entity',
            default_value='wall',
            description='Gazebo entity name'
        ),

        DeclareLaunchArgument('x', default_value='-0.75'),
        DeclareLaunchArgument('y', default_value='-0.75'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('roll',  default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw',   default_value='0.0'),

        OpaqueFunction(function=_gen_sdf_and_spawn),
    ])

