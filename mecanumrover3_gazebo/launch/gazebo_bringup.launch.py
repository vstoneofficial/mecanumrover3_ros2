import os
import subprocess
import tempfile
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration,)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, FindExecutable, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# =========================================================
# Safe kill gazebo (same as fwdsrover)
# =========================================================
def _safe_kill_gazebo(context, *args, **kwargs):
    has_gzserver = subprocess.run(
        ["pgrep", "gzserver"], stdout=subprocess.DEVNULL
    ).returncode == 0
    has_gzclient = subprocess.run(
        ["pgrep", "gzclient"], stdout=subprocess.DEVNULL
    ).returncode == 0

    if has_gzserver or has_gzclient:
        os.system("killall -9 gzserver gzclient 2>/dev/null")

    os.system("rm -f /tmp/.gazebo/lock")
    return [LogInfo(msg="[gazebo_bringup] Safe kill done (no chain events)")]


# =========================================================
# Generate world (same responsibility as fwdsrover)
# =========================================================
def _make_world_with_state(context, *args, **kwargs):
    physics_type = LaunchConfiguration("physics").perform(context)
    if physics_type not in ("ode", "bullet"):
        physics_type = "ode"

    world_xml = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <physics type="{physics_type}">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <publish_model_state>true</publish_model_state>
      <publish_link_state>true</publish_link_state>
      <update_rate>30</update_rate>
      <model_states_topic>/model_states</model_states_topic>
      <link_states_topic>/link_states</link_states_topic>
    </plugin>
  </world>
</sdf>
"""
    fd, path = tempfile.mkstemp(prefix="mecanum_world_", suffix=".world")
    with os.fdopen(fd, "w") as f:
        f.write(world_xml)

    return [
        LogInfo(msg=f"[gazebo_bringup] generated temp world: {path}"),
        SetLaunchConfiguration("world_path", path),
    ]


def _create_urdf_and_rsp(context, *args, **kwargs):
    rover = LaunchConfiguration("rover").perform(context)

    urdf_dir = os.path.join(
        get_package_share_directory("mecanumrover_description"),
        "urdf",
    )

    if rover == "mecanum3":
        xacro_file = os.path.join(urdf_dir, "mecanum3.xacro")
    elif rover == "g120a":
        xacro_file = os.path.join(urdf_dir, "g120a.xacro")
    elif rover == "g40a_lb":
        xacro_file = os.path.join(urdf_dir, "g40a_lb.xacro")
    else:
        raise RuntimeError(
            f"[gazebo_bringup] Unknown rover type: {rover}"
        )

    fd, urdf_path = tempfile.mkstemp(prefix="robot_", suffix=".urdf")
    os.close(fd)

    # Generate URDF file for Gazebo spawn
    gen = ExecuteProcess(
        cmd=[FindExecutable(name="xacro"), xacro_file, "-o", urdf_path],
        output="screen",
    )

    # robot_state_publisher must use Command substitution (official way)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [FindExecutable(name="xacro"), " ", xacro_file]
                ),
                "use_sim_time": True,
            }
        ],
    )

    return [
        SetLaunchConfiguration("urdf_path", urdf_path),
        LogInfo(msg=f"[gazebo_bringup] generated temp urdf: {urdf_path}"),
        gen,
        rsp,
    ]




# =========================================================
# Cleanup temp files (same as fwdsrover)
# =========================================================
def _cleanup_temp_files(context, *args, **kwargs):
    world_path = LaunchConfiguration("world_path").perform(context)
    urdf_path = LaunchConfiguration("urdf_path").perform(context)

    for p in (world_path, urdf_path):
        try:
            if p and os.path.exists(p):
                os.remove(p)
        except Exception:
            pass

    return [LogInfo(msg="[gazebo_bringup] Cleaned up temp files")]


# =========================================================
# Main
# =========================================================
def generate_launch_description():

    rover = LaunchConfiguration("rover")
    gui = LaunchConfiguration("gui")
    world_path = LaunchConfiguration("world_path")
    urdf_path = LaunchConfiguration("urdf_path")

    safe_kill = OpaqueFunction(function=_safe_kill_gazebo)
    make_world = OpaqueFunction(function=_make_world_with_state)
    create_urdf_and_rsp = OpaqueFunction(function=_create_urdf_and_rsp)

    gazebo = ExecuteProcess(
        cmd=[
            "ros2", "launch", "gazebo_ros", "gazebo.launch.py",
            ["gui:=", gui],
            ["world:=", world_path],
        ],
        output="screen",
    )

    spawn = ExecuteProcess(
        cmd=[
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-entity", rover,
            "-file", urdf_path,
            "-z", "0.03",
        ],
        output="screen",
    )

    jsb = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    wheel = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "wheel_velocity_controller",
        ],
        output="screen",
    )

    rover_twist_relay = Node(
        package="mecanumrover_description",
        executable="rover_twist_relay.py",
        name="rover_twist_relay",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("mecanumrover_description"),
                "scripts",
                "rover_twist_relay.yaml",
            ),
            {
                "rover": rover,
                "use_sim_time": True,
            },
        ],
    )

    gazebo_odom_bridge = Node(
        package="mecanumrover3_bringup",
        executable="gazebo_odom_bridge",
        name="gazebo_odom_bridge",
        parameters=[
            {"model_name": rover},
            {"base_link_name": "base_footprint"},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    on_shutdown_cleanup = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup_temp_files)])
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rover", default_value="mecanum3"),
        DeclareLaunchArgument("physics", default_value="ode"),


        safe_kill,
        make_world,
        create_urdf_and_rsp,
        gazebo,
        spawn,
        jsb,
        wheel,
        rover_twist_relay,
        gazebo_odom_bridge,
        on_shutdown_cleanup,
    ])

