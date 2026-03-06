import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    world_sdf_path = LaunchConfiguration("world_sdf_path")
    ign_config_path = LaunchConfiguration("ign_config_path")

    declare_world_sdf_path = DeclareLaunchArgument(
        "world_sdf_path",
        default_value=os.path.join(pkg_simulator, "resource", "worlds", "rmul_2024_world.sdf"),
    )
    declare_ign_config_path = DeclareLaunchArgument(
        "ign_config_path",
        default_value=os.path.join(pkg_simulator, "resource", "ign", "gui.config"),
    )

    # 【核心修改 1】Galactic 使用 ros_ign_gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_ign_gazebo"), "launch", "ign_gazebo.launch.py")
        ),
        launch_arguments={
            "ign_args": [world_sdf_path, TextSubstitution(text=" --gui-config "), ign_config_path],
        }.items(),
    )

    # 【核心修改 2】Galactic 使用 ros_ign_bridge 和 ignition.msgs.Clock
    robot_ign_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_sdf_path)
    ld.add_action(declare_ign_config_path)
    ld.add_action(gazebo)
    ld.add_action(robot_ign_bridge)

    return ld
