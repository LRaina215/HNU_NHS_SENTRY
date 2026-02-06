import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('rm_description')
    
    # 读取参数
    config_file = os.path.join(share_dir, 'config', 'launch_params.yaml')
    launch_params = yaml.safe_load(open(config_file))
    xyz_param = launch_params['odom2camera']['xyz'].replace('"', '')
    rpy_param = launch_params['odom2camera']['rpy'].replace('"', '')


    # 导航部分

    nav_urdf = os.path.join(share_dir, 'urdf', 'infantry.urdf.xacro')
    robot_desc_nav = Command(['xacro ', nav_urdf])

    rsp_nav = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp_navigation',
        parameters=[{'robot_description': robot_desc_nav}], # 默认 50Hz，够导航用
   #     remappings=[('/robot_description', '/robot_description_nav')]
    )

    # 自瞄部分

    aim_urdf = os.path.join(share_dir, 'urdf', 'rm_gimbal.urdf.xacro')
    robot_desc_aim = Command([
        'xacro ', aim_urdf, 
        ' xyz:="' + xyz_param + '"', 
        ' rpy:="' + rpy_param + '"'
    ])

    rsp_aim = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp_autoaim',
        parameters=[{
            'robot_description': robot_desc_aim,
            'publish_frequency': 1000.0 
        }],
   #     remappings=[('/robot_description', '/robot_description_aim')]
    )
    return LaunchDescription([rsp_nav, rsp_aim])
   # return LaunchDescription([rsp_aim])

