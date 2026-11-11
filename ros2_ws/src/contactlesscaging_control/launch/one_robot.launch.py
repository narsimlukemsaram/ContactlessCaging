"""Run a single robot's controller on its own onboard Raspberry Pi."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_nodes(context):
    name=LaunchConfiguration('robot_name').perform(context)
    path=LaunchConfiguration('config_file').perform(context)
    with open(path,'r',encoding='utf-8') as f:
        cfg=yaml.safe_load(f)
    if name not in cfg['robots']:
        raise ValueError(f'No settings for robot {name}')
    role=cfg['robots'][name]
    params={**cfg['controller'], **role, 'robot_name':name, 'leader_name':'robot1'}
    return [Node(package='contactlesscaging_control',executable='formation_controller',
                 namespace=name,name='formation_controller',output='screen',parameters=[params])]


def generate_launch_description():
    default=os.path.join(get_package_share_directory('contactlesscaging_control'),'config','formation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('robot_name',default_value='robot1'),
        DeclareLaunchArgument('config_file',default_value=default),
        OpaqueFunction(function=make_nodes),
    ])
