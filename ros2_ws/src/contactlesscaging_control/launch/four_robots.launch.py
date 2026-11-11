"""Launch four local controllers, reference generator and optionally the UDP pose bridge."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context):
    config_path=LaunchConfiguration('config_file').perform(context)
    use_bridge=LaunchConfiguration('use_udp_bridge').perform(context).lower() in ('true','1','yes')
    with open(config_path,'r',encoding='utf-8') as fh: cfg=yaml.safe_load(fh)
    nodes=[]
    names=list(cfg['robots'])
    if not names or 'robot1' not in cfg['robots']:
        raise ValueError('Expected a robot1 leader in formation configuration')
    for name, role in cfg['robots'].items():
        params={**cfg['controller'],**role,'robot_name':name,'leader_name':'robot1'}
        nodes.append(Node(package='contactlesscaging_control',executable='formation_controller',
                          namespace=name,name='formation_controller',output='screen',parameters=[params]))
    nodes.append(Node(package='contactlesscaging_control',executable='transport_reference',
                      name='transport_reference',output='screen',parameters=[{'leader_name':'robot1',**cfg['reference'],
                                                                 'frame_id':cfg['controller']['frame_id']}]))
    if use_bridge:
        nodes.append(Node(package='contactlesscaging_control',executable='phasespace_udp_bridge',
                          name='phasespace_udp_bridge',output='screen',parameters=[cfg['bridge']]))
    return nodes


def generate_launch_description():
    default=os.path.join(get_package_share_directory('contactlesscaging_control'),'config','formation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file',default_value=default),
        DeclareLaunchArgument('use_udp_bridge',default_value='false'),
        OpaqueFunction(function=_make_nodes)
    ])
