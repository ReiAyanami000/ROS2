"""Jazzy 静止规划测试：固定 TF、规划服务器、生命周期管理和客户端。"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


def launch_nodes(context):
    """检查测试参数；同一次启动保持同一张地图与同一个测试起点。"""
    values = {}
    for name in ('start_x', 'start_y', 'start_yaw', 'robot_radius', 'inflation_extra'):
        values[name] = float(LaunchConfiguration(name).perform(context))
        if not math.isfinite(values[name]):
            raise ValueError(f'{name} 必须为有限数值')
    if values['robot_radius'] <= 0 or values['inflation_extra'] < 0:
        raise ValueError('robot_radius 必须 > 0，inflation_extra 必须 >= 0')

    share = get_package_share_directory('transport_navigation')
    params = ParameterFile(RewrittenYaml(
        source_file=os.path.join(share, 'config', 'nav2_planner.yaml'),
        param_rewrites={
            'robot_radius': str(values['robot_radius']),
            'inflation_radius': str(values['robot_radius'] + values['inflation_extra']),
        },
        convert_types=True,
    ), allow_substs=True)

    return [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='planning_test_static_tf', output='screen',condition=IfCondition(LaunchConfiguration('publish_static_tf')),
            arguments=[
                '--x', str(values['start_x']), '--y', str(values['start_y']),
                '--z', '0', '--yaw', str(values['start_yaw']),
                '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'base_link',
            ],
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen', parameters=[params],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_planning', output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['planner_server'],
            }],
        ),
        Node(
            package='transport_navigation', executable='navigation_manager',
            name='navigation_manager', output='screen',
            parameters=[{'use_sim_time': False, 'planning_timeout': 10.0}],
        ),
    ]


def generate_launch_description():
    """要求用户明确给出起点和车体半径，避免把示例值当成实车数据。"""
    return LaunchDescription([
        DeclareLaunchArgument('start_x', description='静止测试起点 X，米'),
        DeclareLaunchArgument('start_y', description='静止测试起点 Y，米'),
        DeclareLaunchArgument('start_yaw', default_value='0.0', description='朝向，弧度'),
        DeclareLaunchArgument('robot_radius', description='车体保守包络半径及硬余量，米'),
        DeclareLaunchArgument('inflation_extra', default_value='0.15',
                              description='车体半径之外的软代价范围，米'),
        OpaqueFunction(function=launch_nodes),
        DeclareLaunchArgument('publish_static_tf', default_value='true'),
    ])
