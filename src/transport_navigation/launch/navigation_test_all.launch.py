"""无实物联调：一次启动地图、模拟底盘、规划、路径执行与 RViz。"""

import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_nodes(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    domain = int(argument('domain_id'))
    port = int(argument('port'))
    if not 0 <= domain <= 232:
        raise ValueError('domain_id 必须在 0 到 232 之间；本测试建议使用 78')
    if not 1024 <= port <= 65535:
        raise ValueError('port 必须在 1024 到 65535 之间')

    numbers = {}
    for name in ('start_x', 'start_y', 'start_yaw', 'robot_radius',
                 'inflation_extra', 'obstacle_diameter'):
        numbers[name] = float(argument(name))
        if not math.isfinite(numbers[name]):
            raise ValueError(f'{name} 必须为有限数值')
    if numbers['robot_radius'] <= 0 or numbers['obstacle_diameter'] <= 0:
        raise ValueError('robot_radius 和 obstacle_diameter 必须为正数')
    if numbers['inflation_extra'] < 0:
        raise ValueError('inflation_extra 不能为负数')

    centers = json.loads(argument('obstacles'))
    if not isinstance(centers, list):
        raise ValueError('obstacles 必须为 [[x,y], ...] 格式的 JSON 数组')
    poses = []
    for center in centers:
        if not isinstance(center, list) or len(center) != 2:
            raise ValueError('每个障碍物必须为 [x,y]')
        x, y = float(center[0]), float(center[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('障碍物坐标必须为有限数值')
        poses.append({'position': {'x': x, 'y': y, 'z': 0.0},
                      'orientation': {'w': 1.0}})

    share = get_package_share_directory('transport_navigation')
    bridge_share = get_package_share_directory('stm32_bridge')
    map_yaml = argument('map_yaml')
    planner_yaml = argument('planner_yaml')
    for filename in (map_yaml, planner_yaml):
        if not os.path.isfile(filename):
            raise ValueError(f'找不到配置文件：{filename}')
    planner_params = ParameterFile(RewrittenYaml(
        source_file=planner_yaml,
        param_rewrites={
            'robot_radius': str(numbers['robot_radius']),
            'inflation_radius': str(numbers['robot_radius'] + numbers['inflation_extra']),
            'use_sim_time': 'false',
        }, convert_types=True,
    ), allow_substs=True)

    simulator = Node(
        package='stm32_bridge', executable='fake_stm32', output='screen',
        arguments=['--transport', 'tcp', '--port', str(port)],
    )
    actions = [
        # 设置所有后续子进程的通信域；不会修改调用 launch 的父终端。
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain)),
        RegisterEventHandler(OnProcessExit(
            target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason='模拟 STM32 已退出'))],
        )),
        simulator,
        Node(
            package='stm32_bridge', executable='stm32_bridge',
            name='stm32_bridge', output='screen',
            parameters=[os.path.join(bridge_share, 'config', 'stm32_bridge.yaml'), {
                'serial_port': f'socket://127.0.0.1:{port}',
                'publish_tf': True, 'use_sim_time': False,
            }],
        ),
        # 模拟器的 odom 初值为零；这组偏移决定小车在地图中的初始位姿。
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='test_map_odom_tf', output='screen',
            arguments=[
                '--x', str(numbers['start_x']), '--y', str(numbers['start_y']),
                '--z', '0', '--yaw', str(numbers['start_yaw']),
                '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
        ),
        Node(
            package='nav2_map_server', executable='map_server',
            name='field_map_server', output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}],
            remappings=[('map', '/field/base_map')],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_field_map', output='screen',
            parameters=[{'use_sim_time': False, 'autostart': True,
                         'node_names': ['field_map_server']}],
        ),
        Node(
            package='transport_navigation', executable='map_composer',
            name='map_composer', output='screen',
            parameters=[{'obstacle_diameter': numbers['obstacle_diameter'],
                         'use_sim_time': False}],
        ),
        # 等待订阅者后只发布一次。空数组代表本轮扫描没有障碍物。
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '-w', '1',
                 '/radar/obstacle_centers', 'geometry_msgs/msg/PoseArray',
                 json.dumps({'header': {'frame_id': 'map'}, 'poses': poses})],
            output='screen',
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen', parameters=[planner_params],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_planning', output='screen',
            parameters=[{'use_sim_time': False, 'autostart': True,
                         'node_names': ['planner_server']}],
        ),
        Node(
            package='transport_navigation', executable='navigation_manager',
            name='navigation_manager', output='screen',
            parameters=[{'use_sim_time': False, 'planning_timeout': 10.0}],
        ),
        Node(
            package='transport_navigation', executable='path_processor',
            name='path_processor', output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='transport_navigation', executable='path_executor',
            name='path_executor', output='screen',
            parameters=[{'robot_radius': numbers['robot_radius'],'safety_margin': 0.0,'tracking_tolerance': 0.05,'use_sim_time': False}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(share, 'rviz', 'planning_test.rviz')],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]
    return actions


def generate_launch_description():
    share = get_package_share_directory('transport_navigation')
    defaults = {
        'domain_id': EnvironmentVariable('ROS_DOMAIN_ID', default_value='78'),
        'port': '8765',
        'start_x': '0.30', 'start_y': '0.30', 'start_yaw': '0.0',
        'robot_radius': '0.15', 'inflation_extra': '0.15',
        'obstacle_diameter': '0.10', 'obstacles': '[[1.2,0.8]]',
        'map_yaml': os.path.join(share, 'maps', 'field.yaml'),
        'planner_yaml': os.path.join(share, 'config', 'nav2_planner.yaml'),
        'rviz': 'true',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        OpaqueFunction(function=launch_nodes),
    ])
