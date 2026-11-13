"""No Nav2: simulator + processor + executor + known initial map/odom alignment."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bridge = str(Path(get_package_share_directory('stm32_bridge'))/'launch/sim_test.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge)),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='demo_map_odom', arguments=['--x', '1.0', '--y', '1.0', '--z', '0',
                 '--yaw', '0', '--pitch', '0', '--roll', '0',
                 '--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='transport_navigation', executable='path_processor', output='screen'),
        Node(package='transport_navigation', executable='path_executor', output='screen',
             parameters=[{'robot_radius': 0.10}]),
    ])
