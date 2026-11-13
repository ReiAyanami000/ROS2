from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='transport_navigation', executable='path_processor',
             name='path_processor', output='screen'),
    ])
