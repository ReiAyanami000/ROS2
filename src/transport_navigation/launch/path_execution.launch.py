from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_radius', description='Actual conservative circular robot radius in metres'),
        Node(package='transport_navigation', executable='path_executor', name='path_executor',
             output='screen', parameters=[{
                 'robot_radius': ParameterValue(LaunchConfiguration('robot_radius'), value_type=float),
             }]),
    ])
