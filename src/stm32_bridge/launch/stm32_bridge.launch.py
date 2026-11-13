from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('stm32_bridge')) / 'config/stm32_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        Node(package='stm32_bridge', executable='stm32_bridge', name='stm32_bridge', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'serial_port': ParameterValue(LaunchConfiguration('serial_port'), value_type=str),
                 'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
             }]),
    ])
