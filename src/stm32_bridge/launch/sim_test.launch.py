from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('stm32_bridge')) / 'config/stm32_bridge.yaml')
    simulator = Node(package='stm32_bridge', executable='fake_stm32', output='screen',
                     arguments=['--transport', 'tcp', '--port', LaunchConfiguration('port')])
    bridge = Node(package='stm32_bridge', executable='stm32_bridge', name='stm32_bridge',
                  output='screen', parameters=[config, {
                      'serial_port': ParameterValue(['socket://127.0.0.1:', LaunchConfiguration('port')], value_type=str),
                      'publish_tf': True,
                  }])
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        RegisterEventHandler(OnProcessExit(target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason='simulator exited'))])),
        simulator, bridge,
    ])
