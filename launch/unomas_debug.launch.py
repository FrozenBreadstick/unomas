from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unomas',
            executable='base_station',
            output='screen',
            parameters=[{'station_name': 'DebugBase1'}]
        ),
        Node(
            package='unomas',
            executable='ui_bridge',
            output='screen'
        ),
        Node(
            package='unomas',
            executable='simulation_extras',
            output='screen',
            parameters=[{'size': 150, 'resolution': 1}]
        ),
        Node(
            package='unomas',
            executable='robot',
            output='screen',
            parameters=[{'serial_id': 'RobotDebug1'}]
        )
    ])
