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
            name='ui_bridge_node',
            output='screen'
        )
    ])
