from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    station_name_arg = DeclareLaunchArgument(
        'station_name',
        default_value='DebugBase1',
        description='Name of the created Base Station'
    )
    ld.add_action(station_name_arg)


    simulation_size_arg = DeclareLaunchArgument(
        'size',
        default_value=150,
        description='Size of the simulation area (in meters)'
    )
    ld.add_action(simulation_size_arg)

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='RobotDebug1',
        description='Name of the created Robot'
    )
    ld.add_action(robot_name_arg)

    station_name = LaunchConfiguration('station_name')
    simulation_size = LaunchConfiguration('size')
    robot_name = LaunchConfiguration('robot_name')

    base_station_node = Node(
            package='unomas',
            executable='base_station',
            output='screen',
            parameters=[{'station_name': station_name}]
        )
    ld.add_action(base_station_node)

    ui_bridge_node = Node(
            package='unomas',
            executable='ui_bridge',
            output='screen'
        )
    ld.add_action(ui_bridge_node)

    simulation_extras_node = Node(
            package='unomas',
            executable='simulation_extras',
            output='screen',
            parameters=[{'size': simulation_size}]
        )
    ld.add_action(simulation_extras_node)

    robot_node = Node(
            package='unomas',
            executable='robot',
            output='screen',
            parameters=[{'serial_id': robot_name}]
        )
    ld.add_action(robot_node)

    return ld


