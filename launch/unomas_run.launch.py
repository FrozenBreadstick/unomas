from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory

import os
import tempfile

def create_bridge_config(robot_name, template_path):
    """Generate a robot-specific bridge config from template"""
    with open(template_path, 'r') as f:
        content = f.read().replace('{ROBOTNAME}', robot_name) #replace the placeholder "{ROBOTNAME}" with the actual robot name
    
    temp_file = os.path.join(tempfile.gettempdir(), f'{robot_name}_bridge.yaml')
    with open(temp_file, 'w') as f:
        f.write(content)
    
    return temp_file

def create_fella(robot_name, x_pos, y_pos, use_sim_time, pkg_path, config_path):
    """Create a robot instance in Gazebo with a unique name and position"""

    #Make the bridge config temp file first
    template_path = os.path.join(get_package_share_directory('unomas'),
                                 'config',
                                'gazebo_bridge.yaml')
    bridge_config_path = create_bridge_config(robot_name, template_path)

    print("#CREATING DESCRPTION PACKAGE FOR ROBOT:", robot_name, "#")

    #Make the robot description as normal
    robot_description_content = ParameterValue(
    Command([
        'xacro ',
        PathJoinSubstitution([pkg_path, 'urdf', 'fella.urdf.xacro']),
        TextSubstitution(text=(' robot_name:=' + robot_name))
    ]),
    value_type=str)

    #Create a group to define robot name with namespace
    robot_group = GroupAction([
        PushRosNamespace(robot_name),

        #Robot State Publisher (with a namespace now)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time,
                'frame_prefix': f'{robot_name}/' #Prefix all frames with the robot name   
            }]
        ),

        #robot localisation (with a namespace now)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='robot_localization',
            output='screen',
            parameters=[PathJoinSubstitution([config_path,
                                              'robot_localization.yaml']),
                        {'use_sim_time': use_sim_time,
                         'frame_prefix': f'{robot_name}/'}], #Prefix all frames with the robot name
            remappings=[
                ('odometry/filtered','odom'),
                ('odom0','odom'),
            ]
        ),

        #Spawn the robot in Gazebo (global action no namespace)
        Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-topic', f'/{robot_name}/robot_description', 
                '-x', str(x_pos), 
                '-y', str(y_pos), 
                '-z', '0.4', 
                '-name', robot_name
            ]
        ),

        #Bridge using tempfile
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config_path,
                         'use_sim_time': use_sim_time}]
        )
    ])

    return robot_group

def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('unomas')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    #Read text in ~/UNOMAS/robots.txt to determine number of robots + names
    robots_file = os.path.join(os.path.expanduser('~'), 'UNOMAS', 'robots.txt')
    if os.path.exists(robots_file):
        with open(robots_file, 'r') as f:
            lines = f.readlines()
            robot_names = [line.strip() for line in lines if line.strip()]
    else: 
        robot_names = ['fella1']  #Default to one robot if no file
        print(f"Robots file not found at {robots_file}. Defaulting to one robot named 'fella1'.")

    print("#CHECKING FOR DUPLICATES")

    #Test for duplicates, error if found
    for i, name1 in enumerate(robot_names):
        for j, name2 in enumerate(robot_names):
            if i == j:
                continue
            if name1 == name2:
                raise ValueError(f"Duplicate robot name found: {name1}. Please ensure all robot names are unique in {robots_file}.")
        
    print("#STARTING GAZEBO CREATION")
    # Start Gazebo to simulate the robot in the chosen world
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    ld.add_action(world_launch_arg)

    print("#CREATING GAZEBO INSTANCE#")

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    print("#STARTING ROBOT CREATION#")

    #Add robots, spaced out in x direction by 2.0m
    x_start = 0.0
    y_start = 0.0
    spacing = 2.0
    for i, name in enumerate(robot_names):
        robot_group = create_fella(
            robot_name=name,
            x_pos=x_start + i * spacing,
            y_pos=y_start,
            use_sim_time=use_sim_time,
            pkg_path=pkg_path,
            config_path=config_path
        )
        ld.add_action(robot_group)

    print("#ROBOT CREATION COMPLETE#")
    print("#STARTING RVIZ2#")

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 enables mapping and waypoint following
    # nav2 = IncludeLaunchDescription(
    #     PathJoinSubstitution([pkg_path,
    #                           'launch',
    #                           'unomas_sim.launch.py']),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time
    #     }.items(),
    #     condition=IfCondition(LaunchConfiguration('nav2'))
    # )
    # ld.add_action(nav2)

    return ld
