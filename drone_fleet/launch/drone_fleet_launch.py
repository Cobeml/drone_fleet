#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get package directories
    pkg_drone_fleet = get_package_share_directory('drone_fleet_core')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default=os.path.join(pkg_drone_fleet, 'worlds', 'nyc_midtown.world'))
    robot_name = LaunchConfiguration('robot_name', default='drone1')
    x_pose = LaunchConfiguration('x_pose', default='100.0')
    y_pose = LaunchConfiguration('y_pose', default='100.0')
    z_pose = LaunchConfiguration('z_pose', default='10.0')
    
    # Process xacro file
    xacro_file = os.path.join(pkg_drone_fleet, 'models', 'quadrotor_lidar.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value=world_file,
        description='Full path to the world file'
    )
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='drone1',
        description='Name of the robot'
    )
    
    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='100.0',
        description='X position of robot spawn'
    )
    
    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='100.0',
        description='Y position of robot spawn'
    )
    
    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='10.0',
        description='Z position of robot spawn'
    )
    
    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_gazebo_ros, '/launch/gzserver.launch.py']),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_gazebo_ros, '/launch/gzclient.launch.py'])
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_raw
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-topic', PythonExpression(["'", robot_name, "/robot_description'"]),
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', '0',
            '-P', '0',
            '-Y', '0'
        ],
        output='screen'
    )
    
    # Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_nav2_bringup, '/launch/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_drone_fleet, 'config', 'nav2_params.yaml'),
            'namespace': robot_name
        }.items()
    )
    
    # Map server (placeholder - will use actual NYC map later)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': os.path.join(pkg_drone_fleet, 'maps', 'nyc_midtown.yaml')
        }]
    )
    
    # Lifecycle manager for map server
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # ROSbridge WebSocket server
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '',
            'use_compression': False
        }]
    )
    
    # RViz2
    rviz_config_file = os.path.join(pkg_drone_fleet, 'config', 'drone_fleet.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz', default='true'))
    )
    
    # Fleet manager node (custom)
    fleet_manager = Node(
        package='drone_fleet_core',
        executable='fleet_manager',
        name='fleet_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Mission planner node (custom)
    mission_planner = Node(
        package='drone_fleet_core',
        executable='mission_planner',
        name='mission_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_world_file,
        declare_robot_name,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        
        # Gazebo
        gazebo_server,
        gazebo_client,
        
        # Robot
        robot_state_publisher,
        spawn_robot,
        
        # Navigation
        # nav2_launch,  # Uncomment when Nav2 is fully configured
        # map_server,   # Uncomment when map is available
        # lifecycle_manager_map,  # Uncomment when map is available
        
        # Communication
        rosbridge_server,
        
        # Visualization
        # rviz2,  # Uncomment when RViz config is ready
        
        # Custom nodes
        # fleet_manager,    # Uncomment when implemented
        # mission_planner,  # Uncomment when implemented
    ])