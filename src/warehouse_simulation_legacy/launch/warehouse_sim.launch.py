#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('warehouse_gazebo')
    pkg_description = get_package_share_directory('warehouse_robot_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    controllers_yaml = os.path.join(pkg_description, 'config', 'arm_controllers.yaml')

    # Environment
    models_path = os.path.join(pkg_gazebo, 'models') + ':' + os.path.expanduser('~/.gazebo/models')
    env_model = SetEnvironmentVariable('GAZEBO_MODEL_PATH', models_path)
    env_no_db = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')

    # World
    world_arg = DeclareLaunchArgument('world', default_value='warehouse_industrial.world')
    world_file = PathJoinSubstitution([pkg_gazebo, 'worlds', LaunchConfiguration('world')])

    # Xacro Processing
    xacro_file = os.path.join(pkg_description, 'urdf', 'gp12.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file, 'gui': 'true', 'verbose': 'false'}.items(),
    )

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[robot_description, controllers_yaml, {'use_sim_time': True}]
    )

    # Spawn Robot
    spawn = TimerAction(period=5.0, actions=[Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'warehouse_robot', '-x', '0', '-y', '0', '-z', '0.01']
    )])

    # Controllers
    jsb = TimerAction(period=10.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )])

    arm = TimerAction(period=12.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager']
    )])

    # Product Spawner
    products = TimerAction(period=15.0, actions=[Node(
        package='warehouse_gazebo', executable='product_spawner', name='product_spawner', output='screen'
    )])

    return LaunchDescription([env_no_db, env_model, world_arg, gazebo, rsp, controller_manager, spawn, jsb, arm, products])ls