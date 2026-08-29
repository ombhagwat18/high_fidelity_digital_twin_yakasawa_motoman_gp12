#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('warehouse_gazebo')
    pkg_desc = get_package_share_directory('warehouse_robot_description')
    pkg_moveit = get_package_share_directory('warehouse_moveit_config')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    # Critical: Make Gazebo find meshes
    env_model = SetEnvironmentVariable('GAZEBO_MODEL_PATH', pkg_desc)
    env_resource = SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', pkg_desc)

    # Controller config for gazebo_ros2_control. gp12.xacro's <gazebo_ros2_control>
    # plugin block reads its ros2_control YAML from this xacro arg — without it,
    # gazebo_ros2_control loads with an empty controller list and
    # joint_state_broadcaster/arm_controller can never be spawned.
    controllers_file = os.path.join(pkg_moveit, 'config', 'ros2_controllers.yaml')

    # Robot description
    robot_description = xacro.process_file(
        os.path.join(pkg_desc, 'urdf', 'gp12.xacro'),
        mappings={'controllers_file': controllers_file},
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch/gazebo.launch.py')),
        launch_arguments={
            'world': PathJoinSubstitution([pkg_gazebo, 'worlds', 'warehouse_industrial.world']),
            'gui': 'true'
        }.items()
    )

    # Spawn robot (higher Z)
    spawn_robot = TimerAction(
        period=6.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'warehouse_robot',
                '-topic', '/robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.35'
            ],
            output='screen'
        )]
    )

    # Controllers
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager']
    )

    # Sequencing
    jsb_after = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[TimerAction(period=3.0, actions=[jsb_spawner])])
    )
    arm_after = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[TimerAction(period=3.0, actions=[arm_spawner])])
    )

    return LaunchDescription([
        env_model, env_resource,
        robot_state_publisher, gazebo,
        spawn_robot, jsb_after, arm_after,
    ])