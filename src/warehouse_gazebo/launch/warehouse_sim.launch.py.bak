#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    sim_pkg = get_package_share_directory('warehouse_gazebo')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse_industrial.world',
        description='Gazebo world file name',
    )
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_camera_tfs_arg = DeclareLaunchArgument('use_camera_tfs', default_value='true')
    spawn_products_arg = DeclareLaunchArgument('spawn_products', default_value='true')
    top_camera_x_arg = DeclareLaunchArgument('top_camera_x', default_value='0.75')
    top_camera_y_arg = DeclareLaunchArgument('top_camera_y', default_value='0.0')
    top_camera_z_arg = DeclareLaunchArgument('top_camera_z', default_value='1.50')
    top_camera_yaw_arg = DeclareLaunchArgument('top_camera_yaw', default_value='1.5708')
    front_camera_x_arg = DeclareLaunchArgument('front_camera_x', default_value='1.20')
    front_camera_y_arg = DeclareLaunchArgument('front_camera_y', default_value='0.0')
    front_camera_z_arg = DeclareLaunchArgument('front_camera_z', default_value='0.75')
    front_camera_yaw_arg = DeclareLaunchArgument('front_camera_yaw', default_value='0.7854')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_camera_tfs': LaunchConfiguration('use_camera_tfs'),
            'spawn_products': LaunchConfiguration('spawn_products'),
            'top_camera_x': LaunchConfiguration('top_camera_x'),
            'top_camera_y': LaunchConfiguration('top_camera_y'),
            'top_camera_z': LaunchConfiguration('top_camera_z'),
            'top_camera_yaw': LaunchConfiguration('top_camera_yaw'),
            'front_camera_x': LaunchConfiguration('front_camera_x'),
            'front_camera_y': LaunchConfiguration('front_camera_y'),
            'front_camera_z': LaunchConfiguration('front_camera_z'),
            'front_camera_yaw': LaunchConfiguration('front_camera_yaw'),
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        use_sim_time_arg,
        use_camera_tfs_arg,
        spawn_products_arg,
        top_camera_x_arg,
        top_camera_y_arg,
        top_camera_z_arg,
        top_camera_yaw_arg,
        front_camera_x_arg,
        front_camera_y_arg,
        front_camera_z_arg,
        front_camera_yaw_arg,
        gazebo,
    ])
