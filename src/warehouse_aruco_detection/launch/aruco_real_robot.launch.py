#!/usr/bin/env python3
"""
aruco_real_robot.launch.py
==========================
Real Yaskawa GP12 robot + Nvidia Jetson Nano + 2x RealSense D455

Architecture:
  Laptop (this machine)   --  MoveIt2, pick_place_logic, RViz2, aruco_detector
  Jetson Nano (robot PC)  --  RealSense drivers, ros2_control hardware interface
  Cameras:
    top_camera   -- D455 overhead (serial set in top_camera_serial)
    front_camera -- D455 side view (serial set in front_camera_serial)

Static TF published here (measure your physical mount and update):
  world -> top_camera_color_optical_frame
  world -> front_camera_color_optical_frame

Usage:
  # Home (no cameras plugged in) -- camera nodes will warn but rest works
  ros2 launch warehouse_aruco_detection aruco_real_robot.launch.py

  # At company with cameras:
  ros2 launch warehouse_aruco_detection aruco_real_robot.launch.py \
      top_camera_serial:=XXXXXXXXX \
      front_camera_serial:=YYYYYYYYY

  # No RViz (headless Jetson):
  ros2 launch warehouse_aruco_detection aruco_real_robot.launch.py use_rviz:=false
"""

import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    robot_pkg  = get_package_share_directory('warehouse_robot_description')
    moveit_pkg = get_package_share_directory('warehouse_moveit_config')
    aruco_pkg  = get_package_share_directory('warehouse_aruco_detection')
    cfg_file   = os.path.join(aruco_pkg, 'config', 'aruco_config.yaml')
    ros2_controllers_file = os.path.join(moveit_pkg, 'config', 'ros2_controllers.yaml')

    # Build URDF string from xacro
    _raw = xacro.process_file(
        os.path.join(robot_pkg, 'urdf', 'gp12.xacro'),
        mappings={
            'ros2_control_plugin': 'mock_components/GenericSystem',
            'embed_gazebo_ros2_control_plugin': 'false',
        },
    ).toxml()
    _clean = re.sub(r'<!--.*?-->', '', _raw, flags=re.DOTALL)
    robot_description_xml = _clean.replace(
        'package://warehouse_robot_description/',
        'file://' + robot_pkg + '/',
    )

    moveit_config = (
        MoveItConfigsBuilder('warehouse_robot',
                             package_name='warehouse_moveit_config')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    # ── Launch arguments ──────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_rviz',             default_value='true'),
        DeclareLaunchArgument('use_pick_place',       default_value='false'),
        DeclareLaunchArgument('use_cameras',          default_value='true'),
        DeclareLaunchArgument('use_local_ros2_control', default_value='false'),
        DeclareLaunchArgument('use_joint_state_publisher', default_value='true'),
        DeclareLaunchArgument('marker_size_m',        default_value='0.10'),
        DeclareLaunchArgument('show_window',          default_value='false'),
        DeclareLaunchArgument('auto_calibrate_extrinsics', default_value='false'),
        # Camera serial numbers -- leave empty to auto-detect
        DeclareLaunchArgument('top_camera_serial',    default_value=''),
        DeclareLaunchArgument('front_camera_serial',  default_value=''),
        # TOP camera physical mount (measure at company, update here)
        # Default: overhead at z~1.5m, looking down
        DeclareLaunchArgument('top_cam_x',            default_value='0.75'),
        DeclareLaunchArgument('top_cam_y',            default_value='0.0'),
        DeclareLaunchArgument('top_cam_z',            default_value='1.50'),
        DeclareLaunchArgument('top_cam_pitch',        default_value='1.5708'),
        # FRONT camera physical mount
        # Default: side view at z~0.75m, pitch ~45deg down
        DeclareLaunchArgument('front_cam_x',          default_value='1.20'),
        DeclareLaunchArgument('front_cam_y',          default_value='0.0'),
        DeclareLaunchArgument('front_cam_z',          default_value='0.75'),
        DeclareLaunchArgument('front_cam_pitch',      default_value='0.7854'),
    ]

    # ── NODE 1: robot_state_publisher ────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': False,
        }],
    )

    # ── NODE 2: Static TF  world -> top_camera_color_optical_frame ───
    top_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='top_camera_static_tf',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('auto_calibrate_extrinsics')),
        arguments=[
            '--x',     LaunchConfiguration('top_cam_x'),
            '--y',     LaunchConfiguration('top_cam_y'),
            '--z',     LaunchConfiguration('top_cam_z'),
            '--roll',  '0.0',
            '--pitch', LaunchConfiguration('top_cam_pitch'),
            '--yaw',   '0.0',
            '--frame-id',       'world',
            '--child-frame-id', 'top_camera_color_optical_frame',
        ],
    )

    # ── NODE 3: Static TF  world -> front_camera_color_optical_frame ─
    front_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_camera_static_tf',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('auto_calibrate_extrinsics')),
        arguments=[
            '--x',     LaunchConfiguration('front_cam_x'),
            '--y',     LaunchConfiguration('front_cam_y'),
            '--z',     LaunchConfiguration('front_cam_z'),
            '--roll',  '0.0',
            '--pitch', LaunchConfiguration('front_cam_pitch'),
            '--yaw',   '0.0',
            '--frame-id',       'world',
            '--child-frame-id', 'front_camera_color_optical_frame',
        ],
    )

    # World to base_link (important for planning)
    world_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    # Optional online TF calibration from known ArUco reference markers.
    extrinsic_calibrator = Node(
        package='warehouse_aruco_detection',
        executable='extrinsic_calibrator',
        name='extrinsic_calibrator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_calibrate_extrinsics')),
        parameters=[
            cfg_file,
            {
                'marker_size_m': LaunchConfiguration('marker_size_m'),
                'show_window':   LaunchConfiguration('show_window'),
                'use_sim_time':  False,
            },
        ],
    )

    # ── NODE 4: TOP D455 RealSense driver ────────────────────────────
    # Publishes: /top_camera/color/image_raw
    #            /top_camera/color/camera_info
    #            /top_camera/aligned_depth_to_color/image_raw
    top_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace='top_camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_cameras')),
        parameters=[{
            'serial_no':              LaunchConfiguration('top_camera_serial'),
            'enable_color':           True,
            'enable_depth':           True,
            'align_depth.enable':     True,
            'color_width':            1280,
            'color_height':           720,
            'color_fps':              30.0,
            'depth_width':            1280,
            'depth_height':           720,
            'depth_fps':              30.0,
            'pointcloud.enable':      False,
            'camera_name':            'top_camera',
            'base_frame_id':          'top_camera_link',
            'tf_publish_rate':        0.0,
        }],
    )

    # ── NODE 5: FRONT D455 RealSense driver ──────────────────────────
    front_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace='front_camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_cameras')),
        parameters=[{
            'serial_no':              LaunchConfiguration('front_camera_serial'),
            'enable_color':           True,
            'enable_depth':           True,
            'align_depth.enable':     True,
            'color_width':            1280,
            'color_height':           720,
            'color_fps':              30.0,
            'depth_width':            1280,
            'depth_height':           720,
            'depth_fps':              30.0,
            'pointcloud.enable':      False,
            'camera_name':            'front_camera',
            'base_frame_id':          'front_camera_link',
            'tf_publish_rate':        0.0,
        }],
    )

    # ── NODE 5.1: Local ros2_control (fake hardware) for RViz execution ──
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_local_ros2_control')),
        parameters=[
            {'robot_description': robot_description_xml, 'use_sim_time': False},
            ros2_controllers_file,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_local_ros2_control')),
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_local_ros2_control')),
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('use_joint_state_publisher'),
            "' == 'true' and '",
            LaunchConfiguration('use_local_ros2_control'),
            "' == 'false'",
        ])),
        parameters=[{'use_sim_time': False}],
    )

    # ── NODE 6: MoveIt2 move_group (delayed until controllers are active) ──
    move_group = TimerAction(period=2.5, actions=[Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time':  False,
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'num_planning_attempts': 5,
                'planning_time': 5.0,
                'execution_duration_monitoring': True,
                'allowed_execution_duration_scaling': 1.5,
                'allowed_goal_duration_margin': 0.5,
                'trajectory_execution/allowed_start_tolerance': 0.05,
            },
        ],
    )])

    # ── NODE 7: ArUco detector (t=3s, after cameras start) ───────────
    aruco_node = TimerAction(period=3.0, actions=[Node(
        package='warehouse_aruco_detection',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[
            cfg_file,
            {
                'marker_size_m': LaunchConfiguration('marker_size_m'),
                'show_window':   LaunchConfiguration('show_window'),
                'use_sim_time':  False,
            },
        ],
    )])

    # ── NODE 8: RViz2 (t=5s) ────────────────────────────────────────
    rviz = TimerAction(period=5.0, actions=[Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', os.path.join(moveit_pkg, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': False},
        ],
    )])

    # ── NODE 9: Pick-and-place logic (t=8s) ─────────────────────────
    pick_place = TimerAction(period=8.0, actions=[Node(
        package='warehouse_pick_place',
        executable='pick_place_logic',
        name='pick_place_logic',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_pick_place')),
        parameters=[{'use_sim_time': False}],
    )])

    log = LogInfo(msg=(
        '\n[aruco_real_robot] GP12 + 2x D455 startup sequence:\n'
        '  t=0s  robot_state_publisher + 2x static TF + 2x RealSense\n'
        '       + offline joint_state_publisher by default\n'
        '       + optional local mock ros2_control (disables joint_state_publisher)\n'
        '  t=2.5s MoveIt2 move_group (after controller bring-up)\n'
        '  t=3s  ArUco detector (top + front camera)\n'
        '  t=5s  RViz2\n'
        '  t=8s  Pick-and-place logic (disabled by default)\n'
        '\n  Topics to verify:\n'
        '    ros2 topic hz /top_camera/color/image_raw\n'
        '    ros2 topic hz /front_camera/color/image_raw\n'
        '    ros2 topic echo /object_pose\n'
        '    ros2 run rqt_image_view rqt_image_view  -> /aruco/debug_image\n'
        '\n  Optional flags:\n'
        '    use_joint_state_publisher:=false  # disable when MotoROS2 publishes /joint_states\n'
        '    use_local_ros2_control:=true   # local mock controller for RViz execution\n'
        '    use_pick_place:=true           # requires MoveItPy Python bindings installed\n'
        '\n  Before running, update aruco_config.yaml with your camera serial numbers!\n'
    ))

    return LaunchDescription(args + [
        log,
        robot_state_pub,
        world_base_tf,
        top_camera_tf,
        front_camera_tf,
        top_camera,
        front_camera,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        joint_state_publisher,
        extrinsic_calibrator,
        move_group,
        aruco_node,
        rviz,
        pick_place,
    ])
