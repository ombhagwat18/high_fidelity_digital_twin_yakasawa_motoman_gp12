#!/usr/bin/env python3
"""
bringup.launch.py
warehouse_bringup package

Launches the complete GP12 digital twin:
  1. Gazebo + robot spawn + controllers  (via gazebo.launch.py)
  2. MoveIt2 move_group
  3. RViz2 with MoveIt plugin
  4. Parcel perception (ArUco-based, parcel_perception) + simulated
     YRC1000 controller (yrc1000_controller)
  5. Pick-and-place logic node
  6. Product spawner (mixed-shape ArUco-tagged parcels on the conveyor)

Timing ladder (all times relative to launch start):
  t=1    Simulated YRC1000 controller starts (independent of the sim ladder)
  t=0    Gazebo + gzserver start
  t=5    Robot spawned into world
  t=12   joint_state_broadcaster activated
  t=14   arm_controller activated
  t=15   Parcel perception starts (camera topics live from t=5)
  t=17   move_group starts (controllers confirmed active)
  t=20   RViz2 starts (move_group up)
  t=22   Pick-and-place logic starts (move_group + perception ready)
  t=25   Product spawner starts (everything ready)

Usage:
    ros2 launch warehouse_bringup bringup.launch.py
    ros2 launch warehouse_bringup bringup.launch.py use_rviz:=false
    ros2 launch warehouse_bringup bringup.launch.py use_operator_view:=false
    ros2 launch warehouse_bringup bringup.launch.py world:=warehouse_industrial.world
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Package paths
    sim_pkg      = get_package_share_directory('warehouse_gazebo')
    moveit_pkg   = get_package_share_directory('warehouse_moveit_config')
    bringup_pkg  = get_package_share_directory('warehouse_bringup')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2'
    )
    use_operator_view_arg = DeclareLaunchArgument(
        'use_operator_view', default_value='true',
        description=(
            'When true (default), RViz uses the operator dashboard '
            '(planning scene + both camera feeds + ArUco debug + TF) instead '
            'of the bare MoveIt planning-scene-only view.'
        )
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value='warehouse_industrial.world',
        description='Gazebo world file name (with .world extension)'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    use_operator_view = LaunchConfiguration('use_operator_view')

    # MoveIt config
    moveit_config = (
        MoveItConfigsBuilder('warehouse_robot', package_name='warehouse_moveit_config')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    moveit_only_rviz_config = os.path.join(moveit_pkg, 'config', 'moveit.rviz')
    operator_rviz_config    = os.path.join(bringup_pkg, 'config', 'operator.rviz')
    diagnostics_config      = os.path.join(bringup_pkg, 'config', 'diagnostics.yaml')

    # ------------------------------------------------------------------
    # LAYER 1 - Gazebo simulation
    # gazebo.launch.py handles:
    #   - GAZEBO_MODEL_PATH / GAZEBO_MODEL_DATABASE_URI env fixes
    #   - package:// -> file:// mesh URI substitution
    #   - XML comment stripping (fixes gazebo_ros2_control rcl crash)
    #   - Timed robot spawn at t=5s
    #   - joint_state_broadcaster at t=12s
    #   - arm_controller at t=14s
    # ------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # ------------------------------------------------------------------
    # LAYER 2 - MoveIt2 move_group
    # Starts at t=17s: arm_controller confirmed active at t=14s,
    # +3s buffer for /joint_states to stabilise.
    # ------------------------------------------------------------------
    move_group = TimerAction(
        period=17.0,
        actions=[Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {
                    'use_sim_time': True,
                    'planning_plugin': 'ompl_interface/OMPLPlanner',
                    'num_planning_attempts': 5,
                    'planning_time': 5.0,
                    'execution_duration_monitoring': True,
                    'allowed_execution_duration_scaling': 1.5,
                    'allowed_goal_duration_margin': 0.5,
                    'trajectory_execution/allowed_execution_duration_scaling': 1.5,
                    'trajectory_execution/allowed_goal_duration_margin': 0.5,
                    'trajectory_execution/allowed_start_tolerance': 0.05,
                },
            ],
        )]
    )

    # ------------------------------------------------------------------
    # LAYER 3 - RViz2
    # Starts at t=20s: move_group needs ~3s to finish loading after start.
    # Two mutually-exclusive Node entries (both gated on use_rviz) pick the
    # config file based on use_operator_view — RViz needs a concrete file
    # path per-Node, so this can't be a single Node with a computed path.
    # ------------------------------------------------------------------
    rviz_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        {'use_sim_time': True},
    ]

    rviz_operator_view = TimerAction(
        period=20.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            condition=IfCondition(PythonExpression([
                "'", use_rviz, "' == 'true' and '", use_operator_view, "' == 'true'"
            ])),
            arguments=['-d', operator_rviz_config],
            parameters=rviz_params,
        )]
    )

    rviz_moveit_only = TimerAction(
        period=20.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            condition=IfCondition(PythonExpression([
                "'", use_rviz, "' == 'true' and '", use_operator_view, "' == 'false'"
            ])),
            arguments=['-d', moveit_only_rviz_config],
            parameters=rviz_params,
        )]
    )

    # ------------------------------------------------------------------
    # LAYER 4 - Perception
    # Camera topics are live from t=5s (robot spawn).
    # Start at t=15s to ensure Gazebo is fully stable.
    # ------------------------------------------------------------------
    perception_pkg = get_package_share_directory('warehouse_perception')
    perception_params = os.path.join(
        perception_pkg, 'config', 'parcel_perception.yaml')

    parcel_perception_node = TimerAction(
        period=15.0,
        actions=[Node(
            package='warehouse_perception',
            executable='parcel_perception',
            name='parcel_perception',
            output='screen',
            parameters=[perception_params, {'use_sim_time': True}],
        )]
    )

    # ------------------------------------------------------------------
    # LAYER 4b - Simulated YRC1000 controller
    # Starts early (t=1s) and independently of the sim timing ladder — a
    # real controller would already be up before Gazebo even starts, and
    # pick_place_logic gates every cycle on its /yrc1000/status.
    # ------------------------------------------------------------------
    controller_pkg = get_package_share_directory('warehouse_controller')
    controller_params = os.path.join(controller_pkg, 'config', 'yrc1000_params.yaml')

    yrc1000_controller_node = TimerAction(
        period=1.0,
        actions=[Node(
            package='warehouse_controller',
            executable='yrc1000_controller',
            name='yrc1000_controller',
            output='screen',
            parameters=[controller_params],
        )]
    )

    # ------------------------------------------------------------------
    # LAYER 5 - Pick-and-place logic
    # Starts at t=22s: move_group(17s) + ~5s for action servers to be ready
    # and perception nodes to publish first detections.
    # ------------------------------------------------------------------
    pick_place_pkg = get_package_share_directory('warehouse_pick_place')
    pick_place_params = os.path.join(
        pick_place_pkg, 'config', 'pick_place_params.yaml')

    pick_place_node = TimerAction(
        period=22.0,
        actions=[Node(
            package='warehouse_pick_place',
            executable='pick_place_logic',
            name='pick_place_logic',
            output='screen',
            parameters=[pick_place_params, {'use_sim_time': True}],
        )]
    )

    # ------------------------------------------------------------------
    # LAYER 4c - Diagnostic aggregator
    # Groups /diagnostics from the controller/perception/pick-place/gripper
    # nodes above into subsystems for `ros2 run rqt_robot_monitor
    # rqt_robot_monitor`. Starts alongside the controller (t=1s); it's just
    # a message router, no dependency on the sim being up yet.
    # ------------------------------------------------------------------
    diagnostic_aggregator_node = TimerAction(
        period=1.0,
        actions=[Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            output='screen',
            parameters=[diagnostics_config],
        )]
    )

    # ------------------------------------------------------------------
    # LAYER 6 - Product spawner
    # Starts at t=25s: after pick-and-place is ready to handle products.
    # ------------------------------------------------------------------
    product_spawner = TimerAction(
        period=25.0,
        actions=[Node(
            package='warehouse_gazebo',
            executable='product_spawner',
            name='product_spawner',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )]
    )

    # Status log so user knows what to expect
    startup_log = LogInfo(msg=(
        '\n[warehouse_bringup] Startup sequence:\n'
        '  t= 1s  Simulated YRC1000 controller + diagnostic_aggregator starting\n'
        '  t= 0s  Gazebo starting\n'
        '  t= 5s  Robot spawning\n'
        '  t=12s  joint_state_broadcaster activating\n'
        '  t=14s  arm_controller activating\n'
        '  t=15s  Parcel perception starting\n'
        '  t=17s  MoveIt2 move_group starting\n'
        '  t=20s  RViz2 starting (operator dashboard by default — '
        'use_operator_view:=false for the bare planning-scene view)\n'
        '  t=22s  Pick-and-place logic starting\n'
        '  t=25s  Product spawner starting\n'
        '  Ready: ~30 seconds after launch\n'
        '  Health: ros2 run rqt_robot_monitor rqt_robot_monitor\n'
    ))

    return LaunchDescription([
        use_rviz_arg,
        use_operator_view_arg,
        world_arg,
        startup_log,
        # Layer 1
        gazebo_launch,
        # Layer 2
        move_group,
        # Layer 3
        rviz_operator_view,
        rviz_moveit_only,
        # Layer 4
        parcel_perception_node,
        yrc1000_controller_node,
        diagnostic_aggregator_node,
        # Layer 5
        pick_place_node,
        # Layer 6
        product_spawner,
    ])
