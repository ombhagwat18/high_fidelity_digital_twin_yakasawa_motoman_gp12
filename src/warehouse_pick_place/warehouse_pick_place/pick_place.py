#!/usr/bin/env python3
"""
pick_place.py  —  PickPlaceExecutor  (GP12 scale)
==================================================
FK constants updated for GP12 joint origins derived from STL mesh analysis:

  base → joint1  : Tz = 0.1745
  joint1 → joint2: Tz = 0.114   (S body height)
  joint2 → joint3: Tz = 0.690   (upper arm)
  joint3 → joint4: Tx = 0.265, Tz = 0.196  (elbow offset)
  joint4 → joint5: Tx = -0.440  (forearm length)
  joint5 → joint6: Tx = 0.149   (wrist length)
  joint6 → tool0 : Tx = 0.006

GP12 world targets:
  PICK   : X=0.80  Y=0.00  Z=0.54  (product centre on conveyor belt)
  BIN A  : X=0.00  Y=+0.90  Z=0.25
  BIN B  : X=0.00  Y=-0.90  Z=0.25

Why joint-space IK?
  Cartesian pose goals constrain both position AND orientation.
  The GP12's natural tool orientation varies with joint1 (waist yaw).
  A fixed quaternion would cause OMPL IK failures.
  Joint-space goals bypass the orientation constraint.
  j1 = atan2(ty, tx) always points the arm at the target.
"""

import math
import os
import time

import numpy as np
import xacro

from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import rclpy


# ── Gripper ───────────────────────────────────────────────────────────────────
# The end effector is a vacuum suction cup, not a jaw gripper — there is no
# "open/close" joint to command. Control is a real solenoid-style on/off
# (warehouse_gripper_control's VacuumGripperPlugin, mirroring how an industrial
# vacuum gripper is actually wired: digital suction on/off + a vacuum-switch
# feedback line confirming grip), not a fake FollowJointTrajectory goal.
VACUUM_ON_SRV    = '/vacuum_gripper/on'
VACUUM_OFF_SRV   = '/vacuum_gripper/off'
VACUUM_STATE_TOPIC = '/vacuum_gripper/state'
VACUUM_CONFIRM_TIMEOUT = 2.0   # s — how long to wait for a confirmed grip

# ── Arm ───────────────────────────────────────────────────────────────────────
ARM_JOINTS = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']

# ── Approach geometry ─────────────────────────────────────────────────────────
# GP12 gripper fingertip depth from tool0 ≈ 0.08 m
APPROACH_DEPTH = 0.08    # m — pull back so fingers land on box not past it
PRE_APPROACH   = 0.10    # m — extra retract for pre/post grasp waypoint
PRE_LIFT       = 0.08    # m — Z raise for pre/post grasp

# ── IK scan resolution ────────────────────────────────────────────────────────
IK_STEP = 0.08           # rad — finer than old 0.10 for GP12's longer reach

# ── GP12 joint limits (from URDF / datasheet) ─────────────────────────────────
J_LIM = [
    (-math.pi,        math.pi       ),   # j1 S  ±180°
    (-math.pi/2,      2.705         ),   # j2 L  -90°/+155°
    (-3.054,          4.451         ),   # j3 U  -175°/+255°
    (-3.491,          3.491         ),   # j4 R  ±200°
    (-0.873,          3.665         ),   # j5 B  -50°/+210°
    (-2*math.pi,      2*math.pi     ),   # j6 T  ±360°
]


# ── GP12 Forward Kinematics ───────────────────────────────────────────────────
#
# Derived from mesh-verified joint origins in warehouse_robot.urdf.xacro:
#
#   world → base_link :  Tz=0          (base sits on ground)
#   base_link → j1    :  Tz=0.1745     (top of base casting)
#   j1 → link1        :  (S rotation axis Z)
#   link1 → j2        :  Tz=0.114      (top of S body)
#   j2 → link2        :  (L rotation axis Y)
#   link2 → j3        :  Ty=-0.011 Tz=0.690  (top of upper arm)
#   j3 → link3        :  (U rotation axis Y)
#   link3 → j4        :  Tx=0.265 Tz=0.196   (elbow face)
#   j4 → link4        :  (R rotation axis X)
#   link4 → j5        :  Tx=-0.440             (forearm end)
#   j5 → link5        :  (B rotation axis Y)
#   link5 → j6        :  Tx=0.149              (wrist end)
#   j6 → link6        :  (T rotation axis X)
#   link6 → tool0     :  Tx=0.006  Ry=90°

def _fk(q1, q2, q3, q4, q5, q6):
    """Return 4×4 tool0 transform (world frame) for given GP12 joint angles."""
    def Rz(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]], dtype=float)
    def Ry(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]], dtype=float)
    def Rx(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]], dtype=float)
    def Tr(x, y, z):
        return np.array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]], dtype=float)

    T = np.eye(4)
    # base_link origin → joint1 (Z yaw)
    T = T @ Tr(0,      0,      0.1745) @ Rz(q1)
    # joint1 → joint2 (Y pitch) — top of link1
    T = T @ Tr(0,      0,      0.114 ) @ Ry(q2)
    # joint2 → joint3 (Y pitch) — top of upper arm (small Y offset)
    T = T @ Tr(0,     -0.011,  0.690 ) @ Ry(q3)
    # joint3 → joint4 (X roll) — elbow face
    T = T @ Tr(0.265,  0,      0.196 ) @ Rx(q4)
    # joint4 → joint5 (Y pitch) — end of forearm
    T = T @ Tr(-0.440, 0,      0     ) @ Ry(q5)
    # joint5 → joint6 (X roll) — wrist end
    T = T @ Tr(0.149,  0,      0     ) @ Rx(q6)
    # joint6 → tool0 (6mm + Ry90 so Z points forward)
    T = T @ Tr(0.006,  0,      0     ) @ Ry(math.pi/2)
    return T


def _ik_position(tx: float, ty: float, tz: float, step: float = IK_STEP):
    """
    Position-only IK via FK scan.

    j1 = atan2(ty, tx) points the arm at the target.
    Scan j2, j3, j5 (j4=j6=0) to minimise 3-D distance from tool0 to target.

    Returns [j1, j2, j3, 0.0, j5, 0.0] or None if target is unreachable
    within 30 mm tolerance.
    """
    j1 = math.atan2(ty, tx)

    best_err = 999.0
    best     = None

    j2_range = np.arange(J_LIM[1][0], J_LIM[1][1] + step, step)
    j3_range = np.arange(J_LIM[2][0], J_LIM[2][1] + step, step)
    j5_range = np.arange(J_LIM[4][0], J_LIM[4][1] + step, step)

    for j2 in j2_range:
        for j3 in j3_range:
            for j5 in j5_range:
                T   = _fk(j1, j2, j3, 0, j5, 0)
                p   = T[:3, 3]
                err = (p[0]-tx)**2 + (p[1]-ty)**2 + (p[2]-tz)**2
                if err < best_err:
                    best_err = err
                    best     = [j1, float(j2), float(j3), 0.0, float(j5), 0.0]

    if best is None:
        return None
    T   = _fk(*best)
    p   = T[:3, 3]
    err = math.sqrt((p[0]-tx)**2 + (p[1]-ty)**2 + (p[2]-tz)**2)
    return best if err < 0.030 else None   # 30 mm tolerance


# Flip if Gazebo verification shows the gripper aligning opposite the marker.
WRIST_YAW_SIGN = 1.0


def _wrist_j6_for_yaw(yaw: float, j1: float) -> float:
    """
    Approximate the j6 (wrist "T"-axis) command needed so the gripper's
    rotation about its vertical approach axis matches a parcel's world-frame
    marker yaw, for orientation-aware grasping of non-square parcels.

    APPROXIMATION, not a closed-form solution: assumes world yaw ≈ j1 + j6,
    which holds when the tool approaches close to straight down (true for
    this arm's low-j2/j3/j5 belt-height pick posture, since j6 is the last
    joint before tool0 and the intervening Y/X-axis joints keep the tool
    close to vertical for these targets) — a full solution would require
    orientation-aware IK, which _ik_position deliberately does not do (see
    this module's "Why joint-space IK?" docstring).

    Not verified against a running Gazebo instance. If the gripper visibly
    grasps at the wrong angle, flip WRIST_YAW_SIGN above, or measure the
    real offset in sim and add it as a constant here.
    """
    j6 = WRIST_YAW_SIGN * (yaw - j1)
    lo, hi = J_LIM[5]
    while j6 > hi:
        j6 -= 2 * math.pi
    while j6 < lo:
        j6 += 2 * math.pi
    return max(lo, min(hi, j6))


# ─────────────────────────────────────────────────────────────────────────────

class PickPlaceExecutor:

    def __init__(self, node: Node):
        self._node   = node
        self._logger = node.get_logger()

        robot_desc_pkg = get_package_share_directory('warehouse_robot_description')
        moveit_pkg     = get_package_share_directory('warehouse_moveit_config')

        xacro_file = os.path.join(robot_desc_pkg, 'urdf', 'gp12.xacro')
        srdf_file  = os.path.join(moveit_pkg,     'config', 'warehouse_robot.srdf')

        urdf_xml = xacro.process_file(xacro_file).toxml()
        meshes_path = os.path.join(robot_desc_pkg, 'meshes')
        urdf_xml = urdf_xml.replace(
            'package://warehouse_robot_description/meshes',
            'file://' + meshes_path
        )
        with open(srdf_file) as f:
            srdf_xml = f.read()

        moveit_config = {
            'robot_description':          urdf_xml,
            'robot_description_semantic': srdf_xml,
            'robot_description_kinematics': {
                'arm': {
                    'kinematics_solver':
                        'kdl_kinematics_plugin/KDLKinematicsPlugin',
                    'kinematics_solver_search_resolution': 0.001,
                    'kinematics_solver_timeout':           0.05,
                    'kinematics_solver_attempts':          10,
                },
            },
            'planning_pipelines': {'pipeline_names': ['ompl']},
            'moveit_simple_controller_manager': {
                'controller_names': ['controller_manager'],
                'controller_manager': {
                    'type': 'FollowJointTrajectory',
                    'action_ns': 'follow_joint_trajectory',
                    'default': True,
                    'joints': [
                        'joint_1_s', 'joint_2_l', 'joint_3_u',
                        'joint_4_r', 'joint_5_b', 'joint_6_t',
                    ],
                },
            },
            'ompl': {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': (
                    'default_planner_request_adapters/AddTimeOptimalParameterization '
                    'default_planner_request_adapters/ResolveConstraintFrames '
                    'default_planner_request_adapters/FixWorkspaceBounds '
                    'default_planner_request_adapters/FixStartStateBounds '
                    'default_planner_request_adapters/FixStartStateCollision '
                    'default_planner_request_adapters/FixStartStatePathConstraints'
                ),
                'start_state_max_bounds_error': 0.1,
                'arm': {
                    'default_planner_config':         'RRTConnect',
                    'longest_valid_segment_fraction':  0.005,
                },
                'planner_configs': {
                    'RRTConnect': {'type': 'geometric::RRTConnect', 'range': 0.0},
                    'RRT':        {'type': 'geometric::RRT',        'range': 0.0, 'goal_bias': 0.05},
                    'RRTstar':    {'type': 'geometric::RRTstar',    'range': 0.0, 'goal_bias': 0.05},
                },
            },
        }

        self._moveit = MoveItPy(node_name='pick_place_moveit_py',
                                config_dict=moveit_config)
        self._arm    = self._moveit.get_planning_component('arm')
        self._logger.info('MoveItPy initialised.')

        self._plan_params = PlanRequestParameters(self._moveit)
        self._plan_params.planning_pipeline = 'ompl'
        self._plan_params.planner_id        = 'RRTConnect'
        self._plan_params.planning_attempts = 5
        self._plan_params.planning_time     = 5.0

        # JointTrajectoryController exposes its action under its OWN name
        # (arm_controller), not under /controller_manager.
        self._arm_ac = ActionClient(
            node, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        self._vacuum_on_cli  = node.create_client(Trigger, VACUUM_ON_SRV)
        self._vacuum_off_cli = node.create_client(Trigger, VACUUM_OFF_SRV)
        self._vacuum_state   = False
        node.create_subscription(
            Bool, VACUUM_STATE_TOPIC, self._vacuum_state_cb, 10)

        # Tunable approach/grasp geometry — see config/pick_place_params.yaml.
        # Declared on the SAME node as pick_place_logic.py (PickPlaceExecutor
        # is a plain class, not its own Node), so these live under the
        # `pick_place_logic` parameter namespace.
        node.declare_parameter('approach_depth_m', APPROACH_DEPTH)
        node.declare_parameter('pre_approach_m', PRE_APPROACH)
        node.declare_parameter('pre_lift_m', PRE_LIFT)
        node.declare_parameter('vacuum_confirm_timeout_s', VACUUM_CONFIRM_TIMEOUT)
        self._approach_depth = float(node.get_parameter('approach_depth_m').value)
        self._pre_approach   = float(node.get_parameter('pre_approach_m').value)
        self._pre_lift       = float(node.get_parameter('pre_lift_m').value)
        self._vacuum_confirm_timeout = float(
            node.get_parameter('vacuum_confirm_timeout_s').value)

        self._logger.info('Waiting for action/service servers ...')
        self._arm_ac.wait_for_server(timeout_sec=30.0)
        self._vacuum_on_cli.wait_for_service(timeout_sec=30.0)
        self._vacuum_off_cli.wait_for_service(timeout_sec=30.0)
        self._logger.info('Action/service servers connected.')

    # ── Main API ──────────────────────────────────────────────────────────────

    def pick_and_place(self, object_pose: Pose,
                       place_pose: Pose,
                       yaw: float = 0.0,
                       shape: str = 'box') -> bool:

        self._logger.info(f'=== PICK & PLACE  shape={shape}  yaw={math.degrees(yaw):.1f}deg ===')

        ox = object_pose.position.x   # ≈ 0.80
        oy = object_pose.position.y   # ≈ ±0.40
        # Object Z already includes the per-shape grasp_z_offset applied by
        # parcel_perception.py (from parcel_catalog.yaml) — nothing shape-
        # specific to add here.
        oz = object_pose.position.z

        px = place_pose.position.x
        py = place_pose.position.y
        pz = place_pose.position.z

        # Pre-grasp / post-grasp: retract from box + lift
        pre_gx = ox - self._approach_depth - self._pre_approach
        pre_gz = oz + self._pre_lift
        # Grasp: retract approach_depth so fingertips reach box centre
        grx    = ox - self._approach_depth
        # Pre-place: above bin
        pre_pz = pz + 0.15

        ik_pre_grasp = _ik_position(pre_gx, oy,  pre_gz)
        ik_grasp     = _ik_position(grx,    oy,  oz    )
        ik_pre_place = _ik_position(px,     py,  pre_pz)
        ik_place     = _ik_position(px,     py,  pz    )

        # Align the wrist with the parcel's marker yaw for the grasp waypoints
        # only (placement orientation doesn't need to match the marker).
        # See _wrist_j6_for_yaw's docstring for the approximation this relies
        # on and how to tune it after visually checking alignment in Gazebo.
        for ik in (ik_pre_grasp, ik_grasp):
            if ik is not None:
                ik[5] = _wrist_j6_for_yaw(yaw, ik[0])

        for name, ik in [('pre_grasp', ik_pre_grasp), ('grasp',     ik_grasp),
                         ('pre_place', ik_pre_place), ('place',     ik_place)]:
            if ik is None:
                self._logger.error(f'IK failed for {name} — target unreachable.')
                return False
            self._logger.info(
                f'  IK {name}: j1={math.degrees(ik[0]):.1f}°  '
                f'j2={ik[1]:.3f}  j3={ik[2]:.3f}  j5={ik[4]:.3f}  j6={ik[5]:.3f}'
            )

        steps = [
            ('go_home',        lambda: self._go_home()),
            ('vacuum_off',     lambda: self._vacuum_off()),
            ('pre_grasp',      lambda: self._move_arm_joints(ik_pre_grasp)),
            ('grasp',          lambda: self._move_arm_joints(ik_grasp)),
            ('vacuum_on',      lambda: self._vacuum_on_and_confirm()),
            ('post_grasp',     lambda: self._move_arm_joints(ik_pre_grasp)),
            ('pre_place',      lambda: self._move_arm_joints(ik_pre_place)),
            ('place',          lambda: self._move_arm_joints(ik_place)),
            ('vacuum_off2',    lambda: self._vacuum_off()),
            ('go_home_final',  lambda: self._go_home()),
        ]

        for step_name, fn in steps:
            self._logger.info(f'  → {step_name}')
            if not fn():
                self._logger.error(f'Step FAILED: {step_name} — aborting cycle.')
                self._vacuum_off()
                self._go_home()
                return False

        self._logger.info('=== Cycle COMPLETE ===')
        return True

    def go_home(self):       return self._go_home()
    def vacuum_on(self):     return self._vacuum_on_and_confirm()
    def vacuum_off(self):    return self._vacuum_off()

    # ── Arm ───────────────────────────────────────────────────────────────────

    def _go_home(self) -> bool:
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(configuration_name='home')
        return self._plan_and_execute_arm('home')

    def _move_arm_joints(self, joint_values: list) -> bool:
        constraints = Constraints()
        for name, value in zip(ARM_JOINTS, joint_values):
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = value
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(motion_plan_constraints=[constraints])
        return self._plan_and_execute_arm('joints')

    def _plan_and_execute_arm(self, tag: str) -> bool:
        plan_result = self._arm.plan(parameters=self._plan_params)
        if not plan_result:
            self._logger.error(f'Planning failed [{tag}]')
            return False

        traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
        traj     = traj_msg.joint_trajectory

        goal            = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._arm_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        if not future.result() or not future.result().accepted:
            self._logger.error(f'Arm goal rejected [{tag}]')
            return False

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=60.0)
        result = result_future.result()
        if result and result.result.error_code in (
                FollowJointTrajectory.Result.SUCCESSFUL, 0):
            return True

        code = result.result.error_code if result else 'timeout'
        self._logger.error(f'Arm execution failed [{tag}] code={code}')
        return False

    # ── Vacuum gripper ────────────────────────────────────────────────────────

    def _vacuum_state_cb(self, msg: Bool):
        self._vacuum_state = bool(msg.data)

    def _call_vacuum(self, client, tag: str) -> bool:
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.success:
            self._logger.error(
                f'Vacuum {tag} call failed: '
                f'{result.message if result else "timeout"}'
            )
            return False
        return True

    def _vacuum_on_and_confirm(self) -> bool:
        """Turn suction on, then require the gripper's own vacuum-switch
        feedback (/vacuum_gripper/state) to confirm an object actually
        attached — matches how a real vacuum gripper reports grip, and
        catches missed/failed grasps instead of blindly assuming success."""
        if not self._call_vacuum(self._vacuum_on_cli, 'on'):
            return False

        deadline = time.monotonic() + self._vacuum_confirm_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._vacuum_state:
                return True
        self._logger.error(
            'Vacuum ON but no grip confirmed within '
            f'{self._vacuum_confirm_timeout}s — treating as a failed grasp.'
        )
        return False

    def _vacuum_off(self) -> bool:
        return self._call_vacuum(self._vacuum_off_cli, 'off')
