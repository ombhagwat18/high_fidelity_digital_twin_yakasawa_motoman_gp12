#!/usr/bin/env python3
"""
parcel_perception.py — canonical simulated perception node.
==============================================================================
Replaces the previous three independent, mutually-inconsistent perception
attempts (yolo_node.py's HSV colour segmentation, dual_camera_detector.py's
YOLO/RealSense-depth pipeline, and the first cut of aruco_detector_sim.py) with
ONE node built around the actual chosen sensing strategy: an ArUco marker
glued to the top face of every parcel (see warehouse_gazebo's
product_spawner.py + parcel_catalog.yaml). A marker gives a full 6-DOF pose
directly via solvePnP — no separate depth channel, no camera-to-camera pose
fusion, and no hardcoded "all objects sit at height H" assumption is needed,
which is what made the three previous attempts disagree with each other and
with reality in the first place.

Pipeline
--------
  /top_camera_color/image_raw + camera_info
        │  cv2.aruco detect + solvePnP (marker→camera-optical pose)
        ▼
  known, fixed camera extrinsics (top_camera mount in gp12.xacro + the
  0.35 m spawn-height offset from gazebo.launch.py's spawn_entity call)
        │  camera-optical → world transform
        ▼
  world-frame parcel pose (position IS the true top-surface pick point,
  since the marker plane sits directly on the parcel's top face) + yaw
        │  marker_id → shape/dims/bin lookup (parcel_catalog.yaml)
        ▼
  /parcel_detections   warehouse_interfaces/ParcelDetection

The front camera (/front_camera_color/image_raw) is secondary: it only
publishes an annotated debug image on /aruco/debug_front, exactly like the
original aruco_detector_sim.py's front-camera role — it is NOT used for pose
fusion (there is nothing to fuse; the top camera's solvePnP pose is already
the true 6-DOF pose, unlike the old pixel-back-projection approach that
needed a second camera to sanity-check a height assumption).

IMPORTANT — topic names are the thing that broke this pipeline before: three
earlier files each guessed a different image/camera_info topic name and none
of them matched what the gazebo_ros_camera plugin (see
realsense_d435.xacro's ${camera_name}_color/${camera_name}_depth namespaces,
instantiated in gp12.xacro as top_camera/front_camera) actually publishes.
This node assumes `/top_camera_color/image_raw` + `/top_camera_color/camera_info`
(and the front_camera equivalents) — the same assumption yolo_node.py and
depth_pose_estimation.py made (2 of the 3 old files agreed on this). VERIFY
with `ros2 topic list` after launching and fix these two constants if Gazebo
actually names them differently on your installed gazebo_ros version.
"""

import math
import os
import time

import cv2
import numpy as np
import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from warehouse_interfaces.msg import ParcelDetection

DETECTION_STALE_S = 30.0   # no detection in this long -> WARN diagnostic

# ── Camera topic names ────────────────────────────────────────────────────────
TOP_IMAGE_TOPIC    = '/top_camera_color/image_raw'
TOP_INFO_TOPIC     = '/top_camera_color/camera_info'
FRONT_IMAGE_TOPIC  = '/front_camera_color/image_raw'
FRONT_INFO_TOPIC   = '/front_camera_color/camera_info'

# ── Camera extrinsics — MUST match gp12.xacro's top_camera/front_camera
# instantiation (xyz/rpy passed to the realsense_d435 macro) and the spawn
# height in gazebo.launch.py's spawn_entity call ("-z 0.35"). ─────────────────
ROBOT_SPAWN_Z_OFFSET = 0.35

TOP_CAM_XYZ = (0.75, 0.00, 1.50)
TOP_CAM_RPY = (0.0, 1.5708, 0.0)

FRONT_CAM_XYZ = (1.20, 0.00, 0.75)
FRONT_CAM_RPY = (0.0, 0.7854, 0.0)

# realsense_d435.xacro: color_optical_joint is a fixed xyz=0 rpy="-1.5708 0 -1.5708"
# relative to the camera link — standard ROS optical-frame convention.
OPTICAL_RPY = (-1.5708, 0.0, -1.5708)

MIN_CONFIDENCE = 0.4
DETECTION_CONFIDENCE = 0.95   # ArUco pose is geometric, not a learned score

ARUCO_DICTS = {
    'DICT_4X4_50':  cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_5X5_50':  cv2.aruco.DICT_5X5_50,
    'DICT_6X6_50':  cv2.aruco.DICT_6X6_50,
}


# ── Small homogeneous-transform helpers (same style as pick_place.py's _fk) ──

def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def _Tr(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


def _rpy_frame(xyz, rpy):
    """URDF joint origin: translate then rotate R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    x, y, z = xyz
    roll, pitch, yaw = rpy
    return _Tr(x, y, z) @ _Rz(yaw) @ _Ry(pitch) @ _Rx(roll)


def _load_catalog() -> dict:
    share = get_package_share_directory('warehouse_gazebo')
    path = os.path.join(share, 'config', 'parcel_catalog.yaml')
    with open(path) as f:
        data = yaml.safe_load(f)
    return {entry['marker_id']: entry for entry in data['parcels']}


class ParcelPerception(Node):

    def __init__(self):
        super().__init__('parcel_perception')

        self.declare_parameter('marker_size_m', 0.06)   # printed marker black-square size
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('show_window', False)

        self._marker_size = float(self.get_parameter('marker_size_m').value)
        dict_name         = self.get_parameter('aruco_dict').value
        self._show_window = bool(self.get_parameter('show_window').value)

        dict_id = ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
        self._aruco_dict     = cv2.aruco.getPredefinedDictionary(dict_id)
        self._aruco_params   = cv2.aruco.DetectorParameters()
        self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)

        self._catalog = _load_catalog()
        self.get_logger().info(
            f'Loaded {len(self._catalog)} marker ID -> parcel mappings from catalog.')

        # World transform for each camera's optical frame — fixed, computed once.
        self._T_world_top_optical = (
            _Tr(0, 0, ROBOT_SPAWN_Z_OFFSET)
            @ _rpy_frame(TOP_CAM_XYZ, TOP_CAM_RPY)
            @ _rpy_frame((0, 0, 0), OPTICAL_RPY)
        )

        self._K_top    = None
        self._dist_top = None
        self._K_front  = None
        self._bridge   = CvBridge()

        self.create_subscription(
            CameraInfo, TOP_INFO_TOPIC, self._top_info_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, TOP_IMAGE_TOPIC, self._top_image_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, FRONT_INFO_TOPIC, self._front_info_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, FRONT_IMAGE_TOPIC, self._front_image_cb, qos_profile_sensor_data)

        self._det_pub   = self.create_publisher(ParcelDetection, '/parcel_detections', 10)
        self._debug_top   = self.create_publisher(Image, '/aruco/debug_image', 10)
        self._debug_front = self.create_publisher(Image, '/aruco/debug_front', 10)

        self._last_detection_time = None
        self._diag_updater = Updater(self)
        self._diag_updater.setHardwareID('parcel_perception')
        self._diag_updater.add('Perception', self._diagnostics_cb)
        self.create_timer(1.0, self._diag_updater.force_update)

        self.get_logger().info(
            '\n[ParcelPerception] started'
            f'\n  dict={dict_name}  marker_size={self._marker_size} m'
            f'\n  TOP   image : {TOP_IMAGE_TOPIC}'
            f'\n  FRONT image : {FRONT_IMAGE_TOPIC} (debug only)'
            '\n  Waiting for camera_info ...'
        )

    # ── camera_info callbacks ──────────────────────────────────────────────

    def _top_info_cb(self, msg: CameraInfo):
        if self._K_top is not None:
            return
        self._K_top    = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_top = np.array(msg.d, dtype=np.float64)
        self.get_logger().info(
            f'Top camera intrinsics: fx={self._K_top[0,0]:.1f} fy={self._K_top[1,1]:.1f}'
        )

    def _front_info_cb(self, msg: CameraInfo):
        if self._K_front is not None:
            return
        self._K_front = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    # ── top camera: primary detection ──────────────────────────────────────

    def _top_image_cb(self, msg: Image):
        if self._K_top is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Top image bridge error: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._aruco_detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            half = self._marker_size / 2.0
            obj_pts = np.array([
                [-half,  half, 0.], [half,  half, 0.],
                [half, -half, 0.], [-half, -half, 0.]], dtype=np.float64)

            for i, mc in enumerate(corners):
                marker_id = int(ids[i][0])
                entry = self._catalog.get(marker_id)
                if entry is None:
                    self.get_logger().warn(
                        f'Marker ID {marker_id} has no parcel_catalog.yaml entry — skipping.'
                    )
                    continue

                img_pts = mc[0].astype(np.float64)
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, self._K_top, self._dist_top,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue

                R_cam_marker, _ = cv2.Rodrigues(rvec)
                T_optical_marker = np.eye(4)
                T_optical_marker[:3, :3] = R_cam_marker
                T_optical_marker[:3, 3]  = tvec.flatten()

                T_world_marker = self._T_world_top_optical @ T_optical_marker
                wx, wy, wz = T_world_marker[:3, 3]
                yaw = math.atan2(T_world_marker[1, 0], T_world_marker[0, 0])

                self._publish_detection(marker_id, entry, wx, wy, wz, yaw)

                cv2.drawFrameAxes(frame, self._K_top, self._dist_top,
                                  rvec, tvec, self._marker_size * 0.5)
                cv2.putText(
                    frame,
                    f"ID{marker_id}:{entry['shape']}",
                    (int(img_pts[:, 0].mean()) - 40, int(img_pts[:, 1].mean()) - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                self.get_logger().info(
                    f'[TOP ID {marker_id}] {entry["shape"]} -> '
                    f'world=({wx:.3f}, {wy:.3f}, {wz:.3f})  yaw={math.degrees(yaw):.1f}deg'
                )

        self._publish_debug(frame, msg.header, self._debug_top)
        if self._show_window:
            cv2.imshow('TOP ArUco', frame)
            cv2.waitKey(1)

    def _publish_detection(self, marker_id, entry, wx, wy, wz, yaw):
        self._last_detection_time = time.monotonic()
        det = ParcelDetection()
        det.marker_id  = marker_id
        det.shape      = entry['shape']
        det.bin        = entry['bin']
        det.dims_m     = [float(v) for v in entry['dims_m']]
        det.confidence = DETECTION_CONFIDENCE
        det.yaw        = float(yaw)
        det.pose.position.x = float(wx)
        det.pose.position.y = float(wy)
        det.pose.position.z = float(wz) + float(entry.get('grasp_z_offset', 0.0))
        # Gripper-down orientation (tool Z toward -world Z); actual wrist
        # alignment is applied via `yaw` directly by pick_place.py's IK, not
        # by consuming this quaternion (see pick_place.py's joint-space IK
        # docstring for why a fixed quaternion goal isn't used for planning).
        det.pose.orientation.x = 0.0
        det.pose.orientation.y = 0.7071068
        det.pose.orientation.z = 0.0
        det.pose.orientation.w = 0.7071068
        self._det_pub.publish(det)

    # ── front camera: debug/confirmation only ──────────────────────────────

    def _front_image_cb(self, msg: Image):
        if self._K_front is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._aruco_detector.detectMarkers(gray)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        self._publish_debug(frame, msg.header, self._debug_front)
        if self._show_window:
            cv2.imshow('FRONT ArUco', frame)
            cv2.waitKey(1)

    def _publish_debug(self, frame, header, pub):
        try:
            dbg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            dbg.header = header
            pub.publish(dbg)
        except Exception:
            pass

    # ── diagnostics ─────────────────────────────────────────────────────────

    def _diagnostics_cb(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        if self._K_top is None:
            stat.summary(DiagnosticStatus.WARN, 'waiting for top camera_info')
        elif self._last_detection_time is None:
            stat.summary(DiagnosticStatus.WARN, 'camera up, no detections seen yet')
        else:
            age = time.monotonic() - self._last_detection_time
            if age > DETECTION_STALE_S:
                stat.summary(
                    DiagnosticStatus.WARN,
                    f'no detection in {age:.0f}s (belt may be empty/stalled)')
            else:
                stat.summary(DiagnosticStatus.OK, f'last detection {age:.1f}s ago')
        stat.add('top_camera_info_received', str(self._K_top is not None))
        stat.add('front_camera_info_received', str(self._K_front is not None))
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = ParcelPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
