#!/usr/bin/env python3
import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


ARUCO_DICTS = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
    'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
    'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def matrix_to_quaternion(R: np.ndarray):
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def average_rotation(mats):
    # Average in quaternion space, then normalize.
    quats = []
    for R in mats:
        quats.append(np.array(matrix_to_quaternion(R), dtype=np.float64))
    q = np.mean(np.array(quats), axis=0)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
    return R


class ExtrinsicCalibrator(Node):
    def __init__(self):
        super().__init__('extrinsic_calibrator')

        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size_m', 0.10)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('top_camera_frame', 'top_camera_color_optical_frame')
        self.declare_parameter('front_camera_frame', 'front_camera_color_optical_frame')
        self.declare_parameter('reference_markers_json', '{"0":{"x":0.75,"y":0.0,"z":0.0,"yaw":0.0}}')
        self.declare_parameter('calib_min_markers', 1)
        self.declare_parameter('calib_alpha', 0.2)

        dict_name = self.get_parameter('aruco_dict').value
        self._marker_size = float(self.get_parameter('marker_size_m').value)
        self._world_frame = self.get_parameter('world_frame').value
        self._top_frame = self.get_parameter('top_camera_frame').value
        self._front_frame = self.get_parameter('front_camera_frame').value
        self._min_markers = int(self.get_parameter('calib_min_markers').value)
        self._alpha = float(self.get_parameter('calib_alpha').value)

        refs_raw = self.get_parameter('reference_markers_json').value
        self._ref_markers = {}
        parsed = json.loads(refs_raw)
        for k, v in parsed.items():
            mid = int(k)
            self._ref_markers[mid] = (
                float(v.get('x', 0.0)),
                float(v.get('y', 0.0)),
                float(v.get('z', 0.0)),
                float(v.get('yaw', 0.0)),
            )

        aruco_dict_id = ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
        _adict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        _params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(_adict, _params)

        self._bridge = CvBridge()
        self._tf_pub = TransformBroadcaster(self)

        self._state = {
            'top': {'K': None, 'dist': None, 'frame': self._top_frame, 'R': None, 't': None},
            'front': {'K': None, 'dist': None, 'frame': self._front_frame, 'R': None, 't': None},
        }

        self.create_subscription(CameraInfo, '/top_camera/color/camera_info', self._top_info_cb, 10)
        self.create_subscription(Image, '/top_camera/color/image_raw', self._top_image_cb, 10)
        self.create_subscription(CameraInfo, '/front_camera/color/camera_info', self._front_info_cb, 10)
        self.create_subscription(Image, '/front_camera/color/image_raw', self._front_image_cb, 10)
        self.create_timer(0.1, self._publish_tf)

        self.get_logger().info(
            'Extrinsic calibrator enabled. Using reference markers: '
            f'{sorted(list(self._ref_markers.keys()))}'
        )

    def _top_info_cb(self, msg: CameraInfo):
        self._set_intrinsics('top', msg)

    def _front_info_cb(self, msg: CameraInfo):
        self._set_intrinsics('front', msg)

    def _set_intrinsics(self, key: str, msg: CameraInfo):
        st = self._state[key]
        st['K'] = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        st['dist'] = np.array(msg.d, dtype=np.float64)

    def _top_image_cb(self, msg: Image):
        self._update_from_image('top', msg)

    def _front_image_cb(self, msg: Image):
        self._update_from_image('front', msg)

    def _update_from_image(self, key: str, msg: Image):
        st = self._state[key]
        if st['K'] is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            return

        R_list = []
        t_list = []
        half = self._marker_size * 0.5
        # Marker corners in local marker frame (+Z out of marker plane)
        marker_obj = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

        for i, mid_arr in enumerate(ids):
            mid = int(mid_arr[0])
            if mid not in self._ref_markers:
                continue

            xw, yw, zw, yaw = self._ref_markers[mid]
            R_w_m = rot_z(yaw)
            t_w_m = np.array([[xw], [yw], [zw]], dtype=np.float64)

            ok, rvec, tvec = cv2.solvePnP(
                marker_obj,
                corners[i][0].astype(np.float64),
                st['K'],
                st['dist'],
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue
            R_c_m, _ = cv2.Rodrigues(rvec)
            t_c_m = tvec.reshape(3, 1)

            R_m_c = R_c_m.T
            t_m_c = -R_c_m.T @ t_c_m

            R_w_c = R_w_m @ R_m_c
            t_w_c = R_w_m @ t_m_c + t_w_m

            R_list.append(R_w_c)
            t_list.append(t_w_c)

        if len(R_list) < self._min_markers:
            return

        R_new = average_rotation(R_list)
        t_new = np.mean(np.hstack(t_list), axis=1, keepdims=True)

        if st['R'] is None:
            st['R'] = R_new
            st['t'] = t_new
        else:
            st['R'] = average_rotation([st['R'], R_new])
            st['t'] = (1.0 - self._alpha) * st['t'] + self._alpha * t_new

    def _publish_tf(self):
        now = self.get_clock().now().to_msg()
        for key in ('top', 'front'):
            st = self._state[key]
            if st['R'] is None or st['t'] is None:
                continue
            qx, qy, qz, qw = matrix_to_quaternion(st['R'])
            tfm = TransformStamped()
            tfm.header.stamp = now
            tfm.header.frame_id = self._world_frame
            tfm.child_frame_id = st['frame']
            tfm.transform.translation.x = float(st['t'][0, 0])
            tfm.transform.translation.y = float(st['t'][1, 0])
            tfm.transform.translation.z = float(st['t'][2, 0])
            tfm.transform.rotation.x = float(qx)
            tfm.transform.rotation.y = float(qy)
            tfm.transform.rotation.z = float(qz)
            tfm.transform.rotation.w = float(qw)
            self._tf_pub.sendTransform(tfm)


def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
