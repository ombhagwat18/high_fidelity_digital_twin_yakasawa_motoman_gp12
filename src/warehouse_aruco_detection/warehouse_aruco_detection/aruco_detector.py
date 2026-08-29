#!/usr/bin/env python3
"""
aruco_detector.py  --  Dual RealSense D455 ArUco detector for real GP12 robot
===============================================================================
Two cameras:
  TOP camera    (namespace: top_camera)    -- overhead, looking straight down
  FRONT camera  (namespace: front_camera)  -- side view, tilted down ~54deg

Topics consumed:
  /top_camera/color/image_raw
  /top_camera/color/camera_info
  /top_camera/aligned_depth_to_color/image_raw
  /front_camera/color/image_raw
  /front_camera/color/camera_info
  /front_camera/aligned_depth_to_color/image_raw

Topics published:
  /object_pose          geometry_msgs/PoseStamped  (world frame, label in frame_id)
  /detections_2d        warehouse_interfaces/Detection2D
  /aruco/debug_image    sensor_msgs/Image  (annotated top-camera feed)
  /aruco/debug_image_front sensor_msgs/Image (annotated front-camera feed)

Parameters (set via aruco_config.yaml):
  marker_size_m       float   Physical black-square side length [m]
  aruco_dict          str     DICT_4X4_50 etc.
  id_label_map        str     JSON: {"0":"good","1":"defective"}
  top_camera_frame    str     TF frame of top camera optical frame
  front_camera_frame  str     TF frame of front camera optical frame
  world_frame         str     Target output frame (world)
  depth_scale         float   D455 depth unit -> metres (0.001)
  depth_roi_px        int     Depth averaging half-window [px]
  show_window         bool    OpenCV imshow (needs display)
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from tf2_ros import (Buffer, TransformListener,
                     LookupException, ConnectivityException,
                     ExtrapolationException)
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped transform
from warehouse_interfaces.msg import Detection2D

ARUCO_DICTS = {
    'DICT_4X4_50':         cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100':        cv2.aruco.DICT_4X4_100,
    'DICT_5X5_50':         cv2.aruco.DICT_5X5_50,
    'DICT_6X6_50':         cv2.aruco.DICT_6X6_50,
    'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('marker_size_m',      0.10)
        self.declare_parameter('aruco_dict',         'DICT_4X4_50')
        self.declare_parameter('id_label_map',       '{"0":"good","1":"defective"}')
        self.declare_parameter('top_camera_frame',   'top_camera_color_optical_frame')
        self.declare_parameter('front_camera_frame', 'front_camera_color_optical_frame')
        self.declare_parameter('world_frame',        'world')
        self.declare_parameter('depth_scale',        0.001)
        self.declare_parameter('depth_roi_px',       5)
        self.declare_parameter('show_window',        False)

        self._marker_size   = self.get_parameter('marker_size_m').value
        dict_name           = self.get_parameter('aruco_dict').value
        self._top_frame     = self.get_parameter('top_camera_frame').value
        self._front_frame   = self.get_parameter('front_camera_frame').value
        self._world_frame   = self.get_parameter('world_frame').value
        self._depth_scale   = self.get_parameter('depth_scale').value
        self._depth_roi     = self.get_parameter('depth_roi_px').value
        self._show_window   = self.get_parameter('show_window').value

        try:
            self._id_label = {
                int(k): v for k, v in json.loads(
                    self.get_parameter('id_label_map').value).items()
            }
        except Exception:
            self._id_label = {0: 'good', 1: 'defective'}

        # ArUco detector
        aruco_dict_id        = ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
        _adict               = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        _params              = cv2.aruco.DetectorParameters()
        self._detector       = cv2.aruco.ArucoDetector(_adict, _params)

        # Per-camera state
        self._top   = {'K': None, 'dist': None, 'depth': None}
        self._front = {'K': None, 'dist': None, 'depth': None}

        self._bridge      = CvBridge()
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos = qos_profile_sensor_data

        # TOP camera subscriptions
        self.create_subscription(CameraInfo, '/top_camera/color/camera_info',
                                 lambda m: self._info_cb(m, self._top), qos)
        self.create_subscription(Image, '/top_camera/color/image_raw',
                                 lambda m: self._image_cb(m, self._top,
                                                          self._top_frame,
                                                          self._top_debug_pub), qos)
        self.create_subscription(Image, '/top_camera/aligned_depth_to_color/image_raw',
                                 lambda m: self._depth_cb(m, self._top), qos)

        # FRONT camera subscriptions
        self.create_subscription(CameraInfo, '/front_camera/color/camera_info',
                                 lambda m: self._info_cb(m, self._front), qos)
        self.create_subscription(Image, '/front_camera/color/image_raw',
                                 lambda m: self._image_cb(m, self._front,
                                                          self._front_frame,
                                                          self._front_debug_pub), qos)
        self.create_subscription(Image, '/front_camera/aligned_depth_to_color/image_raw',
                                 lambda m: self._depth_cb(m, self._front), qos)

        # Publishers
        self._pose_pub        = self.create_publisher(PoseStamped,  '/object_pose',             10)
        self._det2d_pub       = self.create_publisher(Detection2D,  '/detections_2d',           10)
        self._top_debug_pub   = self.create_publisher(Image,        '/aruco/debug_image',       10)
        self._front_debug_pub = self.create_publisher(Image,        '/aruco/debug_image_front', 10)

        self.get_logger().info(
            f'[ArucoDetector] Started -- dict={dict_name} '
            f'marker={self._marker_size}m labels={self._id_label}\n'
            f'  TOP:   /top_camera/color/image_raw\n'
            f'  FRONT: /front_camera/color/image_raw'
        )

    # -------------------------------------------------------------------------
    def _info_cb(self, msg: CameraInfo, state: dict):
        if state['K'] is None:
            state['K']    = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            state['dist'] = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(
                f'Intrinsics received: '
                f'fx={state["K"][0,0]:.1f} fy={state["K"][1,1]:.1f}'
            )

    def _depth_cb(self, msg: Image, state: dict):
        try:
            state['depth'] = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth bridge error: {e}')

    def _image_cb(self, msg: Image, state: dict, cam_frame: str, debug_pub):
        if state['K'] is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image bridge error: {e}')
            return

        gray           = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            self._pub_debug(frame, msg.header, debug_pub)
            return

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        half    = self._marker_size / 2.0
        obj_pts = np.array([[-half,  half, 0.], [ half,  half, 0.],
                             [ half, -half, 0.], [-half, -half, 0.]],
                           dtype=np.float64)

        for i, mc in enumerate(corners):
            mid   = int(ids[i][0])
            label = self._id_label.get(mid, f'unknown_{mid}')
            ipts  = mc[0].astype(np.float64)

            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, ipts, state['K'], state['dist'],
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue

            cx, cy, cz = float(tvec[0]), float(tvec[1]), float(tvec[2])

            # Depth cross-check
            px = int(np.mean(ipts[:, 0]))
            py = int(np.mean(ipts[:, 1]))
            dm = self._read_depth(state, px, py)
            if dm is not None and dm > 0.05:
                cz = 0.8 * cz + 0.2 * dm

            R, _  = cv2.Rodrigues(rvec)
            quat  = self._rot_to_quat(R)

            pose_cam = PoseStamped()
            pose_cam.header.stamp    = msg.header.stamp
            pose_cam.header.frame_id = cam_frame
            pose_cam.pose.position.x = cx
            pose_cam.pose.position.y = cy
            pose_cam.pose.position.z = cz
            pose_cam.pose.orientation.x = quat[0]
            pose_cam.pose.orientation.y = quat[1]
            pose_cam.pose.orientation.z = quat[2]
            pose_cam.pose.orientation.w = quat[3]

            pose_world = self._to_world(pose_cam)
            pose_world.header.frame_id = f'{self._world_frame}/{label}'
            self._pose_pub.publish(pose_world)

            det            = Detection2D()
            det.label      = label
            det.x          = float(px)
            det.y          = float(py)
            det.confidence = 0.95
            self._det2d_pub.publish(det)

            self.get_logger().info(
                f'[ID {mid}] {label}  '
                f'world=({pose_world.pose.position.x:.3f},'
                f'{pose_world.pose.position.y:.3f},'
                f'{pose_world.pose.position.z:.3f})'
            )

            cv2.drawFrameAxes(frame, state['K'], state['dist'],
                              rvec, tvec, self._marker_size * 0.5)
            cv2.putText(frame, f'ID{mid}:{label}',
                        (px - 40, py - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self._pub_debug(frame, msg.header, debug_pub)
        if self._show_window:
            cv2.imshow(f'ArUco {cam_frame}', frame)
            cv2.waitKey(1)

    # -------------------------------------------------------------------------
    def _read_depth(self, state: dict, cx: int, cy: int):
        d = state['depth']
        if d is None:
            return None
        r  = self._depth_roi
        h, w = d.shape[:2]
        roi = d[max(0, cy-r):min(h, cy+r+1),
                max(0, cx-r):min(w, cx+r+1)].astype(np.float32)
        valid = roi[roi > 0]
        return float(np.median(valid)) * self._depth_scale if len(valid) else None

    def _rot_to_quat(self, R):
        tr = R[0,0]+R[1,1]+R[2,2]
        if tr > 0:
            s = 0.5/np.sqrt(tr+1.0)
            return ((R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s,
                    (R[1,0]-R[0,1])*s, 0.25/s)
        elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
            s = 2.0*np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])
            return (0.25*s, (R[0,1]+R[1,0])/s,
                    (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s)
        elif R[1,1]>R[2,2]:
            s = 2.0*np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])
            return ((R[0,1]+R[1,0])/s, 0.25*s,
                    (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s)
        else:
            s = 2.0*np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])
            return ((R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s,
                    0.25*s, (R[1,0]-R[0,1])/s)

    def _to_world(self, pose_cam: PoseStamped) -> PoseStamped:
        try:
            return self._tf_buffer.transform(
                pose_cam, self._world_frame,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF to world failed: {e}', throttle_duration_sec=5.0)
            return pose_cam

    def _pub_debug(self, frame, header, pub):
        try:
            msg        = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header = header
            pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Debug publish failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
