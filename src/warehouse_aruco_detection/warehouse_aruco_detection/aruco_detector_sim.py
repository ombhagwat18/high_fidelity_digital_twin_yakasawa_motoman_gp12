#!/usr/bin/env python3
"""
aruco_detector_sim.py
---------------------
ArUco detector for SIMULATION using correct Gazebo topic names.

Actual Gazebo topics (from ros2 topic list):
  /top_camera_color/image_raw       <- top camera colour
  /top_camera_color/camera_info     <- top camera intrinsics
  /front_camera_color/image_raw     <- front camera colour
  /front_camera_color/camera_info   <- front camera intrinsics

Camera geometry (gp12.xacro after fix):
  Top   : bracket xyz=0.75 0.0 2.86, offset z=0.15 -> at (0.75, 0.0, 3.01)
          pitch=90deg, looking straight down, covers full 3.2m belt
  Front : bracket xyz=1.45 0.0 0.80, pitch=54.1deg -> aimed at belt surface

Conveyor (warehouse_industrial.world):
  Centre x=0.75, y=0.0, belt surface z=0.12, box pick height z=0.17
"""

import json
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from warehouse_interfaces.msg import Detection2D

# Top camera position (directly above conveyor, looking straight down)
TOP_CAM_X  = 0.75
TOP_CAM_Y  = 0.0
TOP_CAM_Z  = 3.01   # bracket z=2.86 + offset z=0.15

# Conveyor pick height
CONVEYOR_Z = 0.17   # belt surface 0.12 + half box 0.05

ARUCO_DICTS = {
    'DICT_4X4_50':  cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_5X5_50':  cv2.aruco.DICT_5X5_50,
    'DICT_6X6_50':  cv2.aruco.DICT_6X6_50,
}


class ArucoDetectorSim(Node):

    def __init__(self):
        super().__init__('aruco_detector_sim')

        self.declare_parameter('marker_size_m', 0.08)
        self.declare_parameter('aruco_dict',    'DICT_4X4_50')
        self.declare_parameter('show_window',   False)
        self.declare_parameter('id_label_map',  '{"0":"good","1":"defective"}')

        self._marker_size = self.get_parameter('marker_size_m').value
        dict_name         = self.get_parameter('aruco_dict').value
        self._show_window = self.get_parameter('show_window').value
        try:
            self._id_label = {
                int(k): v for k, v in
                json.loads(self.get_parameter('id_label_map').value).items()
            }
        except Exception:
            self._id_label = {0: 'good', 1: 'defective'}

        aruco_dict_id        = ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
        self._aruco_dict     = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        self._aruco_params   = cv2.aruco.DetectorParameters()
        self._aruco_detector = cv2.aruco.ArucoDetector(
            self._aruco_dict, self._aruco_params)

        self._K_top   = None
        self._dist_top = None
        self._K_front  = None
        self._bridge   = CvBridge()

        # Top camera - primary detection camera
        self.create_subscription(
            CameraInfo, '/top_camera_color/top_camera_color_sensor/camera_info',
            self._top_info_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/top_camera_color/top_camera_color_sensor/image_raw',
            self._top_image_cb, qos_profile_sensor_data)

        # Front camera - secondary confirmation camera
        self.create_subscription(
            CameraInfo, '/front_camera_color/front_camera_color_sensor/camera_info',
            self._front_info_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/front_camera_color/front_camera_color_sensor/image_raw',
            self._front_image_cb, qos_profile_sensor_data)

        # Publishers - identical to real robot node
        self._pose_pub    = self.create_publisher(PoseStamped,  '/object_pose',       10)
        self._det2d_pub   = self.create_publisher(Detection2D,  '/detections_2d',     10)
        self._debug_top   = self.create_publisher(Image, '/aruco/debug_image',  10)
        self._debug_front = self.create_publisher(Image, '/aruco/debug_front',  10)

        self.get_logger().info(
            f'\n[ArucoDetectorSim] Started'
            f'\n  dict={dict_name}  marker={self._marker_size}m'
            f'\n  TOP   topic: /top_camera_color/image_raw'
            f'\n  FRONT topic: /front_camera_color/image_raw'
            f'\n  TOP cam at ({TOP_CAM_X}, {TOP_CAM_Y}, {TOP_CAM_Z})'
            f'\n  Pick height: {CONVEYOR_Z}m'
            f'\n  Waiting for camera_info...'
        )

    def _top_info_cb(self, msg: CameraInfo):
        if self._K_top is not None:
            return
        self._K_top    = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_top = np.array(msg.d, dtype=np.float64)
        self.get_logger().info(
            f'Top camera intrinsics received: '
            f'fx={self._K_top[0,0]:.1f} fy={self._K_top[1,1]:.1f} '
            f'cx={self._K_top[0,2]:.1f} cy={self._K_top[1,2]:.1f}'
        )

    def _front_info_cb(self, msg: CameraInfo):
        if self._K_front is not None:
            return
        self._K_front = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.get_logger().info(
            f'Front camera intrinsics received: '
            f'fx={self._K_front[0,0]:.1f} fy={self._K_front[1,1]:.1f}'
        )

    def _top_image_cb(self, msg: Image):
        """Primary detection: overhead view, back-projects to world XYZ."""
        if self._K_top is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Top image bridge: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._aruco_detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            half    = self._marker_size / 2.0
            obj_pts = np.array([
                [-half,  half, 0.], [ half,  half, 0.],
                [ half, -half, 0.], [-half, -half, 0.]], dtype=np.float64)

            depth = TOP_CAM_Z - CONVEYOR_Z   # 2.84 m

            for i, mc in enumerate(corners):
                marker_id = int(ids[i][0])
                label     = self._id_label.get(marker_id, f'id{marker_id}')
                img_pts   = mc[0].astype(np.float64)

                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, self._K_top, self._dist_top,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue

                cx_px = int(np.mean(img_pts[:, 0]))
                cy_px = int(np.mean(img_pts[:, 1]))

                # Back-project pixel to world using known conveyor height
                # Top cam looks straight down: cam +x -> world -y, cam +y -> world -x
                x_cam   = (cx_px - self._K_top[0, 2]) * depth / self._K_top[0, 0]
                y_cam   = (cy_px - self._K_top[1, 2]) * depth / self._K_top[1, 1]
                world_x = TOP_CAM_X - y_cam
                world_y = TOP_CAM_Y - x_cam
                world_z = CONVEYOR_Z

                pose = PoseStamped()
                pose.header.stamp    = msg.header.stamp
                pose.header.frame_id = f'world/{label}'
                pose.pose.position.x = world_x
                pose.pose.position.y = world_y
                pose.pose.position.z = world_z
                pose.pose.orientation.y = 0.7071068
                pose.pose.orientation.w = 0.7071068
                self._pose_pub.publish(pose)

                det            = Detection2D()
                det.label      = label
                det.x          = float(cx_px)
                det.y          = float(cy_px)
                det.confidence = 0.95
                self._det2d_pub.publish(det)

                self.get_logger().info(
                    f'[TOP ID {marker_id}] {label} -> '
                    f'world=({world_x:.3f}, {world_y:.3f}, {world_z:.3f})')

                cv2.drawFrameAxes(frame, self._K_top, self._dist_top,
                                  rvec, tvec, self._marker_size * 0.5)
                cv2.putText(frame, f'ID{marker_id}:{label}',
                            (cx_px - 40, cy_px - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        try:
            dbg        = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            dbg.header = msg.header
            self._debug_top.publish(dbg)
        except Exception:
            pass

        if self._show_window:
            cv2.imshow('TOP ArUco', frame)
            cv2.waitKey(1)

    def _front_image_cb(self, msg: Image):
        """Secondary: front camera annotates and publishes debug image only."""
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
            self.get_logger().debug(
                f'Front camera sees {len(ids)} marker(s)')

        try:
            dbg        = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            dbg.header = msg.header
            self._debug_front.publish(dbg)
        except Exception:
            pass

        if self._show_window:
            cv2.imshow('FRONT ArUco', frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
