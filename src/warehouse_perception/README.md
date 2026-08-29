# warehouse_perception

`parcel_perception.py` — the sole perception node in the simulated pipeline:
ArUco marker detection + `solvePnP` for a real 6-DOF pose (position *and*
yaw), transformed to world frame via known camera extrinsics, looked up
against `warehouse_gazebo`'s `parcel_catalog.yaml` for shape/dims/target bin.

## Interface

- In: `/top_camera_color/image_raw` + `camera_info` (primary),
  `/front_camera_color/image_raw` (debug-only confirmation view).
- Out: `/parcel_detections` (`warehouse_interfaces/msg/ParcelDetection`),
  `/aruco/debug_image`, `/aruco/debug_front`.

## Known caveat

The exact camera topic names assumed here (`/top_camera_color/image_raw`
etc.) were never verified against a running Gazebo instance while writing
this — check with `ros2 topic list` after launch; see the node's own
docstring for why this was historically the actual point of failure in this
pipeline (three earlier, now-removed perception nodes each guessed a
different topic name).

## Real-hardware swap point

`warehouse_aruco_detection` is the separate, untouched real-hardware
package (real GP12 + Intel RealSense D455) — swapping to real hardware
means launching that package's nodes instead of this one, not modifying
this one.
