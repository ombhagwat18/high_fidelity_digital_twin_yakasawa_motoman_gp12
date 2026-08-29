# warehouse_aruco_detection

ArUco marker detection for the **real** Motoman GP12 + Intel RealSense D455
— deliberately separate from, and untouched by, the simulated digital-twin
rebuild (`warehouse_perception`'s `parcel_perception.py` is the sim
equivalent). This is the package you switch to when real hardware is
connected.

## Contents

- `aruco_detector.py` — real-hardware detection node.
- `extrinsic_calibrator.py` — camera-to-world extrinsic calibration tool.
- `generate_markers.py` — prints ArUco markers to attach to real objects.
- `launch/aruco_real_robot.launch.py` — the real launch file for this
  package.

## Known issue (not fixed — out of scope for the sim rebuild)

`launch/aruco_sim.launch.py` does not actually contain a launch
description — it's a stray copy of an older `aruco_detector_sim.py` draft
with camera-geometry constants that don't match anything current. It isn't
wired into any launch graph and is safe to ignore or delete.
