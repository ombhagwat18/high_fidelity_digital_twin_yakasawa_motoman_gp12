# warehouse_interfaces

Custom messages shared across the digital twin.

- `Detection2D` — legacy 2D detection message, still used by the
  real-hardware ArUco path in `warehouse_aruco_detection`. Not used by the
  simulated pipeline any more (superseded by `ParcelDetection`).
- `ParcelDetection` — marker ID, shape, dims, target bin, full world-frame
  pose, and yaw. The perception↔pick-place interface for the sim pipeline.
- `YRC1000Status` — simulated controller status (servo/mode/E-stop/alarm/
  job/cycle telemetry), published by `warehouse_controller`.
- `PickPlaceStats` — throughput/KPI reporting from `warehouse_pick_place`.
