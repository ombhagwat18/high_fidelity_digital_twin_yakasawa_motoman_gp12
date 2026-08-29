# warehouse_pick_place

The pick-and-place state machine and motion executor.

## Contents

- `pick_place_logic.py` — subscribes `/parcel_detections`, routes each
  parcel to a bin purely from `parcel_catalog.yaml`'s `bin` field (nothing
  hardcoded to a specific shape), debounces per marker ID, gates every cycle
  on `/yrc1000/status`, and — after repeated consecutive grasp failures —
  triggers its own protective E-stop via `/yrc1000/estop` rather than
  retrying a broken grasp forever. Publishes KPIs on `/pick_place/stats`
  (`warehouse_interfaces/msg/PickPlaceStats`).
- `pick_place.py` — `PickPlaceExecutor`: MoveItPy + a position-only
  joint-space IK (see its "Why joint-space IK?" docstring for why), an
  approximate wrist-yaw alignment for orientation-aware grasping
  (`_wrist_j6_for_yaw` — explicitly flagged as an unverified approximation,
  tune `WRIST_YAW_SIGN` after checking in Gazebo), and real vacuum on/off/
  grip-confirm calls against `warehouse_gripper_control`.

## Recovering from a protective stop

1. `ros2 service call /yrc1000/alarm_reset std_srvs/srv/Trigger`
2. `ros2 service call /yrc1000/estop std_srvs/srv/SetBool "{data: false}"`
3. `ros2 service call /yrc1000/servo_on std_srvs/srv/SetBool "{data: true}"`
