# Verification Runbook

Every command needed to check that the GP12 digital twin actually works,
subsystem by subsystem. Run this top-to-bottom on a real ROS2 Humble +
Gazebo11 machine (WSL2 or native Ubuntu 22.04) after a fresh build — nothing
here has been executed by an AI in this repo's own history (no ROS/Gazebo
available in that environment), so treat this as the actual first real
test pass.

Each entry: **what it checks → command(s) → what you should see**.
If something doesn't match, the "if this fails" note points at the file
responsible.

---

## 0. Build

```bash
# Fresh clone — grabs the vendored Yaskawa submodule too
git clone --recurse-submodules https://github.com/ombhagwat18/high_fidelity_digital_twin_yakasawa_motoman_gp12.git
cd high_fidelity_digital_twin_yakasawa_motoman_gp12

colcon build --symlink-install
source install/setup.bash
```

**Expect:** all packages build with no errors, including the two C++ Gazebo
plugins (`warehouse_gripper_control`, `warehouse_gazebo`'s
`conveyor_belt_plugin`) and the marker-texture generation step (a message
like `[generate_parcel_marker_assets] generated 6 marker textures...` should
print during `warehouse_gazebo`'s configure step — if it instead prints a
warning about OpenCV not being available, install `python3-opencv`/
`opencv-contrib-python` and rebuild `warehouse_gazebo`, or the spawned
parcels will be plain grey boxes with no detectable marker).

```bash
# Rebuild just one package while iterating (much faster)
colcon build --packages-select warehouse_pick_place --symlink-install
```

---

## 1. Bring the whole system up

```bash
ros2 launch warehouse_bringup bringup.launch.py
```

**Expect:** Gazebo opens with the GP12 on its conveyor cell, RViz opens
alongside it (operator dashboard by default), and the terminal prints the
startup ladder from `bringup.launch.py`'s `LogInfo`, ending around t=25s
with a "Ready" line.

Launch variants:

```bash
# Bare MoveIt planning-scene RViz instead of the camera+TF operator dashboard
ros2 launch warehouse_bringup bringup.launch.py use_operator_view:=false

# No RViz at all (headless perception/control only)
ros2 launch warehouse_bringup bringup.launch.py use_rviz:=false

# Different world file
ros2 launch warehouse_bringup bringup.launch.py world:=warehouse_industrial.world
```

**If this fails:** check `src/warehouse_gazebo/launch/gazebo.launch.py`
(controller config wiring) and `src/warehouse_bringup/launch/bringup.launch.py`
(package/executable names) first — these were the two places most likely to
regress.

---

## 2. Foundation: Gazebo + controllers + MoveIt

```bash
# Controllers actually loaded and active
ros2 control list_controllers

# Joint states are live
ros2 topic hz /joint_states

# MoveIt2 move_group is up
ros2 node list | grep move_group

# TF tree is complete (no missing frames)
ros2 run tf2_tools view_frames
```

**Expect:** `joint_state_broadcaster` and `arm_controller` both show
`active`; `/joint_states` publishes at a steady rate; `move_group` appears
in the node list; the TF tree PDF shows an unbroken chain from `world` down
through the arm to `tool0`/`suction_cup_link` and to both cameras.

---

## 3. Cameras & ArUco perception

```bash
# Confirm the actual topic names Gazebo publishes (flagged as unverified
# in parcel_perception.py's docstring — check this first if perception
# never publishes anything)
ros2 topic list | grep camera

# Both camera streams are live
ros2 topic hz /top_camera_color/image_raw
ros2 topic hz /front_camera_color/image_raw

# Perception node is running and has intrinsics
ros2 node list | grep parcel_perception

# Detections are flowing (position, shape, bin, yaw)
ros2 topic echo /parcel_detections

# Visual debug overlays (open in rqt_image_view or RViz's Image display)
ros2 run rqt_image_view rqt_image_view /aruco/debug_image
ros2 run rqt_image_view rqt_image_view /aruco/debug_front
```

**Expect:** `/parcel_detections` messages show a real `marker_id`, a
`shape` (`box`/`cylinder`/`plate`), a `bin` (`bin_a`/`bin_b`), a `pose`
with sane world-frame coordinates (X roughly 0.5–1.1, Y roughly ±0.5), and
a `yaw` that visibly changes as different parcels rotate past.

**If this fails:** if the two `ros2 topic hz` calls above time out, the
camera topic names in `parcel_perception.py`'s `TOP_IMAGE_TOPIC`/
`FRONT_IMAGE_TOPIC` constants don't match what your installed `gazebo_ros`
version actually publishes — fix those two constants to match
`ros2 topic list`'s actual output.

---

## 4. Conveyor + mixed-shape parcel spawner

```bash
# Belt is actually moving (not just visually — check the joint velocity)
ros2 topic echo /conveyor/joint_states

# Retune belt speed live
ros2 topic pub /conveyor/velocity std_msgs/msg/Float64 "data: 0.2" --once

# Spawner is alive and producing parcels
ros2 node list | grep product_spawner
```

**Expect:** watching Gazebo, the belt surface visibly scrolls and parcels
(box / cylinder / plate, each with a visible ArUco square on top) spawn at
the upstream end and travel toward the robot. `/conveyor/joint_states`
shows a non-zero `velocity` on `belt_joint`.

**If this fails:** check `src/warehouse_gazebo/src/conveyor_belt_plugin.cpp`
(is it registered in `conveyor_belt/model.sdf`?) and confirm
`scripts/generate_parcel_marker_assets.py` actually ran during build (§0) —
without marker textures, parcels spawn as plain grey boxes.

---

## 5. Vacuum gripper (real physics attach/detach)

```bash
# Manually test attach/detach with the suction cup touching a parcel in Gazebo
ros2 service call /vacuum_gripper/on std_srvs/srv/Trigger
ros2 topic echo /vacuum_gripper/state --once   # should now show data: true
ros2 service call /vacuum_gripper/off std_srvs/srv/Trigger
ros2 topic echo /vacuum_gripper/state --once   # should now show data: false

# Contact sensor feed (used internally by the plugin)
ros2 topic echo /vacuum_gripper/contact_states
```

**Expect:** with the suction cup physically touching a parcel in Gazebo,
calling `/vacuum_gripper/on` makes the parcel **stick to and move with**
the gripper (not just visually near it) — this is a real physics joint, not
a rendering trick. `/off` drops it under gravity.

**If this fails:** confirm `libvacuum_gripper_plugin.so` loaded without
error in the Gazebo terminal output (look for
`[vacuum_gripper_plugin] ... plugin will not attach` errors) — see
`src/warehouse_gripper_control/src/vacuum_gripper_plugin.cpp`.

---

## 6. Full pick-and-place cycle

```bash
# Watch the state machine's own logs (this is where cycle steps print)
ros2 topic echo /pick_place/stats

# Confirm the executor's arm action interface is reachable
ros2 action list | grep follow_joint_trajectory
```

**Expect:** watching Gazebo end-to-end, the arm goes to a parcel, its wrist
visibly rotates to roughly match that parcel's marker orientation before
descending (see caveat below), the suction cup grips it, and it's placed in
the correct bin per `parcel_catalog.yaml`. `/pick_place/stats` updates after
each cycle with real `total`/`picked`/`failures`/`last_cycle_time_s`/
`throughput_per_hour` values (not a JSON string — this is a real
`PickPlaceStats` message).

**Known caveat:** the wrist-yaw alignment
(`pick_place.py`'s `_wrist_j6_for_yaw`) is a documented approximation, not a
derived result — if the gripper visibly grasps at the wrong angle, see that
function's docstring for how to tune `WRIST_YAW_SIGN`.

---

## 7. Simulated YRC1000 controller & safety behaviour

```bash
# Controller status
ros2 topic echo /yrc1000/status

# Manual E-stop (operator-triggered)
ros2 service call /yrc1000/estop std_srvs/srv/SetBool "{data: true}"
# ...confirm no new pick cycles start while this is set...
ros2 service call /yrc1000/estop std_srvs/srv/SetBool "{data: false}"
ros2 service call /yrc1000/servo_on std_srvs/srv/SetBool "{data: true}"

# Servo toggle
ros2 service call /yrc1000/servo_on std_srvs/srv/SetBool "{data: false}"

# Force a protective stop deliberately (see below), then recover:
ros2 service call /yrc1000/alarm_reset std_srvs/srv/Trigger
ros2 service call /yrc1000/estop std_srvs/srv/SetBool "{data: false}"
ros2 service call /yrc1000/servo_on std_srvs/srv/SetBool "{data: true}"
```

**To deliberately trigger the automatic protective stop** (3 consecutive
grasp failures by default): temporarily edit
`src/warehouse_pick_place/config/pick_place_params.yaml` so `reach_x_min`/
`reach_x_max` reject every real parcel position, rebuild, relaunch, and
watch `/yrc1000/status` — after 3 failed cycles it should show `estop: true`
with `alarm_code: 8020` (distinct from the manual E-stop's `8010`) and
`alarm_text` mentioning "PROTECTIVE STOP". `pick_place_logic` should refuse
to dispatch any further cycles until the full 3-step recovery above is run.
Revert the YAML edit afterward.

**Expect:** `/yrc1000/status` reflects every state change immediately;
`servo_on:=true` is refused while E-stop is engaged (service call returns
`success: false`); `alarm_reset` is refused while E-stop is still engaged.

---

## 8. Diagnostics / health monitoring

```bash
# Raw feed
ros2 topic echo /diagnostics

# Human-friendly dashboard
ros2 run rqt_robot_monitor rqt_robot_monitor
```

**Expect:** `rqt_robot_monitor` shows four rows — Controller, Perception,
PickPlace, Gripper — all green under normal operation. Engaging E-stop
(§7) turns Controller red immediately; letting perception go >30s with no
detections turns Perception yellow; hitting the protective-stop threshold
turns PickPlace red.

**If this fails to even start:** the `diagnostic_updater` Python API usage
in `yrc1000_controller.py`/`parcel_perception.py`/`pick_place_logic.py` was
never import-tested — see `docs/architecture/README.md`'s "Known leftover
issues" for what to check if you get an `ImportError`/`AttributeError`
around `diagnostic_updater` on node startup.

---

## 9. Configuration overrides

```bash
# Confirm a node actually picked up its YAML config
ros2 param get /pick_place_logic protective_stop_threshold
ros2 param get /parcel_perception marker_size_m
ros2 param get /yrc1000_controller job_name

# Change a value at runtime (doesn't persist across relaunch — edit the
# YAML file for that) and confirm the node reacts
ros2 param set /pick_place_logic debounce_s 1.0
```

Config files, if you want to tune something permanently:
- `src/warehouse_gazebo/config/parcel_catalog.yaml` — parcel shapes/bins
- `src/warehouse_perception/config/parcel_perception.yaml`
- `src/warehouse_pick_place/config/pick_place_params.yaml`
- `src/warehouse_controller/config/yrc1000_params.yaml`
- `src/warehouse_bringup/config/diagnostics.yaml`

---

## 10. Per-package build/lint tests

```bash
# Every package has a BUILD_TESTING/lint block now
colcon test --packages-select warehouse_pick_place
colcon test --packages-select warehouse_perception warehouse_controller warehouse_gazebo warehouse_gripper_control warehouse_robot_description
colcon test-result --verbose
```

**Expect:** copyright/flake8/pep257 lint results (real findings are fine
and expected on first run — this checks the *scaffolding* works, not that
the code is lint-clean).

---

## 11. Quick reference: everything in one block

For a fast smoke test after any change, run these in order:

```bash
colcon build --symlink-install && source install/setup.bash
ros2 launch warehouse_bringup bringup.launch.py &
sleep 30
ros2 topic hz /joint_states --window 5
ros2 topic hz /top_camera_color/image_raw --window 5
ros2 topic echo /parcel_detections --once
ros2 topic echo /yrc1000/status --once
ros2 topic echo /pick_place/stats --once
ros2 run rqt_robot_monitor rqt_robot_monitor
```

If all six checks return real data (not timeouts), the digital twin is
fully wired end-to-end.
