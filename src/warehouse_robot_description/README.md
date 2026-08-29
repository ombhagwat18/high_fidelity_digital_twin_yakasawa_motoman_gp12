# warehouse_robot_description

URDF/xacro description of the Motoman GP12 arm, its vacuum gripper, the two
simulated RealSense D435 cameras, and a cosmetic YRC1000 controller cabinet.

## Contents

- `urdf/gp12.xacro` — top-level robot description; includes everything below
  plus the `ros2_control`/`gazebo_ros2_control` block.
- `urdf/gp12_macro.xacro` — the arm itself (links, joints, meshes).
- `urdf/vacuum_gripper.xacro` — suction cup + contact sensor + loads
  `warehouse_gripper_control`'s `VacuumGripperPlugin`.
- `urdf/realsense_d435.xacro` — reusable camera macro, instantiated twice in
  `gp12.xacro` as `top_camera`/`front_camera` (the name IS the runtime ROS
  topic namespace — see that file's comment before renaming either one).
- `urdf/yrc1000_cabinet.xacro` — visual-only controller cabinet + cabling
  (no collision geometry, deliberately, so it can never affect MoveIt
  planning).

## Real-hardware swap point

`gp12.xacro`'s `ros2_control_plugin` xacro arg currently defaults to
`gazebo_ros2_control/GazeboSystem`. Swapping to real GP12 hardware later
means pointing this at the real Motoman `ros2_control` hardware interface —
see `docs/architecture/README.md`'s swap-point table at the workspace root.
