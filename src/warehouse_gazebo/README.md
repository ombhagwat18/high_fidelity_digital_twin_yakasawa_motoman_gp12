# warehouse_gazebo

Gazebo (classic, gazebo11) simulation world, conveyor, and parcel spawner for
the GP12 digital twin.

## Contents

- `worlds/warehouse_industrial.world` — the world actually used by
  `bringup.launch.py` (ground plane, conveyor include, pick table, two bins).
- `models/conveyor_belt/` — conveyor model; belt motion is driven by this
  package's own `ConveyorBeltPlugin` (`src/conveyor_belt_plugin.cpp`), not by
  Gazebo's stock plugins — retune/command speed live via
  `ros2 topic pub /conveyor/velocity std_msgs/msg/Float64 "data: 0.2"`.
- `config/parcel_catalog.yaml` — single source of truth: ArUco marker ID →
  parcel shape/dimensions/target bin. Read by `product_spawner.py` (what to
  spawn) and by `warehouse_perception`'s `parcel_perception.py` (what a
  detected ID means) — edit this file to add a new parcel type.
- `scripts/generate_parcel_marker_assets.py` — runs automatically from
  `CMakeLists.txt` at configure time; generates the ArUco marker PNG
  textures + Ogre material scripts parcels wear (needs OpenCV at build time;
  warns and no-ops if unavailable rather than failing the build).
- `warehouse_simulation/product_spawner.py` — spawns randomized box/cylinder/
  plate parcels onto the belt's upstream end.
- `launch/gazebo.launch.py` — Gazebo + robot spawn + controller spawners;
  this is what actually sets the `controllers_file` xacro mapping that
  `gazebo_ros2_control` needs (see its own comments — this was previously a
  silent-failure bug where controllers never loaded at all).
