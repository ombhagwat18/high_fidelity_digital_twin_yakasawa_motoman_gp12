# warehouse_bringup

Top-level launch orchestration for the whole digital twin.

- `launch/bringup.launch.py` — the one launch file to run; sequences
  Gazebo → simulated YRC1000 controller → MoveIt2 → RViz → parcel
  perception → pick-and-place logic → product spawner on a `TimerAction`
  ladder (see its own docstring, and the Mermaid Gantt diagram in
  `docs/architecture/README.md` at the workspace root).
- `config/diagnostics.yaml` — `diagnostic_aggregator` analyzer groups
  (Controller/Perception/PickPlace/Gripper) feeding `/diagnostics`; view
  with `ros2 run rqt_robot_monitor rqt_robot_monitor`.
- `config/operator.rviz` — operator-facing RViz layout: MoveIt planning
  scene + both camera feeds + the ArUco debug overlay + TF, alongside the
  bare planning-scene-only `warehouse_moveit_config/config/moveit.rviz`.
- `scripts/hold_position.py` — standalone debug script, not part of normal
  bringup.

No CI workflow exists because this workspace is not (yet) a git repository;
once it is, `colcon test` is fully scaffolded across every package and is
what a CI job should run.
