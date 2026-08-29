# warehouse_gripper_control

Gazebo model plugin (`VacuumGripperPlugin`, C++) that turns the passive
suction-cup contact sensor defined in `vacuum_gripper.xacro` into a real,
physically-attaching vacuum gripper — the same technique used by the
well-known `gazebo_ros_link_attacher`/`linkattacher_plugin` community
packages, scoped to just this gripper.

## Interface

- `/vacuum_gripper/on`, `/vacuum_gripper/off` (`std_srvs/srv/Trigger`) —
  physically attach/detach whatever the suction cup's contact sensor
  currently reports touching.
- `/vacuum_gripper/state` (`std_msgs/msg/Bool`) — simulated vacuum-switch
  grip confirmation, published at 10 Hz.
- Consumes `/vacuum_gripper/contact_states`
  (`gazebo_msgs/msg/ContactsState`, from the `libgazebo_ros_bumper.so`
  sensor plugin already attached to `suction_cup_link`).

## Real-hardware swap point

A real cell would replace this with an actual vacuum solenoid + vacuum
switch I/O driver exposing the same three names — `pick_place.py` calls
these by name/topic only and doesn't care which side of the swap it's
talking to.
