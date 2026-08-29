# warehouse_controller

Simulated Yaskawa YRC1000 controller. There is no real YRC1000 in this
digital twin — `yrc1000_controller.py` stands in for it, with field names
and gating behaviour modeled on the real MotoROS2 ROS2 driver
(https://github.com/Yaskawa-Global/motoros2).

## Interface

- `/yrc1000/status` (`warehouse_interfaces/msg/YRC1000Status`, 10 Hz) —
  servo state, mode, E-stop, alarm, job/cycle telemetry.
- `/yrc1000/servo_on`, `/yrc1000/estop` (`std_srvs/srv/SetBool`).
- `/yrc1000/alarm_reset` (`std_srvs/srv/Trigger`) — refuses while E-stop is
  engaged, matching real controller behaviour.
- `/yrc1000/cycle_complete` (`std_srvs/srv/Trigger`) — telemetry only,
  called by `warehouse_pick_place` after each successful cycle.

## Safety behaviour

`warehouse_pick_place`'s `pick_place_logic.py` gates every new pick cycle on
this node's status (won't dispatch while servo is off / E-stop engaged /
an alarm is active), and will *itself* trigger a protective E-stop here
after repeated consecutive grasp failures — see that package's README for
the threshold and recovery sequence.

## Real-hardware swap point

Swap this node for a real `motoros2` bridge; keep the topic/service names
identical so nothing downstream (`pick_place_logic.py`) needs to change.
