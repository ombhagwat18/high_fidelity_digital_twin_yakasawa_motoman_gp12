#!/usr/bin/env python3
"""
yrc1000_controller.py — simulated Yaskawa YRC1000 controller.
==============================================================================
There is no real YRC1000 in this digital twin, so this node stands in for it:
it tracks servo/E-stop/alarm/job state and publishes it exactly the way the
real controller's ROS2 driver (MotoROS2) would — see
https://github.com/Yaskawa-Global/motoros2, which exposes robot status,
alarms, and servo/I-O state over ROS2 topics/services on top of standard
FollowJointTrajectory motion. pick_place_logic.py gates every pick cycle on
this node's /yrc1000/status the same way it would gate on the real
controller's status later — swapping this node for a real motoros2 bridge
should not require changing pick_place_logic.py at all.

Topics / services
------------------
  Pub  /yrc1000/status          warehouse_interfaces/YRC1000Status  (10 Hz)
  Srv  /yrc1000/servo_on        std_srvs/SetBool   (true=servo on, false=off)
  Srv  /yrc1000/estop           std_srvs/SetBool   (true=engage, false=release
                                                     — operator-triggered)
  Srv  /yrc1000/protective_stop std_srvs/Trigger   (system-triggered halt,
                                                     e.g. repeated grasp
                                                     failures — see
                                                     warehouse_pick_place)
  Srv  /yrc1000/alarm_reset     std_srvs/Trigger   (fails while E-stop/
                                                     protective-stop active)
  Srv  /yrc1000/cycle_complete  std_srvs/Trigger   (telemetry only)

Also publishes /diagnostics (diagnostic_updater) so `rqt_robot_monitor` and
warehouse_bringup's diagnostic_aggregator can surface controller health.

Manual testing
--------------
  ros2 topic echo /yrc1000/status
  ros2 service call /yrc1000/estop std_srvs/srv/SetBool "{data: true}"
  ros2 service call /yrc1000/protective_stop std_srvs/srv/Trigger
  ros2 service call /yrc1000/alarm_reset std_srvs/srv/Trigger
"""

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from warehouse_interfaces.msg import YRC1000Status

STATUS_RATE_HZ = 10.0

ALARM_CODE_ESTOP           = 8010
ALARM_TEXT_ESTOP           = 'EX. EMERGENCY STOP (simulated)'
ALARM_CODE_PROTECTIVE_STOP = 8020
ALARM_TEXT_PROTECTIVE_STOP = (
    'PROTECTIVE STOP — repeated grasp failures (self-triggered by pick_place_logic)'
)


class YRC1000Controller(Node):

    def __init__(self):
        super().__init__('yrc1000_controller')

        self.declare_parameter('job_name', 'PALLETIZE_DEMO')
        self._job_name = str(self.get_parameter('job_name').value)

        # Digital twin default: servo pre-enabled so bringup doesn't require
        # a manual servo-on step every launch. A real cell would start
        # de-energised; toggle via /yrc1000/servo_on if you want that here.
        self._servo_on     = True
        self._estop        = False
        self._alarm_active = False
        self._alarm_code   = 0
        self._alarm_text   = ''
        self._job_line     = 0
        self._cycle_count  = 0

        self._status_pub = self.create_publisher(YRC1000Status, '/yrc1000/status', 10)

        self.create_service(SetBool, '/yrc1000/servo_on', self._on_servo_on)
        self.create_service(SetBool, '/yrc1000/estop', self._on_estop)
        self.create_service(Trigger, '/yrc1000/protective_stop', self._on_protective_stop)
        self.create_service(Trigger, '/yrc1000/alarm_reset', self._on_alarm_reset)
        self.create_service(Trigger, '/yrc1000/cycle_complete', self._on_cycle_complete)

        self._diag_updater = Updater(self)
        self._diag_updater.setHardwareID('yrc1000-simulated')
        self._diag_updater.add('Controller', self._diagnostics_cb)

        self.create_timer(1.0 / STATUS_RATE_HZ, self._tick)

        self.get_logger().info(
            'YRC1000Controller (simulated) started. servo_on=True, estop=False.'
        )

    # ── Services ─────────────────────────────────────────────────────────────

    def _on_servo_on(self, request, response):
        if request.data and self._estop:
            response.success = False
            response.message = 'cannot servo ON while E-stop/protective-stop is engaged'
            return response
        self._servo_on = bool(request.data)
        response.success = True
        response.message = f'servo_on={self._servo_on}'
        self.get_logger().info(response.message)
        return response

    def _on_estop(self, request, response):
        self._estop = bool(request.data)
        if self._estop:
            self._servo_on     = False
            self._alarm_active = True
            self._alarm_code   = ALARM_CODE_ESTOP
            self._alarm_text   = ALARM_TEXT_ESTOP
            self.get_logger().warn('E-STOP ENGAGED — servo forced OFF.')
        else:
            self.get_logger().info('E-stop released (servo still OFF until re-enabled).')
        response.success = True
        response.message = f'estop={self._estop}'
        return response

    def _on_protective_stop(self, request, response):
        self._estop        = True
        self._servo_on      = False
        self._alarm_active  = True
        self._alarm_code    = ALARM_CODE_PROTECTIVE_STOP
        self._alarm_text    = ALARM_TEXT_PROTECTIVE_STOP
        self.get_logger().error(f'PROTECTIVE STOP ENGAGED — {ALARM_TEXT_PROTECTIVE_STOP}')
        response.success = True
        response.message = 'protective stop engaged'
        return response

    def _on_alarm_reset(self, request, response):
        if self._estop:
            response.success = False
            response.message = 'cannot reset alarm while E-stop/protective-stop is engaged'
            return response
        self._alarm_active = False
        self._alarm_code   = 0
        self._alarm_text   = ''
        response.success = True
        response.message = 'alarm cleared'
        self.get_logger().info(response.message)
        return response

    def _on_cycle_complete(self, request, response):
        self._cycle_count += 1
        self._job_line += 1
        response.success = True
        response.message = f'cycle_count={self._cycle_count}'
        return response

    # ── Status + diagnostics ─────────────────────────────────────────────────

    def _tick(self):
        self._publish_status()
        self._diag_updater.force_update()

    def _publish_status(self):
        msg = YRC1000Status()
        msg.servo_on     = self._servo_on
        msg.mode         = YRC1000Status.MODE_PLAY
        msg.estop        = self._estop
        msg.alarm_active = self._alarm_active
        msg.alarm_code   = self._alarm_code
        msg.alarm_text   = self._alarm_text
        msg.job_name     = self._job_name
        msg.job_line     = self._job_line
        msg.cycle_count  = self._cycle_count
        self._status_pub.publish(msg)

    def _diagnostics_cb(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        if self._estop:
            level = DiagnosticStatus.ERROR
            msg   = f'HALTED: {self._alarm_text or "E-stop engaged"}'
        elif self._alarm_active:
            level = DiagnosticStatus.WARN
            msg   = f'alarm {self._alarm_code}: {self._alarm_text}'
        elif not self._servo_on:
            level = DiagnosticStatus.WARN
            msg   = 'servo OFF'
        else:
            level = DiagnosticStatus.OK
            msg   = 'servo on, no alarms'
        stat.summary(level, msg)
        stat.add('servo_on', str(self._servo_on))
        stat.add('estop', str(self._estop))
        stat.add('alarm_code', str(self._alarm_code))
        stat.add('cycle_count', str(self._cycle_count))
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = YRC1000Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
