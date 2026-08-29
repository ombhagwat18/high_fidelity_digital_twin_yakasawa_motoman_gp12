#!/usr/bin/env python3
"""
pick_place_logic.py  —  Conveyor-belt palletizing state machine (GP12 scale)
=========================================================================
World layout (warehouse_industrial.world — GP12 version):
  Robot base     : X=0.00  Y=0.00  (origin)
  Conveyor belt  : X=0.80  Y=0.00  belt surface Z=0.51
  BIN A          : X=0.00  Y=+0.90  Z=0.20
  BIN B          : X=0.00  Y=-0.90  Z=0.20

GP12 max reach ≈ 1.2 m — all targets comfortably inside the envelope.

Pipeline
--------
  ArUco marker on parcel  →  parcel_perception  →  /parcel_detections
                                                        │
                                                  PickPlaceLogic (this node)
                                                        │
                            entry.bin == 'bin_a' → BIN A  (Y=+0.90)
                            entry.bin == 'bin_b' → BIN B  (Y=-0.90)
  (bin routing is fully data-driven from warehouse_gazebo's
   parcel_catalog.yaml via the ParcelDetection.bin field — nothing here is
   hardcoded to a specific shape or a "good/defective" binary any more, so
   adding a new parcel type + bin only means editing the catalog.)

Before dispatching a cycle, this node also gates on the simulated YRC1000
controller's status (/yrc1000/status): no new cycle starts while the servo is
off, an alarm is active, or E-stop is engaged — mirroring how a real
controller would refuse motion. On a successful cycle, it reports completion
to the controller node (/yrc1000/cycle_complete) purely for status/telemetry.

Safety: after `protective_stop_threshold` consecutive cycle failures, this
node itself calls /yrc1000/protective_stop instead of silently retrying a
broken grasp forever — see warehouse_pick_place/README.md for the recovery
sequence. This is a real, enforced behaviour, not just a log message.

Tunable parameters (see config/pick_place_params.yaml): reach_x_min/max,
reach_y_min/max, debounce_s, min_confidence, protective_stop_threshold.

State machine
-------------
  IDLE  →  detection arrives (+ controller ready)  →  PICKING  →  IDLE
"""

import threading
import time

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

from warehouse_interfaces.msg import ParcelDetection, PickPlaceStats, YRC1000Status
from warehouse_pick_place.pick_place import PickPlaceExecutor


# ── GP12 Drop-bin positions (world frame) ──────────────────────────────────
# Robot rotates joint1 ±90° to face each bin. Which parcels go to which bin
# is entirely decided by parcel_catalog.yaml's `bin` field — these are just
# the two physical bin poses that data can point at.
BIN_RELEASE_HEIGHT = 0.10   # m above bin floor before release

BINS = {
    'bin_a': dict(x=0.00, y= 0.90, z=0.25, frame='world'),
    'bin_b': dict(x=0.00, y=-0.90, z=0.25, frame='world'),
}

# Delay before initialising MoveIt (give move_group + controllers time to start)
MOVEIT_INIT_DELAY = 2.0

CYCLE_COMPLETE_SRV     = '/yrc1000/cycle_complete'
PROTECTIVE_STOP_SRV    = '/yrc1000/protective_stop'
THROUGHPUT_WINDOW_N    = 10   # cycles averaged for the rolling throughput estimate


class PickPlaceLogic(Node):
    """Subscribes to parcel detections, routes each to the correct bin via
    PickPlaceExecutor, gated by the simulated YRC1000 controller's status."""

    def __init__(self):
        super().__init__('pick_place_logic')

        self.declare_parameter('reach_x_min', 0.50)
        self.declare_parameter('reach_x_max', 1.10)
        self.declare_parameter('reach_y_min', -0.50)
        self.declare_parameter('reach_y_max', 0.50)
        self.declare_parameter('debounce_s', 0.5)
        self.declare_parameter('min_confidence', 0.4)
        self.declare_parameter('protective_stop_threshold', 3)

        self._reach_x = (
            float(self.get_parameter('reach_x_min').value),
            float(self.get_parameter('reach_x_max').value),
        )
        self._reach_y = (
            float(self.get_parameter('reach_y_min').value),
            float(self.get_parameter('reach_y_max').value),
        )
        self._debounce_s      = float(self.get_parameter('debounce_s').value)
        self._min_confidence  = float(self.get_parameter('min_confidence').value)
        self._protective_stop_threshold = int(
            self.get_parameter('protective_stop_threshold').value)

        self._executor   = None
        self._exec_ready = False

        self._lock = threading.Lock()
        self._busy = False

        self._last_trigger: dict = {}
        self._pending: dict = {}   # marker_id -> ParcelDetection, queued pre-init

        self._stats = dict(total=0, picked=0, failures=0)
        self._consecutive_failures = 0
        self._last_bin = ''
        self._last_cycle_time_s = 0.0
        self._cycle_complete_times: list = []   # monotonic timestamps, for throughput

        # Controller gate — default to "not ready" until a status arrives.
        self._controller_ready  = False
        self._controller_reason = 'no /yrc1000/status received yet'

        self.create_subscription(
            ParcelDetection, '/parcel_detections', self._detection_cb, 10)
        self.create_subscription(
            YRC1000Status, '/yrc1000/status', self._controller_status_cb, 10)

        self._stats_pub = self.create_publisher(PickPlaceStats, '/pick_place/stats', 10)
        self._cycle_complete_cli  = self.create_client(Trigger, CYCLE_COMPLETE_SRV)
        self._protective_stop_cli = self.create_client(Trigger, PROTECTIVE_STOP_SRV)

        self._diag_updater = Updater(self)
        self._diag_updater.setHardwareID('pick_place_logic')
        self._diag_updater.add('PickPlace', self._diagnostics_cb)
        self.create_timer(1.0, self._diag_updater.force_update)

        self._init_timer = self.create_timer(MOVEIT_INIT_DELAY, self._init_executor)

        self.get_logger().info(
            'PickPlaceLogic (GP12) started.\n'
            f'  Bins: {list(BINS.keys())}\n'
            f'  Reach X: {self._reach_x}  Y: {self._reach_y}\n'
            f'  Protective-stop threshold: {self._protective_stop_threshold} consecutive failures\n'
            f'  MoveIt init in {MOVEIT_INIT_DELAY} s ...'
        )

    # ── MoveIt deferred init ────────────────────────────────────────────────

    def _init_executor(self):
        self._init_timer.cancel()
        self.get_logger().info('Initialising MoveItPy + action/service clients ...')
        try:
            self._executor   = PickPlaceExecutor(self)
            self._exec_ready = True
            self.get_logger().info('MoveItPy ready — pick cycles enabled.')
            self._flush_pending()
        except Exception as e:
            self.get_logger().error(f'PickPlaceExecutor init failed: {e}')

    def _flush_pending(self):
        with self._lock:
            pending = dict(self._pending)
            self._pending.clear()
        for marker_id, msg in pending.items():
            self.get_logger().info(f'Flushing queued marker {marker_id}.')
            self._dispatch(msg)

    # ── Controller status gate ──────────────────────────────────────────────

    def _controller_status_cb(self, msg: YRC1000Status):
        if msg.estop:
            self._controller_ready, self._controller_reason = False, 'E-STOP engaged'
        elif not msg.servo_on:
            self._controller_ready, self._controller_reason = False, 'servo OFF'
        elif msg.alarm_active:
            self._controller_ready, self._controller_reason = (
                False, f'alarm {msg.alarm_code}: {msg.alarm_text}')
        else:
            self._controller_ready, self._controller_reason = True, 'ready'

    # ── Detection callback ───────────────────────────────────────────────────

    def _detection_cb(self, msg: ParcelDetection):
        if msg.confidence < self._min_confidence:
            return

        px, py = msg.pose.position.x, msg.pose.position.y
        if not (self._reach_x[0] <= px <= self._reach_x[1] and
                self._reach_y[0] <= py <= self._reach_y[1]):
            self.get_logger().debug(
                f'Out of reach — skip marker {msg.marker_id} @ ({px:.3f},{py:.3f})'
            )
            return

        if msg.bin not in BINS:
            self.get_logger().error(
                f'Unknown bin "{msg.bin}" for marker {msg.marker_id} — dropping.')
            return

        if not self._exec_ready:
            with self._lock:
                self._pending[msg.marker_id] = msg
            return

        self._dispatch(msg)

    def _dispatch(self, msg: ParcelDetection):
        if not self._controller_ready:
            self.get_logger().warn(
                f'Controller not ready ({self._controller_reason}) — '
                f'dropping marker {msg.marker_id}.')
            return

        now = time.monotonic()
        key = msg.marker_id
        with self._lock:
            if now - self._last_trigger.get(key, 0.0) < self._debounce_s:
                return
            if self._busy:
                self.get_logger().warn(
                    f'Busy — dropping marker {msg.marker_id} '
                    f'({msg.shape}) @ ({msg.pose.position.x:.2f},{msg.pose.position.y:.2f})'
                )
                return
            self._last_trigger[key] = now
            self._busy = True

        thread = threading.Thread(target=self._run_cycle, args=(msg,), daemon=True)
        thread.start()

    # ── Pick cycle ────────────────────────────────────────────────────────────

    def _run_cycle(self, msg: ParcelDetection):
        cycle_start = time.monotonic()
        try:
            place_pose = self._make_bin_pose(msg.bin)

            self.get_logger().info(
                f'┌── CYCLE START  marker={msg.marker_id}  shape={msg.shape}  '
                f'bin={msg.bin}  yaw={msg.yaw:.2f}rad'
            )

            success = self._executor.pick_and_place(
                msg.pose, place_pose, yaw=msg.yaw, shape=msg.shape)

            self._last_cycle_time_s = time.monotonic() - cycle_start
            self._last_bin = msg.bin
            self._stats['total'] += 1

            if success:
                self._stats['picked'] += 1
                self._consecutive_failures = 0
                self._cycle_complete_times.append(time.monotonic())
                self._cycle_complete_times = self._cycle_complete_times[-THROUGHPUT_WINDOW_N:]
                self.get_logger().info(
                    f'└── CYCLE OK  picked={self._stats["picked"]}  '
                    f'fail={self._stats["failures"]}  '
                    f'cycle_time={self._last_cycle_time_s:.1f}s'
                )
                self._report_cycle_complete()
            else:
                self._stats['failures'] += 1
                self._consecutive_failures += 1
                self.get_logger().error(
                    f'└── CYCLE FAILED  total_failures={self._stats["failures"]}  '
                    f'consecutive={self._consecutive_failures}'
                )
                self._maybe_trigger_protective_stop()

            self._publish_stats()

        except Exception as e:
            self._stats['failures'] += 1
            self._consecutive_failures += 1
            self.get_logger().error(f'Exception in pick cycle: {e}')
            self._maybe_trigger_protective_stop()
        finally:
            with self._lock:
                self._busy = False

    def _maybe_trigger_protective_stop(self):
        if self._consecutive_failures < self._protective_stop_threshold:
            return
        self.get_logger().error(
            f'{self._consecutive_failures} consecutive failures >= threshold '
            f'({self._protective_stop_threshold}) — engaging protective stop.'
        )
        if self._protective_stop_cli.service_is_ready():
            self._protective_stop_cli.call_async(Trigger.Request())
        else:
            self.get_logger().error(
                'yrc1000_controller not reachable — cannot engage protective stop!'
            )

    def _report_cycle_complete(self):
        if not self._cycle_complete_cli.service_is_ready():
            return   # controller node not up — non-fatal, just skip telemetry
        self._cycle_complete_cli.call_async(Trigger.Request())

    def _publish_stats(self):
        msg = PickPlaceStats()
        msg.total                = self._stats['total']
        msg.picked                = self._stats['picked']
        msg.failures              = self._stats['failures']
        msg.consecutive_failures  = self._consecutive_failures
        msg.last_cycle_time_s     = float(self._last_cycle_time_s)
        msg.throughput_per_hour   = self._throughput_per_hour()
        msg.last_bin              = self._last_bin
        self._stats_pub.publish(msg)

    def _throughput_per_hour(self) -> float:
        times = self._cycle_complete_times
        if len(times) < 2:
            return 0.0
        span_s = times[-1] - times[0]
        if span_s <= 0.0:
            return 0.0
        cycles = len(times) - 1
        return float(cycles / span_s * 3600.0)

    def _diagnostics_cb(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        if self._consecutive_failures >= self._protective_stop_threshold:
            stat.summary(DiagnosticStatus.ERROR, 'protective stop threshold reached')
        elif self._consecutive_failures > 0:
            stat.summary(DiagnosticStatus.WARN,
                         f'{self._consecutive_failures} consecutive failure(s)')
        elif self._busy:
            stat.summary(DiagnosticStatus.OK, 'PICKING')
        else:
            stat.summary(DiagnosticStatus.OK, 'IDLE')
        stat.add('total', str(self._stats['total']))
        stat.add('picked', str(self._stats['picked']))
        stat.add('failures', str(self._stats['failures']))
        stat.add('consecutive_failures', str(self._consecutive_failures))
        stat.add('controller_ready', str(self._controller_ready))
        return stat

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _make_bin_pose(self, bin_name: str):
        cfg = BINS[bin_name]
        pose = Pose()
        pose.position.x = cfg['x']
        pose.position.y = cfg['y']
        pose.position.z = cfg['z'] + BIN_RELEASE_HEIGHT
        pose.orientation.y = 0.7071068
        pose.orientation.w = 0.7071068
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceLogic()
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
