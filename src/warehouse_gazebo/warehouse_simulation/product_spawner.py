#!/usr/bin/env python3
"""
product_spawner.py — spawns randomized-shape, ArUco-tagged parcels onto the
moving conveyor belt (Amazon-conveyor-style mixed-shape stream).
==============================================================================
Every parcel is one of the shapes/sizes defined in
warehouse_gazebo/config/parcel_catalog.yaml (box, cylinder/"round", or a thin
flat plate), each carrying exactly one ArUco marker decal on its top face
(rendered via model://parcel_markers — see generate_parcel_marker_assets.py).
The marker ID is what parcel_perception.py actually detects; shape/size/target
bin are looked up from the SAME catalog file, so spawner and perception can
never disagree about what a given ID means.

World geometry (must match warehouse_industrial.world's conveyor_belt include
at pose 0.8 0 0, and conveyor_belt/model.sdf's belt_link local pose 0 0 0.5):
  Belt top surface Z  : 0.51 m  (model 0.8,0,0 + belt_link z=0.5 + 0.01 half-thickness)
  Belt travel direction: +X (see conveyor_belt_plugin.cpp), so parcels spawn at
  the upstream end (low X) and are carried by belt friction toward the robot's
  pick zone around X=0.8-0.9 m.

Parcels are deleted a fixed time after spawning (long enough to be picked)
rather than in a fixed batch cycle, since with real belt motion a parcel's
position downstream depends on time-on-belt, not spawn order.

Non-blocking spawn/delete via future.add_done_callback() — see the original
FIX note this replaces: calling rclpy.spin_until_future_complete() from
inside a timer callback that runs under the SAME executor as rclpy.spin(node)
is a reentrant-spin deadlock (the executor can't process the service response
while it's already busy running the callback waiting on it). That bug is not
reintroduced here.
"""

import math
import os
import random

import yaml

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory

# ── Belt geometry (world frame) ───────────────────────────────────────────────
SPAWN_X        = 0.05    # upstream end of the belt
SPAWN_Y_RANGE  = (-0.18, 0.18)
BELT_TOP_Z     = 0.51
SPAWN_CLEARANCE = 0.02   # extra gap above belt so parcels drop in cleanly

SPAWN_INTERVAL_S  = 8.0
PARCEL_LIFETIME_S = 30.0   # long enough to travel the pick zone and be removed


def _load_catalog() -> list:
    share = get_package_share_directory('warehouse_gazebo')
    path = os.path.join(share, 'config', 'parcel_catalog.yaml')
    with open(path) as f:
        data = yaml.safe_load(f)
    return data['parcels']


def _box_inertia(mass: float, l: float, w: float, h: float):
    ixx = mass * (w * w + h * h) / 12.0
    iyy = mass * (l * l + h * h) / 12.0
    izz = mass * (l * l + w * w) / 12.0
    return ixx, iyy, izz


def _shape_sdf(name: str, shape: str, dims_m: list, marker_id: int) -> str:
    """Build a parcel model: a plain grey body (box or cylinder) plus a thin
    ArUco-textured decal visual on the top face — the standard Gazebo
    "sticker on a primitive" technique, since a primitive's own material
    can't be mapped to a single face."""
    length, width, height = dims_m
    # Light cardboard/plastic-ish density, not a real-mass simulation concern.
    mass = max(0.05, 0.9 * length * width * height * 400.0)
    ixx, iyy, izz = _box_inertia(mass, length, width, height)

    if shape == 'cylinder':
        radius = length / 2.0
        body_geom  = f'<cylinder><radius>{radius:.4f}</radius><length>{height:.4f}</length></cylinder>'
        decal_geom = f'<cylinder><radius>{radius * 0.85:.4f}</radius><length>0.002</length></cylinder>'
    else:  # 'box' or 'plate' — both are just boxes of different proportions
        body_geom  = f'<box><size>{length:.4f} {width:.4f} {height:.4f}</size></box>'
        decal_geom = (
            f'<box><size>{length * 0.85:.4f} {width * 0.85:.4f} 0.002</size></box>'
        )

    decal_z = height / 2.0 + 0.0011

    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{mass:.4f}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="body_visual">
        <geometry>{body_geom}</geometry>
        <material>
          <ambient>0.55 0.55 0.55 1</ambient>
          <diffuse>0.62 0.62 0.62 1</diffuse>
        </material>
      </visual>
      <collision name="body_collision">
        <geometry>{body_geom}</geometry>
        <surface>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="marker_decal">
        <pose>0 0 {decal_z:.5f} 0 0 0</pose>
        <geometry>{decal_geom}</geometry>
        <material>
          <script>
            <uri>model://parcel_markers</uri>
            <name>ParcelMarker/marker_{marker_id}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class ProductSpawner(Node):

    def __init__(self):
        super().__init__('product_spawner')

        self._catalog = _load_catalog()
        self.get_logger().info(
            f'Loaded {len(self._catalog)} parcel types from parcel_catalog.yaml.'
        )

        self._spawn_client  = self.create_client(SpawnEntity,  '/spawn_entity')
        self._delete_client = self.create_client(DeleteEntity, '/delete_entity')

        self.get_logger().info('ProductSpawner: waiting for Gazebo services ...')
        self._spawn_client.wait_for_service(timeout_sec=30.0)
        self._delete_client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('ProductSpawner ready.')

        self._seq = 0

        # Spawn the first parcel shortly after start, then on a fixed interval.
        self._first_timer = self.create_timer(2.0, self._first_spawn)
        self.create_timer(SPAWN_INTERVAL_S, self._spawn_one)

    def _first_spawn(self):
        self._first_timer.cancel()
        self._spawn_one()

    def _spawn_one(self):
        self._seq += 1
        entry  = random.choice(self._catalog)
        name   = f"parcel_{entry['shape']}_{entry['marker_id']}_{self._seq}"
        dims   = entry['dims_m']
        y      = random.uniform(*SPAWN_Y_RANGE)
        z      = BELT_TOP_Z + dims[2] / 2.0 + SPAWN_CLEARANCE
        yaw    = random.uniform(-math.pi, math.pi)

        pose = Pose()
        pose.position.x = SPAWN_X
        pose.position.y = y
        pose.position.z = z
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        req = SpawnEntity.Request()
        req.name            = name
        req.xml             = _shape_sdf(name, entry['shape'], dims, entry['marker_id'])
        req.robot_namespace = ''
        req.initial_pose    = pose
        req.reference_frame = 'world'

        future = self._spawn_client.call_async(req)
        future.add_done_callback(lambda fut, n=name: self._on_spawn_done(fut, n))

        # One-shot lifetime timer: cancels itself on first fire, then deletes.
        timer_box = {}

        def _on_lifetime_expired(n=name, box=timer_box):
            t = box.get('timer')
            if t is not None:
                t.cancel()
            self._request_delete(n)

        timer_box['timer'] = self.create_timer(PARCEL_LIFETIME_S, _on_lifetime_expired)

    def _request_delete(self, name: str):
        req = DeleteEntity.Request()
        req.name = name
        future = self._delete_client.call_async(req)
        future.add_done_callback(lambda fut, n=name: self._on_delete_done(fut, n))

    def _on_spawn_done(self, future, name):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Spawn call for {name} raised an exception: {exc}')
            return
        if result and result.success:
            self.get_logger().info(f'Spawned {name}')
        else:
            msg = result.status_message if result else 'unknown error'
            self.get_logger().error(f'Failed to spawn {name}: {msg}')

    def _on_delete_done(self, future, name):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Delete call for {name} raised an exception: {exc}')
            return
        if result and result.success:
            self.get_logger().info(f'Removed {name} (end of belt)')
        else:
            self.get_logger().warn(
                f'Could not delete {name} (likely already picked/gone) — normal.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ProductSpawner()
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
