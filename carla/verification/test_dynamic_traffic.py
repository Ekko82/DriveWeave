"""Offline contract/geometry tests. Imports real CARLA bindings, never a Client."""
from __future__ import annotations

from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import carla
from runtime_config import RuntimeConfig
from dynamic_traffic import (Footprint, TrafficConfig, _actor_footprint,
    build_spawn_candidates, horizontal_body_clearance, setup_dynamic_traffic,
    spawn_rejection)

BASE = Path(__file__).resolve().parents[1]
MAP_FILE = BASE / 'map-inventory/Town04/map.xodr'
if not MAP_FILE.is_file():
    MAP_FILE = (RuntimeConfig.resolve(require_carla=True).carla_root /
                'CarlaUE4/Content/Carla/Maps/OpenDrive/Town04.xodr')


def copied_pose(pose):
    return carla.Transform(carla.Location(x=pose.location.x, y=pose.location.y,
        z=pose.location.z), carla.Rotation(pitch=pose.rotation.pitch,
        yaw=pose.rotation.yaw, roll=pose.rotation.roll))


class Attribute:
    def __init__(self, value, recommended_values=()):
        self.value, self.recommended_values = value, recommended_values

    def __int__(self):
        return int(self.value)


class Blueprint:
    def __init__(self, name):
        self.id = name
        self.attributes = {'number_of_wheels': Attribute(4),
            'color': Attribute('0,0,0', ['0,0,0', '50,70,90']), 'role_name': Attribute('')}

    def has_attribute(self, key):
        return key in self.attributes

    def get_attribute(self, key):
        return self.attributes[key]

    def set_attribute(self, key, value):
        self.attributes[key].value = value


class Library:
    def __init__(self):
        self.blueprints = {name: Blueprint(name) for name in TrafficConfig().blueprints}

    def filter(self, pattern):
        return list(self.blueprints.values())

    def find(self, name):
        return self.blueprints[name]


class Actor:
    def __init__(self, actor_id, pose, blueprint='vehicle.tesla.model3', role='hero', velocity=None):
        self.id, self.pose, self.type_id = actor_id, copied_pose(pose), blueprint
        self.attributes = {'role_name': role}
        self.bounding_box = carla.BoundingBox(carla.Location(x=.03, z=.75),
                                              carla.Vector3D(2.4, 1.05, .75))
        self.velocity = velocity or carla.Vector3D()
        self.is_alive = True
        self.calls = []

    def get_transform(self):
        return copied_pose(self.pose)

    def get_location(self):
        return self.get_transform().location

    def get_velocity(self):
        return self.velocity

    def get_speed_limit(self):
        return 70.

    def set_autopilot(self, enabled, port):
        self.calls.append(('set_autopilot', enabled, port))

    def destroy(self):
        self.calls.append(('destroy',))
        self.is_alive = False


class ActorList(list):
    def filter(self, pattern):
        return ActorList(a for a in self if a.is_alive and a.type_id.startswith('vehicle.'))


class Snapshot:
    def __init__(self, actors, seconds, frame):
        self.actors = {a.id: a for a in actors if a.is_alive}
        self.timestamp = SimpleNamespace(elapsed_seconds=seconds)
        self.frame = frame

    def find(self, actor_id):
        return self.actors.get(actor_id)


class TM:
    def __init__(self):
        self.calls = []

    def get_port(self):
        return 8010

    def __getattr__(self, key):
        # Catch unsupported guesses against the INSTALLED official binding.
        if not hasattr(carla.TrafficManager, key):
            raise AttributeError(key)
        def record(*args):
            self.calls.append((key, tuple(a.id if isinstance(a, Actor) else a for a in args)))
        return record


class World:
    def __init__(self, road_map):
        self.road_map, self.library = road_map, Library()
        self.ego = Actor(1, road_map.get_waypoint_xodr(40, -3, 10).transform)
        self.actors, self.seconds, self.frame = ActorList([self.ego]), 0., 10
        self.synced = True

    def get_settings(self):
        return SimpleNamespace(synchronous_mode=self.synced)

    def get_map(self):
        return self.road_map

    def get_blueprint_library(self):
        return self.library

    def get_actors(self):
        return self.actors

    def get_snapshot(self):
        return Snapshot(self.actors, self.seconds, self.frame)

    def try_spawn_actor(self, blueprint, pose):
        actor = Actor(len(self.actors)+1, pose, blueprint.id,
                      blueprint.get_attribute('role_name').value)
        self.actors.append(actor)
        return actor


class TrafficOfflineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.road_map = carla.Map('Town04-offline', MAP_FILE.read_text(encoding='utf-8'))

    def test_exact_footprint_distances_rotation_and_intersection(self):
        a = Footprint(0, 0, 0, 0, 2, 1)
        self.assertAlmostEqual(horizontal_body_clearance(a, replace(a, x=8)), 4)
        self.assertAlmostEqual(horizontal_body_clearance(a, replace(a, y=5)), 3)
        self.assertAlmostEqual(horizontal_body_clearance(a, replace(a, x=8, y=5)), 5)
        self.assertAlmostEqual(horizontal_body_clearance(a, replace(a, x=7, yaw=90)), 4)
        self.assertEqual(horizontal_body_clearance(a, replace(a, x=1, yaw=60)), 0)

    def test_time_gap_reservations_and_different_levels(self):
        cfg = TrafficConfig()
        a = Footprint(0, 0, 0, 0, 2.4, 1, 40, -3, 10, 15)
        self.assertEqual(spawn_rejection(replace(a, x=20, s_m=30), [a], cfg), 'same_lane_time_gap')
        self.assertIsNone(spawn_rejection(replace(a, x=40, s_m=50), [a], cfg))
        self.assertIsNone(spawn_rejection(replace(a, z=8), [a], cfg))
        self.assertEqual(spawn_rejection(replace(a, y=1, lane_id=-2), [a], cfg), 'body_clearance')
        cfg = replace(cfg, exclude_xodr=({'road_id': 40, 'lane_ids': [-3], 's_start': 0, 's_end': 20},))
        self.assertEqual(spawn_rejection(a, [], cfg), 'reserved_native_corridor')
        cfg = replace(cfg, exclude_xodr=(), exclude_circles=({'x': 0, 'y': 0, 'radius_m': 10},))
        self.assertEqual(spawn_rejection(a, [], cfg), 'reserved_world_area')

    def test_offline_native_candidates_deduplicate_and_do_not_mutate_map(self):
        corridor = {'road_id': 40, 'lane_ids': [-4, -3, -2, -1, 4, 5],
                    's_start': 12, 's_end': 220, 'spacing_m': 10}
        cfg = TrafficConfig.from_mapping({'corridors': [corridor, corridor], 'spawn_indices': []})
        before = self.road_map.get_waypoint_xodr(40, -3, 12).transform.location.z
        once = build_spawn_candidates(self.road_map, replace(cfg, corridors=(corridor,)), carla)
        twice = build_spawn_candidates(self.road_map, cfg, carla)
        self.assertEqual(len(once), len(twice))
        self.assertGreater(len(once), 100)
        self.assertTrue(all(not wp.is_junction and wp.lane_type == carla.LaneType.Driving for wp, _ in twice))
        self.assertAlmostEqual(self.road_map.get_waypoint_xodr(40, -3, 12).transform.location.z, before)
        self.assertAlmostEqual(twice[0][1].location.z-twice[0][0].transform.location.z, cfg.spawn_z_offset_m, places=5)

    def test_setup_is_reproducible_spaced_and_uses_supported_safe_tm_api(self):
        config = {'count': 18, 'seed': 73, 'require_count': True,
            'corridors': [{'road_id': 40, 'lane_ids': [-4, -3, -2, -1, 4, 5],
                           's_start': 12, 's_end': 220, 'spacing_m': 10}]}
        pairs = []
        for _ in range(2):
            world, tm = World(self.road_map), TM()
            owned = []
            session = setup_dynamic_traffic(world, tm, world.ego, config,
                carla_api=carla, register_actor=owned.append)
            self.assertEqual(owned, session.actors)
            self.assertEqual(world.ego.calls, [])
            self.assertEqual(len(session.actors), 18)
            pairs.append((session, tm, world))
        first, second = pairs[0][0].evidence, pairs[1][0].evidence
        self.assertEqual(first['actors'], second['actors'])
        self.assertEqual(first['rejections'], second['rejections'])
        self.assertGreater(len({round(r['driver_profile']['cruise_cap_kmh'], 3) for r in first['actors']}), 10)
        for name, args in pairs[0][1].calls:
            if name.startswith('ignore_'):
                self.assertEqual(args[1], 0.)
            self.assertNotEqual(name, 'force_lane_change')
            self.assertNotEqual(name, 'collision_detection')
        self.assertIn(('set_hybrid_physics_mode', (False,)), pairs[0][1].calls)
        self.assertIn(('set_respawn_dormant_vehicles', (False,)), pairs[0][1].calls)
        records = first['actors']
        for i, record in enumerate(records):
            body = Footprint(**record['spawn_bbox_from_native_asset'])
            profile = record['driver_profile']
            body = replace(body, speed_mps=profile['cruise_cap_kmh']/3.6)
            others = [replace(Footprint(**r['spawn_bbox_from_native_asset']), speed_mps=r['driver_profile']['cruise_cap_kmh']/3.6)
                      for r in records[:i]]
            self.assertIsNone(spawn_rejection(body, others, pairs[0][0].config))

    def test_real_bbox_local_location_is_unchanged(self):
        actor = World(self.road_map).ego
        before = (actor.bounding_box.location.x, actor.bounding_box.location.y, actor.bounding_box.location.z)
        _actor_footprint(actor, self.road_map)
        _actor_footprint(actor, self.road_map)
        after = (actor.bounding_box.location.x, actor.bounding_box.location.y, actor.bounding_box.location.z)
        self.assertEqual(before, after)

    def test_native_spawn_cached_zero_until_owner_tick_regression(self):
        class PendingActor(Actor):
            def get_transform(self):
                return carla.Transform()  # native accessor cache before owner tick

        class PendingWorld(World):
            def try_spawn_actor(self, blueprint, pose):
                actor=PendingActor(len(self.actors)+1,pose,blueprint.id,
                    blueprint.get_attribute('role_name').value)
                self.actors.append(actor)
                return actor

            def get_snapshot(self):
                # New actors have not entered a native physics snapshot yet.
                return Snapshot([self.ego],self.seconds,self.frame)

        world,tm=PendingWorld(self.road_map),TM()
        owned=[]
        session=setup_dynamic_traffic(world,tm,world.ego,
            {'count':24,'seed':7,'require_count':True,
             'corridors':[{'road_id':40,'lane_ids':[-4,-3,-2,-1,4,5,6],
                           's_start':8,'s_end':250,'spacing_m':12}]},
            carla_api=carla,register_actor=owned.append)
        self.assertEqual(len(session.actors),24)
        self.assertEqual(owned,session.actors)
        self.assertTrue(all(a.get_transform().location.length()==0 for a in owned))
        self.assertTrue(all(abs(r['spawn_bbox_from_native_asset']['x'])>100 for r in session.evidence['actors']))
        self.assertTrue(all(r['spawn_pose_basis']=='requested native spawn transform before first physics tick'
                            for r in session.evidence['actors']))
        self.assertEqual(world.frame,10)  # module has no tick API to call

    def test_registered_actor_cleanup_is_exclusively_caller_owned(self):
        world,tm=World(self.road_map),TM();owned=[]
        with self.assertRaises(RuntimeError):
            setup_dynamic_traffic(world,tm,world.ego,
                {'count':10,'require_count':True,
                 'corridors':[{'road_id':40,'lane_ids':[-3],'s_start':100,'s_end':100}]},
                carla_api=carla,register_actor=owned.append)
        self.assertEqual(len(owned),1)
        self.assertTrue(owned[0].is_alive)
        self.assertNotIn(('destroy',),owned[0].calls)
        owned[0].destroy()
        self.assertEqual(owned[0].calls.count(('destroy',)),1)

    def test_measured_front_gap_response_recovery_and_ego_protection(self):
        world, tm = World(self.road_map), TM()
        session = setup_dynamic_traffic(world, tm, world.ego,
            {'count': 1, 'require_count': True, 'seed': 2,
             'corridors': [{'road_id': 40, 'lane_ids': [-3], 's_start': 100, 's_end': 100}]}, carla_api=carla)
        bg = session.actors[0]
        forward = bg.pose.get_forward_vector()
        bg.velocity = carla.Vector3D(x=10*forward.x, y=10*forward.y)
        lead_pose = copied_pose(bg.pose)
        lead_pose.location += carla.Location(x=forward.x*15, y=forward.y*15)
        lead = Actor(100, lead_pose, role='slow_lead', velocity=carla.Vector3D(x=forward.x*5, y=forward.y*5))
        world.actors.append(lead)
        world.seconds, world.frame = .25, 11
        row = session.update(world.get_snapshot())['actors'][0]
        self.assertEqual(row['front_vehicle']['actor_id'], lead.id)
        self.assertLess(row['desired_speed_kmh'], 18.)
        self.assertGreater(row['front_to_back_gap_target_m'], 17.)
        self.assertFalse(row['near_ego_protection'])
        lead.is_alive = False
        prior = row['desired_speed_kmh']
        world.seconds, world.frame = .50, 12
        row = session.update(world.get_snapshot())['actors'][0]
        self.assertLessEqual(row['desired_speed_kmh']-prior, 1.2*.25*3.6+1e-6)
        world.ego.pose = copied_pose(bg.pose)
        world.ego.pose.location += carla.Location(x=-forward.x*10, y=-forward.y*10)
        world.seconds, world.frame = .75, 13
        row = session.update(world.get_snapshot())['actors'][0]
        self.assertTrue(row['near_ego_protection'])
        self.assertFalse(any(row['lane_change_allowed'].values()))
        self.assertFalse(session.update(world.get_snapshot())['updated'])

    def test_async_world_and_insufficient_density_fail_without_leaking_actors(self):
        world, tm = World(self.road_map), TM()
        world.synced = False
        with self.assertRaises(RuntimeError):
            setup_dynamic_traffic(world, tm, world.ego, carla_api=carla)
        self.assertEqual(len(world.actors), 1)
        world.synced = True
        with self.assertRaises(RuntimeError):
            setup_dynamic_traffic(world, tm, world.ego,
                {'count': 10, 'require_count': True,
                 'corridors': [{'road_id': 40, 'lane_ids': [-3], 's_start': 100, 's_end': 100}]}, carla_api=carla)
        self.assertTrue(all(not a.is_alive for a in world.actors[1:]))
        self.assertTrue(world.ego.is_alive)


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(TrafficOfflineTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = {'schema': 'offline_dynamic_traffic_contract_v1', 'passed': result.wasSuccessful(),
        'tests_run': result.testsRun, 'failures': [str(t) for t, _ in result.failures],
        'errors': [str(t) for t, _ in result.errors],
        'carla_binding_version': '0.9.16',
        'native_offline_map': str(MAP_FILE),
        'native_map_sha256': hashlib.sha256(MAP_FILE.read_bytes()).hexdigest(),
        'module_sha256': hashlib.sha256((BASE/'dynamic_traffic.py').read_bytes()).hexdigest(),
        'no_client_or_server_tick': True, 'live_maneuvers_or_safety_validated': False}
    (BASE/'verification/dynamic-traffic-offline-tests.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    sys.exit(0 if result.wasSuccessful() else 1)
