"""Offline fail-closed/cleanup tests. These never connect to a CARLA server."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from types import SimpleNamespace
import unittest

from signal_policy import SignalPolicyError, setup_signal_policy, town05_wide_left_signal_config


API = SimpleNamespace(TrafficLightState=SimpleNamespace(Red='Red', Green='Green', Yellow='Yellow'))


class Waypoint:
    def __init__(self, road, lane, yaw, *, internal=False, incoming=None):
        self.road_id, self.lane_id, self.s = road, lane, 1.0
        self.is_junction = internal
        self.incoming = incoming
        self.transform = SimpleNamespace(location=SimpleNamespace(x=-50., y=-92., z=0.),
                                         rotation=SimpleNamespace(yaw=yaw))

    def get_junction(self):
        return SimpleNamespace(id=1126)

    def previous(self, distance):
        return [self.incoming] if self.incoming else []

    def next(self, distance):
        return [Waypoint(1182, self.lane_id, self.transform.rotation.yaw, internal=True)]


class Light:
    def __init__(self, identifier, waypoints, world):
        self.id, self.waypoints, self.world = identifier, waypoints, world
        self.state, self.frozen, self.green, self.yellow, self.red = 'Yellow', False, 10., 3., 20.
        self.calls = []
        self.fail_green_once = False
        self.actor_state, self.is_active, self.is_dormant = 'Active', True, False

    def get_stop_waypoints(self): return self.waypoints
    def get_group_traffic_lights(self): return self.world.lights[:4]
    def get_state(self): return self.state
    def get_green_time(self): return self.green
    def get_yellow_time(self): return self.yellow
    def get_red_time(self): return self.red
    def get_elapsed_time(self): return 1.
    def is_frozen(self): return self.frozen
    def set_green_time(self, value): self.calls.append(('green', value)); self.green = value
    def set_yellow_time(self, value): self.calls.append(('yellow', value)); self.yellow = value
    def set_red_time(self, value): self.calls.append(('red', value)); self.red = value

    def set_state(self, value):
        self.calls.append(('state', value))
        if self.fail_green_once and value == 'Green':
            self.fail_green_once = False
            raise RuntimeError('Synthetic native RPC failure during application')
        self.state = value

    def freeze(self, value):
        self.calls.append(('freeze_world', value))
        for light in self.world.lights: light.frozen = value


class World:
    def __init__(self):
        self.lights = []
        config = town05_wide_left_signal_config()
        for i, approach in enumerate(config['green_approaches'] + config['red_approaches']):
            points = []
            for lane in approach['lane_ids']:
                incoming = Waypoint(approach['road_id'], lane, approach['yaw_deg'])
                points.append(Waypoint(1182, lane, approach['yaw_deg'], internal=True, incoming=incoming))
            self.lights.append(Light(i+1, points, self))
        # An unrelated junction is outside the plan's selection radius.
        outside = Waypoint(100, -1, 0)
        outside.transform.location.x = 200.
        self.lights.append(Light(5, [outside], self))
        self.map_name = '/Game/Carla/Maps/Town05'

    def get_actors(self): return SimpleNamespace(filter=lambda pattern: self.lights)
    def get_map(self): return SimpleNamespace(name=self.map_name, to_opendrive=lambda: '<synthetic-test/>')
    def snapshot(self, frame=1):
        return SimpleNamespace(frame=frame, timestamp=SimpleNamespace(elapsed_seconds=frame*.05))


class TestSignalPolicy(unittest.TestCase):
    def assert_untouched(self, world):
        self.assertTrue(all(not light.calls for light in world.lights))

    def test_disabled_never_reads_world(self):
        self.assertIsNone(setup_signal_policy(None, None))
        self.assertIsNone(setup_signal_policy(None, {'enable': False}))

    def test_correct_opposing_green_transverse_red_restore(self):
        world = World()
        session = setup_signal_policy(world, town05_wide_left_signal_config(), carla_api=API)
        self.assertEqual([q.state for q in world.lights], ['Green','Green','Red','Red','Yellow'])
        self.assertFalse(any(q.frozen for q in world.lights))
        self.assertTrue(session.observe(world.snapshot())['pass'])
        self.assertFalse(world.lights[4].calls)
        # One final state request per actor; real snapshot verification is
        # required regardless of the request count.
        self.assertTrue(all(len([c for c in q.calls if c[0]=='state'])==1 for q in world.lights[:4]))
        session.restore()
        self.assertEqual([q.state for q in world.lights], ['Yellow']*5)
        self.assertEqual([(q.green, q.yellow, q.red) for q in world.lights], [(10.,3.,20.)]*5)
        self.assertTrue(session.evidence['restoration']['requests_completed'])

    def test_missing_lane_fails_before_mutation(self):
        world = World(); world.lights[0].waypoints.pop()
        with self.assertRaisesRegex(SignalPolicyError, 'required lanes'):
            setup_signal_policy(world, town05_wide_left_signal_config(), carla_api=API)
        self.assert_untouched(world)

    def test_wrong_direction_fails_before_mutation(self):
        world = World(); world.lights[0].waypoints[0].incoming.transform.rotation.yaw = 0
        with self.assertRaisesRegex(SignalPolicyError, 'direction mismatch'):
            setup_signal_policy(world, town05_wide_left_signal_config(), carla_api=API)
        self.assert_untouched(world)

    def test_mixed_light_lane_policy_fails_before_mutation(self):
        world = World(); world.lights[0].waypoints.append(world.lights[2].waypoints[0])
        with self.assertRaisesRegex(SignalPolicyError, 'incompatible'):
            setup_signal_policy(world, town05_wide_left_signal_config(), carla_api=API)
        self.assert_untouched(world)

    def test_unmatched_group_member_fails_before_mutation(self):
        world = World()
        for q in world.lights[:4]: q.get_group_traffic_lights = lambda: world.lights
        with self.assertRaisesRegex(SignalPolicyError, 'unmatched signal actors'):
            setup_signal_policy(world, town05_wide_left_signal_config(), carla_api=API)
        self.assert_untouched(world)

    def test_runtime_state_drift_is_rejected_and_recorded(self):
        world = World(); session = setup_signal_policy(world, town05_wide_left_signal_config(), carla_api=API)
        session.observe(world.snapshot(1)); world.lights[0].state = 'Red'
        with self.assertRaisesRegex(SignalPolicyError, 'mismatch'):
            session.observe(world.snapshot(2))
        self.assertFalse(session.evidence['observations'][-1]['pass'])
        self.assertFalse(session.evidence['runtime_validated'])
        session.restore()

    def test_same_native_frame_is_rejected(self):
        world = World(); session = setup_signal_policy(world, town05_wide_left_signal_config(), carla_api=API)
        session.observe(world.snapshot(1))
        with self.assertRaisesRegex(SignalPolicyError, 'strictly increasing'):
            session.observe(world.snapshot(1))
        session.restore()

    def test_dormant_native_signal_is_rejected(self):
        world = World(); session = setup_signal_policy(world, town05_wide_left_signal_config(), carla_api=API)
        light = world.lights[0]
        light.actor_state, light.is_active, light.is_dormant = 'Dormant', False, True
        with self.assertRaisesRegex(SignalPolicyError, 'not active'):
            session.observe(world.snapshot(1))
        self.assertEqual(session.evidence['observations'][0]['lights'][0]['actor_state'], 'Dormant')
        session.restore()

    def test_initialization_mismatches_retained_then_five_actual_passes(self):
        world = World(); session = setup_signal_policy(world, town05_wide_left_signal_config(), carla_api=API)
        world.lights[0].state = 'Red'
        self.assertFalse(session.settling_observe(world.snapshot(1))['pass'])
        self.assertEqual(session.evidence['observations'], [])
        self.assertFalse(session.evidence['runtime_validated'])
        world.lights[0].state = 'Green'
        for frame in range(2,7): result=session.settling_observe(world.snapshot(frame))
        self.assertTrue(result['settling_ready'])
        self.assertFalse(session.evidence['settling_observations'][0]['pass'])
        self.assertTrue(session.observe(world.snapshot(7))['pass'])
        world.lights[0].state = 'Red'
        with self.assertRaisesRegex(SignalPolicyError, 'mismatch'):
            session.observe(world.snapshot(8))
        session.restore()

    def test_settling_cannot_mask_a_later_capture_mismatch(self):
        world = World(); session = setup_signal_policy(world, town05_wide_left_signal_config(), carla_api=API)
        session.observe(world.snapshot(1))
        world.lights[0].state = 'Red'
        with self.assertRaisesRegex(SignalPolicyError, 'cannot resume'):
            session.settling_observe(world.snapshot(2))
        session.restore()

    def test_global_freeze_needs_fresh_owner_opt_in(self):
        world = World(); config = town05_wide_left_signal_config(); config['freeze_world_lights'] = True
        with self.assertRaisesRegex(SignalPolicyError, 'owner-exclusive'):
            setup_signal_policy(world, config, carla_api=API)
        self.assert_untouched(world)

    def test_explicit_global_freeze_records_scope_and_restores_all(self):
        world = World(); config = town05_wide_left_signal_config()
        config.update(freeze_world_lights=True, owned_fresh_world=True)
        session = setup_signal_policy(world, config, carla_api=API)
        self.assertTrue(all(q.frozen for q in world.lights))
        self.assertEqual(world.lights[4].state, 'Yellow')
        self.assertTrue(session.observe(world.snapshot())['pass'])
        self.assertTrue(session.evidence['global_freeze_called'])
        self.assertIn('ALL', session.evidence['global_freeze_scope'])
        session.restore()
        self.assertFalse(any(q.frozen for q in world.lights))
        self.assertEqual([q.state for q in world.lights], ['Yellow']*5)
        self.assertEqual(session.evidence['restoration']['actors'], [1,2,3,4,5])

    def test_application_failure_restores_original_states(self):
        world = World(); world.lights[0].fail_green_once = True
        with self.assertRaisesRegex(RuntimeError, 'Synthetic native RPC'):
            setup_signal_policy(world, town05_wide_left_signal_config(), carla_api=API)
        self.assertEqual([q.state for q in world.lights], ['Yellow']*5)
        self.assertEqual([(q.green, q.yellow, q.red) for q in world.lights], [(10.,3.,20.)]*5)


if __name__ == '__main__': unittest.main(verbosity=2)
