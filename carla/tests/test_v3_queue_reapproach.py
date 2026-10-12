"""Regression for a queue stopping before its own stop line; no physics client."""
from pathlib import Path
from types import SimpleNamespace as Obj
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import carla
from native_controller import NativeRuleController, junction_coast_stop_policy
from test_junction_coast_stop import FakeEgo, FakeActors, FakeWorld, snapshot


class QueueEgo(FakeEgo):
    is_alive = True
    attributes = {'role_name': 'queue'}

    def __init__(self, x=29., speed=.09):
        self.x, self.speed = x, speed

    def get_transform(self):
        return carla.Transform(carla.Location(x=self.x))


def fixture(*, optin=True, x=29., speed=.09, blocker=False):
    ego = QueueEgo(x, speed)
    world = FakeWorld()
    if blocker:
        leader = QueueEgo(x + 7., 0.)
        leader.id = 2
        world.get_actors = lambda: FakeActors([leader])
    policy = junction_coast_stop_policy()
    if optin:
        policy['allow_reapproach_after_intermediate_stop'] = True
    plan = {'longitudinal_stop_policy': policy, 'stages': [
        {'name': 'queue_approach', 'kind': 'follow_path',
         'path': [[i * .5, 0., 0.] for i in range(71)],
         'speed_kmh': 30., 'stop_at_end': True, 'complete_radius_m': 1.2}]}
    c = NativeRuleController(world, ego, plan, carla)
    c.path = c.load_path(plan['stages'][0])
    c.progress = round(x / .5)
    c.coast_stop.latched_stage = 0
    return c, ego


class QueueReapproachTests(unittest.TestCase):
    def test_stopped_queue_moves_to_same_line_when_leader_clears(self):
        c, ego = fixture()
        c.step(snapshot(1.))
        self.assertIsNone(c.coast_stop.latched_stage)
        self.assertEqual(c.index, 0)
        self.assertEqual(c.reapproach_stage, 0)
        self.assertEqual(c.diagnostics['desired_speed_mps'], 2.)
        self.assertEqual(c.events[-1]['event'], 'intermediate_stop_reapproach')
        ego.speed = .5
        c.step(snapshot(1.05))
        self.assertLessEqual(c.diagnostics['desired_speed_mps'], 2.)

    def test_queue_does_not_release_into_stationary_leader(self):
        c, _ = fixture(blocker=True)
        control = c.step(snapshot(1.))
        self.assertEqual(c.coast_stop.latched_stage, 0)
        self.assertIsNone(c.reapproach_stage)
        self.assertEqual(control.throttle, 0.)
        self.assertGreater(control.brake, 0.)

    def test_moving_car_keeps_existing_coast_until_measured_stop(self):
        c, _ = fixture(speed=.11)
        c.step(snapshot(1.))
        self.assertEqual(c.coast_stop.latched_stage, 0)
        self.assertIsNone(c.reapproach_stage)

    def test_v2_default_latch_is_unchanged_without_optin(self):
        c, _ = fixture(optin=False)
        c.step(snapshot(1.))
        self.assertEqual(c.coast_stop.latched_stage, 0)
        self.assertIsNone(c.reapproach_stage)

    def test_actual_stop_at_original_line_completes_without_reapproach(self):
        c, _ = fixture(x=34.6)
        c.step(snapshot(1.))
        self.assertEqual(c.index, 1)
        self.assertIsNone(c.reapproach_stage)

    def test_stop_just_outside_completion_radius_does_not_deadlock(self):
        c, _ = fixture(x=33.6)
        c.step(snapshot(1.))
        self.assertEqual(c.index, 0)
        self.assertEqual(c.reapproach_stage, 0)
        self.assertIsNone(c.coast_stop.latched_stage)


if __name__ == '__main__':
    unittest.main(verbosity=2)
