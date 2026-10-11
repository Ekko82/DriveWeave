"""Junction-only native-control policy checks; no Client or world tick."""
from pathlib import Path
from types import SimpleNamespace as Obj
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import hashlib
import json
import unittest
import numpy as np
import carla
from native_controller import (LongitudinalGovernor, JunctionCoastStop,
                               NativeRuleController, junction_coast_stop_policy,
                               remaining_path_arc)


class FakeEgo:
    type_id = 'vehicle.tesla.model3'
    id = 1
    bounding_box = Obj(extent=Obj(x=2.39589,y=1.081725))
    def __init__(self):self.speed=.25
    def get_velocity(self):return carla.Vector3D(x=self.speed)
    def get_transform(self):return carla.Transform(carla.Location(x=34.6))
    def get_location(self):return self.get_transform().location
    def get_physics_control(self):return Obj(wheels=[Obj(max_steer_angle=70)])
    def is_at_traffic_light(self):return False
    def set_autopilot(self,value):assert value is False


class FakeActors(list):
    def filter(self,name):return self


class FakeMap:
    def get_waypoint(self,location):return Obj(road_id=6,lane_id=-1,
        transform=carla.Transform(location),lane_width=3.5)


class FakeWorld:
    def get_map(self):return FakeMap()
    def get_settings(self):return Obj(fixed_delta_seconds=.05)
    def get_actors(self):return FakeActors()


def snapshot(timestamp):return Obj(frame=int(timestamp*20),timestamp=Obj(elapsed_seconds=timestamp))


class JunctionCoastTests(unittest.TestCase):
    def test_positive_brake_never_used_for_optin_low_speed_stop(self):
        governor=LongitudinalGovernor(.05)
        for speed in [2.5,1.7,1.,.5,.25,.10001]:
            self.assertEqual(governor.step(0.,speed,coast_stop=True),(0.,0.))
        self.assertEqual(governor.step(0.,.1,coast_stop=True),(0.,.35))

    def test_emergency_physical_brake_is_not_hidden(self):
        self.assertEqual(LongitudinalGovernor(.05).step(0.,2.,emergency=True,coast_stop=True),(0.,1.))

    def test_existing_default_governor_not_silently_coasted(self):
        governor=LongitudinalGovernor(.05)
        self.assertGreater(governor.step(0.,.2)[1],0.)
        self.assertFalse(JunctionCoastStop({},.05).enabled)

    def test_native_arc_for_curve_exceeds_shortcut(self):
        path=[[0.,0.],[5.,0.],[5.,5.]]
        self.assertAlmostEqual(remaining_path_arc([2.,0.],path,0),8.)
        self.assertAlmostEqual(remaining_path_arc([5.,2.],path,1),3.)

    def test_coast_latch_prevents_low_speed_brake_reintroduction(self):
        policy=JunctionCoastStop(junction_coast_stop_policy(),.05)
        self.assertFalse(policy.approach(0,30.,5.)[0])
        self.assertTrue(policy.approach(0,12.,5.)[0])
        self.assertTrue(policy.approach(0,6.,.25)[0])
        self.assertFalse(policy.approach(1,30.,5.)[0])

    def test_confirmation_uses_observed_speed_and_resets_on_motion(self):
        policy=JunctionCoastStop(junction_coast_stop_policy(),.05)
        policy.observe_actual_stop(.25,1.)
        self.assertEqual(policy.stopped_seconds(2.),0.)
        policy.observe_actual_stop(.09,2.)
        self.assertAlmostEqual(policy.stopped_seconds(2.5),.5)
        policy.observe_actual_stop(.11,2.6)
        self.assertEqual(policy.stopped_seconds(3.),0.)

    def test_stage_completion_and_yield_release_require_real_zero_speed(self):
        ego=FakeEgo();plan={'longitudinal_stop_policy':junction_coast_stop_policy(),
            'stages':[{'name':'approach','kind':'follow_path','path':[[i*.5,0.,0.] for i in range(71)],
                'speed_kmh':21.6,'stop_at_end':True,'complete_radius_m':1.2},
                {'name':'yield','kind':'follow_path','path':[[35.+i*.5,0.,0.] for i in range(10)],
                 'yield_gate':{'polygon_xy':[[40,0],[41,0],[41,1],[40,1]]}}]}
        controller=NativeRuleController(FakeWorld(),ego,plan,carla)
        # This fixture tests the terminal speed gate after genuine reference
        # progress; it does not simulate driving from the path's origin.
        controller.path=controller.load_path(plan['stages'][0])
        controller.progress=67
        control=controller.step(snapshot(1.))
        self.assertEqual(controller.index,0)
        self.assertEqual(control.brake,0.)
        ego.speed=.09;controller.step(snapshot(1.05))
        self.assertEqual(controller.index,1)
        controller.step(snapshot(1.10))
        self.assertFalse(controller.gate_released)
        controller.step(snapshot(1.60))
        self.assertTrue(controller.gate_released)

    def test_table_exactly_binds_actual_coast_pose_samples(self):
        policy=junction_coast_stop_policy()
        source=Path(__file__).resolve().parents[1]/policy['calibration_source']
        if not source.exists():self.skipTest('Optional native diagnostic records absent in portable package')
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(),policy['calibration_frames_sha256'])
        rows=[json.loads(l) for l in source.read_text(encoding='utf-8').splitlines()]
        first=next(i for i,r in enumerate(rows) if r['phase']=='brake')
        last=next(i for i in range(first,len(rows)) if rows[i]['speed_mps']<=.1)
        points=np.array([r['location_m'] for r in rows[first:last+1]])
        remaining=np.r_[np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1)[::-1])[::-1],0.]
        for speed,distance in policy['coasting_speed_remaining_distance_m']:
            i=min(range(first,last+1),key=lambda i:abs(rows[i]['speed_mps']-speed))
            self.assertAlmostEqual(rows[i]['speed_mps'],speed,places=10)
            self.assertAlmostEqual(remaining[i-first],distance,places=8)


if __name__=='__main__':unittest.main(verbosity=2)
