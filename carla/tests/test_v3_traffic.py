"""Offline native route-arc and nearest-start regressions; no Client/tick."""
from pathlib import Path
from types import SimpleNamespace as Obj
import math
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import carla
from v3_traffic import NativeRouteMetric, V3TrafficController
from native_controller import NativeRuleController
from test_v3_queue_reapproach import fixture, QueueEgo
from test_junction_coast_stop import snapshot, FakeActors


class MetricMap:
    def get_waypoint_xodr(self,road,lane,s):
        if road==1:
            # A 150m-radius curve; s really increases along this lane.
            pose=carla.Transform(carla.Location(x=150*math.sin(s/150),y=150*(1-math.cos(s/150))),
                                 carla.Rotation(yaw=math.degrees(s/150)))
        elif road==2:
            pose=carla.Transform(carla.Location(x=s,y=10.))
        else:
            return None
        return Obj(road_id=road,lane_id=lane,s=s,transform=pose)


class FakeBase:
    def __init__(self,world,actor,plan,api):
        self.index=0;self.plan=plan;self.stages=plan['stages'];self.events=[];self.diagnostics={}
    def step(self,snapshot):
        return carla.VehicleControl(throttle=.2)


class NativeTrafficTests(unittest.TestCase):
    def test_role_forwards_explicit_native_policy_without_weakening_headway(self):
        m=MetricMap();world=Obj(get_map=lambda:m)
        spec={'name':'rear','nominal_speed_kmh':80,'desired_time_headway_s':1.8,'min_body_gap_m':6.,
            'following_policy':{'native_lane_corridor':True},'xodr_path':[[1,-2,s] for s in range(20)]}
        with patch('v3_traffic.NativeRuleController',FakeBase):
            c=V3TrafficController(world,Obj(attributes={'role_name':'rear'}),spec,carla_api=carla)
        self.assertEqual(c.base.plan['following_policy'],
            {'native_lane_corridor':True,'min_body_gap_m':6.,'time_headway_s':1.8})

    def test_explicit_leader_cannot_tangent_fallback_into_known_parallel_lane(self):
        m=MetricMap();own_wp=m.get_waypoint_xodr(1,-2,10.);other_wp=m.get_waypoint_xodr(1,-1,20.)
        def actor(identity,role,wp):
            return Obj(id=identity,attributes={'role_name':role},get_transform=lambda:wp.transform,
                get_location=lambda:wp.transform.location,get_velocity=lambda:carla.Vector3D(x=10.),
                bounding_box=Obj(extent=Obj(x=2.,y=1.)),native_wp=wp)
        own=actor(1,'rear',own_wp);other=actor(2,'hero',other_wp);actors=[own,other]
        class Actors(list):
            def filter(self,_):return self
        m.get_waypoint=lambda location:min(actors,key=lambda a:a.get_location().distance(location)).native_wp
        world=Obj(get_map=lambda:m,get_actors=lambda:Actors(actors))
        spec={'name':'rear','nominal_speed_kmh':92,'lead_role':'hero',
            'following_policy':{'native_lane_corridor':True},'xodr_path':[[1,-2,s] for s in range(50)]}
        with patch('v3_traffic.NativeRuleController',FakeBase):
            c=V3TrafficController(world,own,spec,carla_api=carla)
        c.step(snapshot(1.))
        self.assertIsNone(c.diagnostics['v3_leader'])
        self.assertEqual(c.diagnostics['v3_requested_speed_mps'],92/3.6)
        # Ramp plans without the explicit policy retain the previous fallback.
        spec['following_policy']['native_lane_corridor']=False
        with patch('v3_traffic.NativeRuleController',FakeBase):
            legacy=V3TrafficController(world,own,spec,carla_api=carla)
        legacy.step(snapshot(1.))
        self.assertIsNotNone(legacy.diagnostics['v3_leader'])
        self.assertEqual(legacy.diagnostics['v3_leader']['separation_method'],'local_tangent')

    def corridor_fixture(self,*,optin=True,same_lane=False,other_road=False,junction=False,near=False,opposite=False):
        c,ego=fixture(x=29.,speed=15.)
        c.coast_stop.enabled=False
        c.stages[0].pop('stop_at_end',None)
        leader=QueueEgo(x=ego.x+(7. if near else 20.),speed=5.)
        leader.id=2
        leader.get_transform=lambda:carla.Transform(carla.Location(x=leader.x,y=0. if near else 2.))
        c.world.get_actors=lambda:FakeActors([leader])
        def waypoint(location):
            other=location.x>ego.x+1
            return Obj(road_id=7 if other and other_road else 6,
                lane_id=(1 if opposite else (-1 if same_lane else -2)) if other else -1,
                is_junction=junction,transform=carla.Transform(location),lane_width=3.5)
        c.road_map.get_waypoint=waypoint
        c.plan['following_policy']={'native_lane_corridor':optin}
        c.step(snapshot(1.))
        return c

    def test_native_lane_filter_removes_only_clear_parallel_lane_false_leader(self):
        self.assertEqual(self.corridor_fixture().diagnostics['blocking'],[])

    def test_native_lane_filter_optout_keeps_conservative_old_projection(self):
        self.assertTrue(self.corridor_fixture(optin=False).diagnostics['blocking'])

    def test_native_lane_filter_keeps_actual_same_lane_leader(self):
        self.assertTrue(self.corridor_fixture(same_lane=True).diagnostics['blocking'])

    def test_native_lane_filter_keeps_crossroad_and_junction_risks(self):
        self.assertTrue(self.corridor_fixture(other_road=True).diagnostics['blocking'])
        self.assertTrue(self.corridor_fixture(junction=True).diagnostics['blocking'])

    def test_native_lane_filter_keeps_near_body_and_opposite_lane_risks(self):
        self.assertTrue(self.corridor_fixture(near=True).diagnostics['blocking'])
        self.assertTrue(self.corridor_fixture(opposite=True).diagnostics['blocking'])

    def test_physical_follower_uses_observed_curved_arc_without_state_setters(self):
        m=MetricMap()
        def actor(identity,role,s,speed):
            wp=m.get_waypoint_xodr(1,-2,s);pose=wp.transform;f=pose.get_forward_vector()
            return Obj(id=identity,attributes={'role_name':role},
                get_transform=lambda:pose,get_location=lambda:pose.location,
                get_velocity=lambda:carla.Vector3D(x=f.x*speed,y=f.y*speed),
                bounding_box=Obj(extent=Obj(x=2.,y=1.)),native_wp=wp)
        own=actor(1,'rear',10.,15.);leader=actor(2,'hero',60.,10.)
        actors=[own,leader]
        class Actors(list):
            def filter(self,_):return self
        m.get_waypoint=lambda location:min(actors,key=lambda a:a.get_location().distance(location)).native_wp
        world=Obj(get_map=lambda:m,get_actors=lambda:Actors(actors))
        spec={'name':'rear','nominal_speed_kmh':92,'lead_role':'hero',
            'xodr_path':[[1,-2,s] for s in range(151)]}
        with patch('v3_traffic.NativeRuleController',FakeBase):
            c=V3TrafficController(world,own,spec,carla_api=carla)
        control=c.step(snapshot(1.))
        self.assertIsInstance(control,carla.VehicleControl)
        self.assertEqual(c.diagnostics['v3_leader']['separation_method'],'native_route_arc')
        self.assertAlmostEqual(c.diagnostics['v3_leader']['actual_body_gap_m'],46.,places=3)
        self.assertAlmostEqual(c.diagnostics['v3_requested_speed_mps'],16.475,places=3)
        self.assertFalse(c.diagnostics['state_overrides'])

    def test_curve_uses_native_arc_and_rejects_other_lane(self):
        m=MetricMap();metric=NativeRouteMetric(m,[[1,-2,s] for s in range(151)])
        a=metric.project(m.get_waypoint_xodr(1,-2,10))
        b=metric.project(m.get_waypoint_xodr(1,-2,90))
        self.assertAlmostEqual(b-a,80,places=3)
        self.assertIsNone(metric.project(m.get_waypoint_xodr(1,-1,90)))
        self.assertIsNone(metric.project(m.get_waypoint_xodr(1,-2,160)))

    def test_positive_lane_decreasing_s_keeps_forward_arc(self):
        m=MetricMap();metric=NativeRouteMetric(m,[[1,2,s] for s in range(100,-1,-1)])
        self.assertGreater(metric.project(m.get_waypoint_xodr(1,2,20)),
                           metric.project(m.get_waypoint_xodr(1,2,80)))

    def test_false_jump_and_loop_are_rejected(self):
        with self.assertRaisesRegex(ValueError,'Discontinuous'):
            NativeRouteMetric(MetricMap(),[[1,-2,0],[1,-2,20]])
        with self.assertRaisesRegex(ValueError,'ambiguous'):
            NativeRouteMetric(MetricMap(),[[1,-2,0],[1,-2,1],[1,-2,0]])

    def test_multi_stage_lane_change_hole_is_not_a_fake_connection(self):
        m=MetricMap();world=Obj(get_map=lambda:m)
        actor=Obj(attributes={'role_name':'rear'})
        spec={'name':'rear','nominal_speed_kmh':80,'stages':[
            {'name':'before','kind':'follow_path','xodr_path':[[1,-2,s] for s in range(10)],'speed_kmh':60},
            {'name':'change','kind':'lane_change','direction':'left','distance_m':90,'speed_kmh':50},
            {'name':'after','kind':'follow_path','xodr_path':[[1,-1,s] for s in range(110,151)],'speed_kmh':40}]}
        with patch('v3_traffic.NativeRuleController',FakeBase):
            c=V3TrafficController(world,actor,spec,carla_api=carla)
        self.assertEqual(set(c.route_metrics),{0,2})
        self.assertEqual(c.stage_speed_caps,[60,50,40])

    def test_nearest_optin_selects_actual_deep_reference(self):
        c,_=fixture(x=29.,speed=8.)
        c.plan.pop('longitudinal_stop_policy')
        c.coast_stop.enabled=False
        c.stages[0]['start_from_nearest_native_reference']=True
        c.path=None;c.progress=0
        c.step(snapshot(1.))
        event=next(e for e in c.events if e['event']=='native_reference_start_selected')
        self.assertEqual(event['reference_index'],58)
        self.assertEqual(event['reference_error_m'],0.)
        self.assertNotIn('waiting',c.diagnostics)

    def test_nearest_optout_preserves_old_local_start_rejection(self):
        c,_=fixture(x=29.,speed=8.)
        c.path=None;c.progress=0
        c.step(snapshot(1.))
        self.assertEqual(c.diagnostics['waiting'],'excessive_reference_error')
        self.assertFalse(any(e['event']=='native_reference_start_selected' for e in c.events))

    def test_nearest_optin_retains_3_5m_gate(self):
        c,ego=fixture(x=29.,speed=8.)
        ego.get_transform=lambda:carla.Transform(carla.Location(x=29.,y=3.6))
        c.stages[0]['start_from_nearest_native_reference']=True
        c.path=None;c.progress=0
        c.step(snapshot(1.))
        self.assertIsNone(c.path)
        self.assertEqual(c.diagnostics['waiting'],'excessive_initial_reference_error')
        self.assertGreater(c.diagnostics['error_m'],3.5)


if __name__=='__main__':
    unittest.main(verbosity=2)
