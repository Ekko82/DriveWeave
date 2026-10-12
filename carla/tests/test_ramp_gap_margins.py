"""Start-margin regression tests; no server, tick or simulator connection."""
from pathlib import Path
import copy
import sys
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from native_controller import NativeRuleController


class Waypoint:
    def __init__(self, lane=-7, s=0.):
        self.lane_id,self.road_id,self.s=lane,37,s
        self.is_junction=False
        self.lane_type='Driving'
        self.lane_change='Both'
        self.transform=NS(location=NS(x=s,y=(lane+7)*3.5,z=0.))
    def get_left_lane(self):
        return Waypoint(self.lane_id+1,self.s)
    def next(self, step):
        return [Waypoint(self.lane_id,self.s+step)]


class StartMargins(unittest.TestCase):
    def decide(self, front=20., rear=25., margin=True):
        controller=object.__new__(NativeRuleController)
        controller.road_map=NS(get_waypoint=lambda _:Waypoint())
        controller.ego=NS(get_location=lambda:None)
        controller.carla=NS(LaneType=NS(Driving='Driving'))
        gap={'front':{'actor_id':1,'gap_m':front,'required_m':20.,'actual_speed_mps':12.},
             'rear':{'actor_id':2,'gap_m':rear,'required_m':25.,'actual_speed_mps':13.}}
        controller.lane_gap=lambda *_:([],copy.deepcopy(gap))
        controller.event=lambda *_,**__:None
        stage={'kind':'lane_change','direction':'left','distance_m':45.}
        if margin:stage['gap_acceptance_margin_m']={'front':2.,'rear':3.}
        return controller.build_lane_change(stage,None,[],12.),gap

    def test_old_zero_margin_accepts_original_exact_boundaries(self):
        (path,reason,details),_=self.decide(margin=False)
        self.assertTrue(path);self.assertIsNone(reason)
    def test_front_original_threshold_does_not_satisfy_extra_margin(self):
        (path,reason,details),_=self.decide(front=20.,rear=28.)
        self.assertIsNone(path);self.assertEqual(reason,'target_lane_gap')
        self.assertEqual(details['risks'][0]['required_m'],20.)
        self.assertEqual(details['risks'][0]['required_with_margin_m'],22.)
    def test_rear_original_threshold_does_not_satisfy_extra_margin(self):
        (path,reason,details),_=self.decide(front=22.,rear=25.)
        self.assertIsNone(path);self.assertEqual(reason,'target_lane_gap')
        self.assertEqual(details['risks'][0]['planning_margin_m'],3.)
        self.assertEqual(details['risks'][0]['required_m'],25.)
    def test_exact_front_and_rear_extra_margins_release(self):
        (path,reason,details),_=self.decide(front=22.,rear=28.)
        self.assertTrue(path);self.assertIsNone(reason)
    def test_small_deficit_below_front_margin_still_refuses(self):
        (path,reason,details),_=self.decide(front=21.999999,rear=28.)
        self.assertIsNone(path);self.assertEqual(reason,'target_lane_gap')
    def test_small_deficit_below_rear_margin_still_refuses(self):
        (path,reason,details),_=self.decide(front=22.,rear=27.999999)
        self.assertIsNone(path);self.assertEqual(reason,'target_lane_gap')
    def test_independent_required_values_are_not_replaced_by_margin(self):
        (_,_,details),original=self.decide(front=20.,rear=25.)
        self.assertEqual(details['gap'],original)
        self.assertEqual([r['required_m'] for r in details['risks']],[20.,25.])


if __name__=='__main__':
    unittest.main()
