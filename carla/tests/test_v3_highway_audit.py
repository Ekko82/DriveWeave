"""Offline physical action negatives. Retained native fixture is optional.

Portable geometry tests always run. Integration tests read the actual owner
probe when present and mutate only in-memory copies, never its saved sources.
"""
from pathlib import Path
from types import SimpleNamespace as Obj
import copy
import json
import sys
import unittest
import carla

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
from audit_v3_highway import physical_actions
from audit_native_actions import measure_lane_gaps


def vehicle(identity,x,y,speed,ego=False):
    pose=carla.Transform(carla.Location(x=x,y=y))
    return {'id':identity,'ego':ego,'role_name':'hero' if ego else 'target_rear',
        'transform':{'location_m':[x,y,0.],'rotation_deg':[0.,0.,0.], 'world_from_local':pose.get_matrix()},
        'bounding_box':{'center_local_m':[0.,0.,.7],'extent_m':[2.,1.,.7],'rotation_rpy_deg':[0.,0.,0.]},
        'velocity_mps':[speed,0.,0.],'speed_mps':speed}


class StraightMap:
    def get_waypoint(self,location):return self.get_waypoint_xodr(45,-1,location.x)
    def get_waypoint_xodr(self,road,lane,s):return Obj(road_id=road,lane_id=lane,s=s,lane_width=3.5,
        transform=carla.Transform(carla.Location(x=s)))


class PortableGapTests(unittest.TestCase):
    def test_real_closing_speed_is_part_of_rear_gap_requirement(self):
        ego=vehicle(1,0.,0.,15.,True);rear=vehicle(2,-34.,0.,20.)
        proof=measure_lane_gaps({'vehicles':[ego,rear]},ego,StraightMap(),45,-1)
        self.assertEqual(proof['rear']['actual_projected_bumper_gap_m'],30.)
        self.assertEqual(proof['rear']['required_m'],43.)
        self.assertFalse(proof['safe'])

    def test_safe_pose_gap_passes_without_command_labels(self):
        ego=vehicle(1,0.,0.,15.,True);rear=vehicle(2,-50.,0.,20.)
        self.assertTrue(measure_lane_gaps({'vehicles':[ego,rear]},ego,StraightMap(),45,-1)['safe'])


class ActualNativeActionNegatives(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p=HERE/'v3-runtime/probe-highway-06'
        if not (p/'frames.jsonl').exists():
            raise unittest.SkipTest('Retained native owner probe is optional and excluded from public source-only checkout')
        cls.rows=[json.loads(s) for s in (p/'frames.jsonl').read_text(encoding='utf-8').splitlines()]
        cls.plan=json.loads((p/'plan.json').read_text(encoding='utf-8'))
        summary=json.loads((p/'summary.json').read_text(encoding='utf-8'))
        ego=next(v['id'] for v in cls.rows[0]['vehicles'] if v['ego'])
        cls.events=summary['controller_events'][str(ego)]
        cls.native_map=carla.Map('Town04',(p/'map.xodr').read_text(encoding='utf-8'))

    def proof(self,rows=None,events=None):
        return physical_actions(rows or self.rows,self.plan,self.events if events is None else events,self.native_map)[0]

    def test_actual_probe_physical_core_passes(self):
        self.assertTrue(all(self.proof().values()))

    def test_wait_commands_removed_cannot_prove_gap_rejection(self):
        rows=copy.deepcopy(self.rows)
        for row in rows:row['rule_controller']['gap_waiting']=None
        self.assertFalse(self.proof(rows)['actual_unsafe_target_gap_waiting_at_least_1_second'])

    def test_unsafe_actual_gap_is_not_overruled_by_success_event(self):
        rows=copy.deepcopy(self.rows)
        event=next(e for e in self.events if e['event']=='lane_change_started')
        row=next(r for r in rows if r['frame']==event['frame'])
        ego=next(v for v in row['vehicles'] if v['ego'])
        wp=self.native_map.get_waypoint(carla.Location(*ego['transform']['location_m']))
        target=self.native_map.get_waypoint_xodr(45,-1,wp.s-8.)
        v=next(v for v in row['vehicles'] if v['role_name']=='target_rear')
        pose=target.transform
        v['transform'].update(location_m=[pose.location.x,pose.location.y,pose.location.z],world_from_local=pose.get_matrix())
        self.assertFalse(self.proof(rows)['every_actual_lane_change_frame_dynamic_gaps_safe'])

    def test_missing_required_actor_is_not_dense_traffic(self):
        rows=copy.deepcopy(self.rows)
        for row in rows:row['vehicles']=[v for v in row['vehicles'] if v['role_name']!='rear_far']
        self.assertFalse(self.proof(rows)['all_explicit_native_traffic_present_and_actually_moves'])

    def test_arrival_event_cannot_substitute_for_actual_final_pose(self):
        rows=copy.deepcopy(self.rows)
        ego=next(v for v in rows[-1]['vehicles'] if v['ego'])
        ego['transform']['location_m'][0]+=20.
        self.assertFalse(self.proof(rows)['actual_last_pose_reaches_terminal_native_waypoint'])

    def test_missing_completion_event_is_rejected(self):
        events=[e for e in self.events if e['event']!='stage_completed']
        self.assertFalse(self.proof(events=events)['all_completion_events_inside_actual_capture'])


if __name__=='__main__':unittest.main(verbosity=2)
