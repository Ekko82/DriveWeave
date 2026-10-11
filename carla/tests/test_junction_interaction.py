"""Executed offline native-map fixtures and immutable v1 negative regression.

Native waypoint fixtures test geometry/decision logic, not vehicle physics.
The previous actual v1 NPC right turns are independently rejected when their
raw capture remains available. This never declares a new v2 run accepted.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from runtime_config import RuntimeConfig
from audit_junction_interaction import reference, project, straight_motion_proof, first_body_pass, overlap, signal_observations_proof
from signal_policy import town05_wide_left_signal_config
import carla
import json
import hashlib
import unittest
import numpy as np
from copy import deepcopy

ROOT=Path(__file__).resolve().parents[1]
NEGATIVE_EVIDENCE=[]


class TestJunctionInteraction(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        runtime=RuntimeConfig.resolve(require_carla=True)
        cls.source=runtime.carla_root/'CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr'
        cls.map=carla.Map('Town05',cls.source.read_text(encoding='utf-8'))
        cls.plan=json.loads((ROOT/'plans/v2-wide-left-turn.json').read_text(encoding='utf-8'))

    def native_trace(self, points):
        out=[]
        for i,point in enumerate(points):
            wp=self.map.get_waypoint_xodr(*point);p=wp.transform.location
            out.append({'frame':i,'time_s':i*.05,'position_m':[p.x,p.y,p.z],
                        'yaw_deg':wp.transform.rotation.yaw,'native_road':wp.road_id,
                        'junction':wp.is_junction})
        return out

    def test_true_native_oncoming_connector_is_straight(self):
        stages=self.plan['traffic'][0]['stages']
        points=[p for p in stages[0]['xodr_path'] if p[0] in (7,1140,6)]
        good,evidence=straight_motion_proof(self.native_trace(points),reference(self.map,stages,{1140}))
        self.assertTrue(good,evidence)
        self.assertLess(evidence['entry_to_exit_heading_change_deg'],.01)

    def test_actual_native_left_geometry_cannot_pass_as_straight(self):
        stages=self.plan['stages']
        turn=[p for p in stages[1]['xodr_path'] if p[0]==1182]
        straight=reference(self.map,self.plan['traffic'][0]['stages'],{1140})
        good,evidence=straight_motion_proof(self.native_trace(turn),straight)
        self.assertFalse(good)
        self.assertGreater(evidence['entry_to_exit_heading_change_deg'],70)

    def test_shifted_native_reference_motion_rejected(self):
        stages=self.plan['traffic'][0]['stages']
        points=[p for p in stages[0]['xodr_path'] if p[0]==1140]
        trace=self.native_trace(points)
        for q in trace:q['position_m'][1]+=3.5
        good,_=straight_motion_proof(trace,reference(self.map,stages,{1140}))
        self.assertFalse(good)

    def test_left_arc_exact_native_geometry_projectable(self):
        turn=[p for p in self.plan['stages'][1]['xodr_path'] if p[0]==1182]
        points=reference(self.map,self.plan['stages'],{1182})
        fits=[project(q['position_m'],q['yaw_deg'],points) for q in self.native_trace(turn)]
        self.assertLess(max(q['lateral_distance_m'] for q in fits),.001)
        self.assertLess(max(q['reference_heading_error_deg'] for q in fits),3)

    def test_body_gate_pass_waits_for_rear_not_center(self):
        trace=[{'frame':i,'time_s':i*.05,'body_in_gate':i in (2,3,4,5),
                'center_in_gate':i in (2,3)} for i in range(8)]
        p=first_body_pass(trace)
        self.assertEqual(p['fully_clear_frame'],6)
        self.assertEqual(p['last_body_gate_frame'],5)

    def test_body_gate_contact_with_center_outside_detected(self):
        gate=[[0,0],[4,0],[4,4],[0,4]]
        self.assertTrue(overlap([[-2,1],[.1,1],[.1,2],[-2,2]],gate))
        self.assertFalse(overlap([[-2,1],[-.1,1],[-.1,2],[-2,2]],gate))

    def test_actual_v1_right_turn_capture_cannot_pass_straight(self):
        run=ROOT/'captures/20261011-183824-308732'
        if not (run/'frames.jsonl').is_file():self.skipTest('Immutable previous native capture is not included in source-only package')
        manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
        frames=[json.loads(x) for x in (run/'frames.jsonl').read_text(encoding='utf-8').splitlines() if x.strip()]
        m=carla.Map('Town04',(run/'map.xodr').read_text(encoding='utf-8'))
        for specification in manifest['route_specification']['traffic']:
            role=specification['name'];trace=[]
            for f in frames:
                v=next((v for v in f['vehicles'] if v.get('role_name')==role),None)
                if v is None:continue
                p=v['transform']['location_m'];w=m.get_waypoint(carla.Location(*p))
                trace.append({'frame':f['frame'],'time_s':f['time_s'],'position_m':p,
                              'yaw_deg':v['transform']['rotation_deg'][1],
                              'native_road':w.road_id,'junction':w.is_junction})
            good,evidence=straight_motion_proof(trace,np.asarray(specification['path'],float))
            self.assertFalse(good,evidence)
            self.assertGreater(evidence['entry_to_exit_heading_change_deg'],70)
            NEGATIVE_EVIDENCE.append({'role':role,'correctly_rejected_as_straight':not good,'actual_metrics':evidence,
                                     'raw_frames_sha256':hashlib.sha256((run/'frames.jsonl').read_bytes()).hexdigest()})

    def make_signal(self):
        config=town05_wide_left_signal_config()
        states={'1':'Green','2':'Green','3':'Red','4':'Red'}
        row={'native_frame':1,'pass':True,'lights':[{'actor_id':int(k),'native_state':v,'desired_state':v} for k,v in states.items()]}
        return {'config':config,'desired_states':states,'observations':[row]},[{'frame':1}],config

    def test_signal_correct_exact_states_pass(self):
        signal,frames,plan=self.make_signal()
        self.assertTrue(signal_observations_proof(signal,frames,plan))

    def test_signal_empty_light_evidence_rejected(self):
        signal,frames,plan=self.make_signal();signal['observations'][0]['lights']=[]
        self.assertFalse(signal_observations_proof(signal,frames,plan))

    def test_signal_one_opposing_red_rejected(self):
        signal,frames,plan=self.make_signal();signal['observations'][0]['lights'][1]['native_state']='Red'
        self.assertFalse(signal_observations_proof(signal,frames,plan))

    def test_two_actual_native_failed_signal_runs_stay_rejected(self):
        for identifier in ['20261012-033134-164137','20261012-033213-256902']:
            path=ROOT/'captures-v2'/identifier/'signal-policy.json'
            if not path.is_file():continue
            signal=json.loads(path.read_text(encoding='utf-8'))
            observations=signal['observations']
            self.assertTrue(observations)
            self.assertFalse(signal_observations_proof(signal,[{'frame':q['native_frame']} for q in observations],signal['config']))
            NEGATIVE_EVIDENCE.append({'actual_failed_signal_run':identifier,'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                                     'correctly_rejected_as_opposing_green':True,
                                     'native_failure_errors':observations[-1]['errors']})

    def test_signal_spurious_state_name_rejected(self):
        signal,frames,plan=self.make_signal();signal['observations'][0]['lights'][1]['native_state']='NotReallyGreen'
        self.assertFalse(signal_observations_proof(signal,frames,plan))


if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(TestJunctionInteraction)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    report={'offline_tests_passed':result.wasSuccessful(),'tests_run':result.testsRun,
            'skipped':len(result.skipped),'v2_runtime_actions_accepted':False,
            'native_geometry_fixtures_only':True,'server_connected':False,'world_ticked':False,
            'actual_previous_raw_capture_negative_regressions':NEGATIVE_EVIDENCE}
    (ROOT/'verification/v2-junction-auditor-offline-tests.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
