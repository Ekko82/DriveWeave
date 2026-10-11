"""Negative cases for the narrowly disclosed, immutable boundary scope."""
import copy
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

spec=importlib.util.spec_from_file_location('boundary',Path(__file__).resolve().parents[1]/'carla/audit_sumo_sync_boundary_qualified.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)


class BoundaryTests(unittest.TestCase):
    def setUp(self):
        self.net=ET.fromstring('''<net><edge id=":j" function="internal"><lane id=":j_0" index="0" width="3.5" shape="0,0 5,0"><param key="origId" value="20_1"/></lane></edge><edge id="out"><lane id="out_0" index="0" width="3.5" shape="5,0 10,0"><param key="origId" value="30_1"/></lane></edge><connection from=":j" to="out" fromLane="0" toLane="0"/></net>''')
        self.scope={'sample':1,'source_frame':101,'actor_id':2,'role':'dynamic_bg_015','expected_samples':3,'expected_actor_samples':6,'strict_input_sha256':{'mapping':'pinned'}}
        self.rows=[]
        for i,(x,l,orig) in enumerate(((4,':j_0','20_1'),(5.05,'',''),(6,'out_0','30_1'))):
            actor={'carla_id':2,'role':'dynamic_bg_015','ego':False,'readback_sumo_lane_id':l,
                   'readback_lane_origId':orig,'readback_sumo_front_bumper_xy_m':[x,0],
                   'readback_sumo_nav_angle_deg':90,'bbox_extent_m':[1,.5,.5]}
            ego=dict(actor,carla_id=1,role='hero',ego=True,readback_sumo_lane_id='out_0')
            self.rows.append({'source_frame':100+i,'actors':[ego,actor]})
        self.proof={'actual_returncode':0,'sample':1,'source_frame':101,'source_actor_id':2,
                    'readback_xy':[5.05,0],'readback_yaw':90,'readback_lane':'',
                    'CARLA_connected_or_ticked':False,'native_or_replay_modified':False,
                    'actual_after_step_boundary':[[0,10],[10,20]]}
        self.strict={'audit_type':'independent_all_sample_native_SUMO_replay_mapping','passed':False,
            'failures_count':1,'failures_examples':[{'kind':'readback_lane_missing_from_actual_network','sample':1,'actor_id':2,'lane':''}],
            'failed_checks':[b.EXCEPTION_CHECK],'checks':{b.EXCEPTION_CHECK:False,'source_and_PNG':True},
            'samples_checked':3,'actors_checked':6,'native_pngs_fully_decoded':3,'input_sha256':{'mapping':'pinned'}}

    def facts(self):return b.qualification_facts(self.rows,self.net,self.proof,self.scope)
    def rejected(self):
        with self.assertRaises(ValueError):self.facts()
    def test_exact_qualified_fact_keeps_false_all_lane(self):
        r=self.facts();self.assertFalse(r['all_actor_lane_membership']);self.assertFalse(r['lane_field_filled'])
        self.assertAlmostEqual(r['connected_endpoint_distance_m'],.05)
    def test_ego_is_never_qualified(self):
        self.rows[1]['actors'][1]['ego']=True;self.rejected()
    def test_non_background_is_never_qualified(self):
        self.rows[1]['actors'][1]['role']='oncoming_0';self.rejected()
    def test_visible_body_rejected_even_front_outside(self):
        self.proof['actual_after_step_boundary']=[[3,-1],[4.5,1]];self.rejected()
    def test_disconnected_lane_rejected(self):
        self.net.remove(self.net.find('connection'));self.rejected()
    def test_shared_endpoint_geometry_required(self):
        self.net.find("edge[@id='out']/lane").set('shape','5.02,0 10,0');self.rejected()
    def test_more_than_point_one_rejected(self):
        self.rows[1]['actors'][1]['readback_sumo_front_bumper_xy_m']=[5.101,0]
        self.proof['readback_xy']=[5.101,0];self.rejected()
    def test_two_missing_samples_rejected(self):
        self.rows[0]['actors'][1]['readback_sumo_lane_id']='';self.rejected()
    def test_outside_approved_sample_rejected(self):
        self.scope['sample']=0;self.rejected()
    def test_outside_approved_actor_rejected(self):
        self.scope['actor_id']=3;self.rejected()
    def test_changed_frame_rejected(self):
        self.scope['source_frame']=999;self.rejected()
    def test_other_ego_missing_rejected(self):
        self.rows[2]['actors'][0]['readback_sumo_lane_id']='';self.rejected()
    def test_missing_adjacent_actor_rejected(self):
        self.rows[0]['actors'][1]['carla_id']=7;self.rejected()
    def test_bad_adjacent_original_id_rejected(self):
        self.rows[0]['actors'][1]['readback_lane_origId']='invented';self.rejected()
    def test_adjacent_pose_off_lane_rejected(self):
        self.rows[2]['actors'][1]['readback_sumo_front_bumper_xy_m']=[6,5];self.rejected()
    def test_viewport_actual_process_failure_rejected(self):
        self.proof['actual_returncode']=1;self.rejected()
    def test_viewport_pose_mismatch_rejected(self):
        self.proof['readback_xy']=[5.06,0];self.rejected()
    def test_wrong_full_source_range_rejected(self):
        self.scope['expected_samples']=4;self.rejected()
    def test_original_single_failure_scope_passes(self):
        b.strict_failure_gate(self.strict,self.scope)
    def test_coordinate_failure_cannot_be_qualified(self):
        self.strict['checks']['source_and_PNG']=False
        with self.assertRaises(ValueError):b.strict_failure_gate(self.strict,self.scope)
    def test_more_original_failures_refused(self):
        self.strict['failures_count']=2
        with self.assertRaises(ValueError):b.strict_failure_gate(self.strict,self.scope)
    def test_original_passing_report_not_relabelled(self):
        self.strict['passed']=True
        with self.assertRaises(ValueError):b.strict_failure_gate(self.strict,self.scope)
    def test_changed_pinned_source_refused(self):
        self.strict['input_sha256']['mapping']='other'
        with self.assertRaises(ValueError):b.strict_failure_gate(self.strict,self.scope)


if __name__=='__main__':unittest.main()
