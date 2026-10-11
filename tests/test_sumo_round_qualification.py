"""The roundabout representation scope does not weaken physical/source gates."""
from pathlib import Path
import copy
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'carla'))
import audit_sumo_sync_round_qualified as q


class RoundQualificationTests(unittest.TestCase):
    def setUp(self):
        self.scope={'background_samples':list(range(41,55)),'ego_samples':[157,473],
            'background_actor_id':2,'ego_actor_id':1,'expected_samples':1001,'expected_actor_samples':23023,
            'strict_input_sha256':{'source':'immutable'}}
        failures=[{'kind':'readback_lane_missing_from_actual_network','sample':s,'actor_id':2 if s<55 else 1,'lane':''}
                  for s in [*range(41,55),157,473]]
        self.strict={'audit_type':'independent_all_sample_native_SUMO_replay_mapping','passed':False,
            'failures_count':16,'failures_examples':failures,'failed_checks':[q.CHECK],
            'checks':{q.CHECK:False,'exact_source_pose_time_images':True},'samples_checked':1001,
            'actors_checked':23023,'native_pngs_fully_decoded':1001,'input_sha256':{'source':'immutable'}}
        self.net=ET.fromstring('''<net><edge id="in"><lane id="in_0" index="0" width="3.5" shape="0,0 5,0"><param key="origId" value="25_-2"/></lane></edge><edge id=":j" function="internal"><lane id=":j_0" index="0" width="3.5" shape="5,0 10,0"><param key="origId" value="1577_-2"/></lane></edge><edge id="out"><lane id="out_0" index="0" width="3.5" shape="10,0 15,0"><param key="origId" value="38_-4"/></lane></edge><edge id="-44"><lane id="-44_1" index="1" width="0.63" shape="0,30 20,30"/></edge><connection from="in" fromLane="0" to="out" toLane="0" via=":j_0"/><connection from=":j" fromLane="0" to="out" toLane="0"/></net>''')
        def actor(id,ego,lane,x):
            return {'carla_id':id,'ego':ego,'role':'hero' if ego else 'dynamic_bg_013',
                'readback_sumo_lane_id':lane,'readback_lane_origId':{'in_0':'25_-2',':j_0':'1577_-2','out_0':'38_-4'}.get(lane,''),
                'readback_sumo_front_bumper_xy_m':[x,0],'readback_sumo_nav_angle_deg':90,
                'bbox_extent_m':[1,.5,.5],'recorded_native_nearest_waypoint':{'road_id':44,'lane_id':-5,'s_m':x}}
        self.rows=[{'source_frame':1000+i,'actors':[actor(1,True,'in_0',4),actor(2,False,'in_0',4)]} for i in range(1001)]
        for s in range(41,55):
            a=self.rows[s]['actors'][1];a['readback_sumo_lane_id']='';a['recorded_native_nearest_waypoint']['s_m']=s/10
        self.rows[157]['actors'][0]=actor(1,True,'',5.04)
        self.rows[158]['actors'][0]=actor(1,True,':j_0',6)
        self.rows[472]['actors'][0]=actor(1,True,':j_0',9)
        self.rows[473]['actors'][0]=actor(1,True,'',10.05)
        self.rows[474]['actors'][0]=actor(1,True,'out_0',11)
        self.metrics={s:{'actual_boundary':[[0,10],[20,20]],'original_center_ego_blue_pixels':30,
                        'changed_pixels':1,'PSNR_dB':63,'bytes_identical':False} for s in [*range(41,55),157,473]}

    def facts(self):return q.qualification_facts(self.rows,self.net,self.scope,self.metrics)
    def test_exact_scope_keeps_both_membership_flags_false(self):
        q.strict_gate(self.strict,self.scope);f=self.facts()
        self.assertFalse(f['all_actor_lane_membership']);self.assertFalse(f['ego_lane_membership_all_frames'])
        self.assertEqual(f['ego_offnetwork_samples'],2);self.assertEqual(f['background_offnetwork_samples'],14)
    def test_longer_background_gap_rejected(self):
        self.rows[55]['actors'][1]['readback_sumo_lane_id']=''
        with self.assertRaises(ValueError):self.facts()
    def test_extra_ego_gap_rejected(self):
        self.rows[158]['actors'][0]['readback_sumo_lane_id']=''
        with self.assertRaises(ValueError):self.facts()
    def test_ego_endpoint_more_than_point_one_rejected(self):
        self.rows[157]['actors'][0]['readback_sumo_front_bumper_xy_m']=[5.11,0]
        with self.assertRaises(ValueError):self.facts()
    def test_visible_background_rejected(self):
        self.metrics[43]['actual_boundary']=[[0,-2],[20,2]]
        with self.assertRaises(ValueError):self.facts()
    def test_missing_native_screenshot_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(FileNotFoundError):q.image_metrics(Path(d)/'missing.png',Path(d)/'missing2.png')
    def test_actual_ego_not_shown_rejected(self):
        self.metrics[157]['original_center_ego_blue_pixels']=0
        with self.assertRaises(ValueError):self.facts()
    def test_disconnected_endpoint_rejected(self):
        self.net.remove(self.net.find('connection'))
        with self.assertRaises(ValueError):self.facts()
    def test_native_wrong_road_rejected(self):
        self.rows[43]['actors'][1]['recorded_native_nearest_waypoint']['road_id']=45
        with self.assertRaises(ValueError):self.facts()
    def test_actual_background_trajectory_reversed_rejected(self):
        self.rows[44]['actors'][1]['recorded_native_nearest_waypoint']['s_m']=1
        with self.assertRaises(ValueError):self.facts()
    def test_source_pose_failure_cannot_be_qualified(self):
        self.strict['checks']['exact_source_pose_time_images']=False
        with self.assertRaises(ValueError):q.strict_gate(self.strict,self.scope)
    def test_wrong_full_frame_range_rejected(self):
        self.strict['samples_checked']=1000
        with self.assertRaises(ValueError):q.strict_gate(self.strict,self.scope)
    def test_pixel_repeat_within_explicit_bound_allowed(self):
        q.repeat_render_gate(2,61,[[0,0],[20,20]],[[0,0],[20,20]],[{'xy_error_m':0,'yaw_error_deg':0}])
    def test_large_pixel_difference_rejected(self):
        with self.assertRaises(ValueError):q.repeat_render_gate(5,70,[],[],[{'xy_error_m':0,'yaw_error_deg':0}])
    def test_psnr_is_disclosed_not_a_second_arbitrary_proof_gate(self):
        q.repeat_render_gate(3,58.7,[],[],[{'xy_error_m':0,'yaw_error_deg':0}])
    def test_changed_actual_viewport_rejected(self):
        with self.assertRaises(ValueError):q.repeat_render_gate(0,None,[[0,0],[20,20]],[[0,0],[21,20]],[{'xy_error_m':0,'yaw_error_deg':0}])
    def test_changed_actual_native_pose_rejected(self):
        with self.assertRaises(ValueError):q.repeat_render_gate(0,None,[],[],[{'xy_error_m':.001,'yaw_error_deg':0}])


if __name__=='__main__':unittest.main()
