"""Meaningful offline motion/projection/provenance regressions. No CARLA client.

Temporary analytical fixtures are test data, never represented as captures.
Native waypoint geometry comes from the actual exported Town04 map.
"""
from __future__ import annotations
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import carla
import numpy as np
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parent))
import audit_motion_quality as quality

MAP_FILE=quality.CARLA_DIR/'map-inventory/Town04/map.xodr'


def transform(pose):
    return {'location_m':[pose.location.x,pose.location.y,pose.location.z],
        'rotation_deg':[pose.rotation.pitch,pose.rotation.yaw,pose.rotation.roll],
        'world_from_local':pose.get_matrix()}


def vehicle(native_map,actor_id,lane,s,speed=8.,ego=False):
    wp=native_map.get_waypoint_xodr(40,lane,s)
    pose=wp.transform
    forward=pose.get_forward_vector()
    return {'id':actor_id,'ego':ego,'role_name':'hero' if ego else 'analytical_test_neighbor',
        'blueprint':'vehicle.tesla.model3','transform':transform(pose),
        'velocity_mps':[forward.x*speed,forward.y*speed,forward.z*speed],
        'speed_mps':speed,'bounding_box':{'center_local_m':[0,0,.75],
            'extent_m':[2.4,1.05,.75],'rotation_rpy_deg':[0,0,0]},
        'waypoint':{'road_id':wp.road_id,'lane_id':wp.lane_id,'s_m':wp.s},
        'control':{'throttle':.15,'brake':0.,'steer':0.}}


def fixture(directory,native_map,scenario='analytical_offline_test_only'):
    run=directory/'run';run.mkdir()
    (run/'map.xodr').write_bytes(MAP_FILE.read_bytes())
    (run/'collisions.json').write_text('[]',encoding='utf-8')
    records=[]
    for i in range(61):
        t=i*.05;s=40+8*t
        ego=vehicle(native_map,1,-3,s,ego=True)
        neighbors=[vehicle(native_map,2,-1,s+24),vehicle(native_map,3,-4,s+18)]
        sensors={}
        for name,offset in [('rgb',carla.Transform(carla.Location(x=1.7,z=1.5))),
                            ('chase',carla.Transform(carla.Location(x=-6.5,y=2.1,z=2.05),carla.Rotation(pitch=-8,yaw=-14)))]:
            matrix=np.asarray(ego['transform']['world_from_local'])@np.asarray(offset.get_matrix())
            sensors[name]={'frame':100+i,'timestamp':10+t,'width':1280,'height':720,'fov':90,
                'transform':{'world_from_local':matrix.tolist()}}
        records.append({'sample':i,'frame':100+i,'timestamp':10+t,'time_s':t,'sensors':sensors,'vehicles':[ego,*neighbors]})
    manifest={'status':'complete','native_cleanup_complete':True,'frames_captured':len(records),
        'map':'Carla/Maps/Town04','fps':20,'fixed_delta_seconds':.05,'collision_count':0,
        'scenario':scenario,'route_specification':{'map':'Town04','stages':[{'name':'test','kind':'follow_path',
            'xodr_path':[[40,-3,s] for s in range(40,81)]}]}}
    execution=directory/'execution.json'
    def save():
        (run/'manifest.json').write_text(json.dumps(manifest),encoding='utf-8')
        (run/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records),encoding='utf-8')
        execution.write_text(json.dumps({'returncode':0,'native_process_returncode':0,
            'capture_run':str(run),'manifest_sha256':quality.file_sha(run/'manifest.json'),
            'frames_jsonl_sha256':quality.file_sha(run/'frames.jsonl')}),encoding='utf-8')
    save()
    return run,execution,records,manifest,save


class MotionQualityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native_map=carla.Map('Town04-offline',MAP_FILE.read_text(encoding='utf-8'))

    def audit_fixture(self,root,modify=None):
        run,execution,records,manifest,save=fixture(root,self.native_map)
        if modify is not None:modify(records,manifest)
        save()
        with contextlib.redirect_stdout(io.StringIO()):
            report=quality.audit(run,execution,root/'report.json')
        return report

    def test_quadratic_speed_analytical_acceleration_and_jerk_at_endpoints(self):
        times=np.arange(41)*.05+1000.
        relative=times-times[0]
        speed=8+.6*relative+.8*relative**2
        acceleration,jerk=quality.local_polynomial_derivatives(times,speed,.5)
        np.testing.assert_allclose(acceleration,.6+1.6*relative,atol=1e-9)
        np.testing.assert_allclose(jerk,np.full(41,1.6),atol=1e-8)

    def test_pinhole_real_camera_transform_front_behind_and_near_plane_clipping(self):
        v=vehicle(self.native_map,1,-3,40,ego=True)
        pose=np.eye(4);pose[:3,3]=[10,0,0];v['transform']['world_from_local']=pose.tolist()
        v['bounding_box']['center_local_m']=[0,0,0]
        v['bounding_box']['extent_m']=[1,1,1]
        sensor={'width':640,'height':480,'fov':90,'transform':{'world_from_local':np.eye(4).tolist()}}
        result=quality.project_box(quality.obb(v),sensor)
        np.testing.assert_allclose(result['clipped_bbox_xyxy'],[320-320/9,240-320/9,320+320/9,240+320/9],atol=1e-8)
        camera=np.eye(4);camera[0,3]=20;sensor['transform']['world_from_local']=camera.tolist()
        self.assertIsNone(quality.project_box(quality.obb(v),sensor))
        camera[0,3]=9.5;sensor['transform']['world_from_local']=camera.tolist()
        result=quality.project_box(quality.obb(v),sensor)
        self.assertIsNotNone(result)
        self.assertTrue(all(math.isfinite(x) for x in result['clipped_bbox_xyxy']))
        self.assertEqual(result['visibility'],'geometric_candidate_occlusion_unknown')

    def test_smooth_rolling_analytical_scene_passes_with_qualified_visibility(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name)
            report=self.audit_fixture(root)
            self.assertTrue(report['motion_quality_validated'],report['failed_checks'])
            self.assertFalse(report['actual_pixel_occlusion_or_neighbor_identity_verified'])
            self.assertFalse(report['server_connected_or_ticked'])
            records=[json.loads(x) for x in (root/'run/frames.jsonl').read_text().splitlines()]
            brute=[]
            for r in records:
                boxes=[quality.obb(v) for v in r['vehicles']]
                brute.extend(quality.obb_signed_clearance(boxes[a],boxes[b])
                    for a,b in [(0,1),(0,2),(1,2)])
            self.assertAlmostEqual(min(brute),report['all_vehicle_minimum_3d_obb']['signed_clearance_m'],places=8)
            self.assertEqual(report['all_pairs_screened'],len(records)*3)

    def test_kinematic_neighbor_activity_uses_real_pose_changes_and_keeps_zero_velocity_label(self):
        def modify(records,manifest):
            for r in records:
                for v in r['vehicles'][1:]:
                    v['velocity_mps']=[0,0,0];v['speed_mps']=0.
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);report=self.audit_fixture(root,modify)
            self.assertTrue(report['checks']['opening_projected_neighbor_actual_activity_target'])
            series=[json.loads(x) for x in Path(report['series_file']).read_text().splitlines()]
            candidate=series[1]['camera_geometry']['chase']['candidates'][0]
            self.assertEqual(candidate['native_velocity_speed_mps'],0.)
            self.assertGreater(candidate['actual_pose_interval_speed_mps'],7.)
            self.assertTrue(candidate['active_from_actual_state'])

    def test_stationary_actual_velocity_fails_despite_saved_speed_claim(self):
        def modify(records,manifest):
            for r in records[:40]:r['vehicles'][0]['velocity_mps']=[0,0,0]
        with tempfile.TemporaryDirectory() as name:
            report=self.audit_fixture(Path(name),modify)
            self.assertIn('first_frame_actual_forward_speed_at_least_4_mps',report['failed_checks'])
            self.assertIn('opening_two_seconds_actual_moving',report['failed_checks'])
            self.assertIn('native_speed_values_match_actual_velocity',report['failed_checks'])

    def test_real_speed_step_and_control_jump_cannot_pass_smoothing(self):
        def modify(records,manifest):
            for r in records[21:]:
                r['vehicles'][0]['velocity_mps']=[x*2 for x in r['vehicles'][0]['velocity_mps']]
                r['vehicles'][0]['speed_mps']=16.
                r['vehicles'][0]['control']['brake']=1.
        with tempfile.TemporaryDirectory() as name:
            report=self.audit_fixture(Path(name),modify)
            self.assertIn('windowed_longitudinal_acceleration_target',report['failed_checks'])
            self.assertIn('windowed_longitudinal_jerk_target',report['failed_checks'])
            self.assertIn('actual_control_rate_targets',report['failed_checks'])

    def test_bbox_overlap_fails_and_exact_minimum_includes_local_offset(self):
        def modify(records,manifest):
            for r in records:
                r['vehicles'][1]['transform']=json.loads(json.dumps(r['vehicles'][0]['transform']))
                r['vehicles'][1]['waypoint']=dict(r['vehicles'][0]['waypoint'])
                r['vehicles'][1]['bounding_box']['center_local_m']=[0,0,1.]
        with tempfile.TemporaryDirectory() as name:
            report=self.audit_fixture(Path(name),modify)
            self.assertIn('sampled_vehicle_obb_no_overlap',report['failed_checks'])
            self.assertIn('sampled_ego_body_clearance_target',report['failed_checks'])
            self.assertLess(report['ego_minimum_3d_obb']['signed_clearance_m'],0)

    def test_nonfinite_native_npc_and_ego_controls_fail_without_losing_report(self):
        def modify(records,manifest):
            records[12]['vehicles'][1]['control']['brake']=float('nan')
            records[15]['vehicles'][0]['control']['steer']=float('nan')
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);report=self.audit_fixture(root,modify)
            self.assertIn('all_native_vehicle_controls_finite_and_in_range',report['failed_checks'])
            self.assertIn('actual_control_rate_targets',report['failed_checks'])
            evidence=report['details']['all_native_vehicle_controls_finite_and_in_range']
            self.assertEqual(evidence['invalid_actual_samples'],2)
            self.assertEqual(evidence['examples'][0]['actual_value_repr'],'nan')
            self.assertGreater(report['details']['actual_control_rate_targets']['steer']['nonfinite_delta_samples'],0)
            rows=[json.loads(line) for line in Path(report['series_file']).read_text().splitlines()]
            self.assertEqual(rows[15]['control']['steer'],'nan')

    def test_instance_pixels_without_bgra_require_exact_native_hash_not_actor_id_assumption(self):
        with tempfile.TemporaryDirectory() as name:
            run=Path(name);(run/'instance').mkdir()
            pixels=np.zeros((8,8,3),np.uint8);pixels[:4,:,0]=14;pixels[:4,:,1]=3;pixels[:4,:,2]=231
            Image.fromarray(pixels).save(run/'instance/00000100.png')
            bgra=np.dstack((pixels[:,:,[2,1,0]],np.full((8,8),255,np.uint8))).tobytes()
            frame={'frame':100,'sensors':{'instance':{'height':8,'width':8,'raw_sha256':hashlib.sha256(bgra).hexdigest()}}}
            result=quality.vehicle_mask_diagnostic(run,frame,20)
            self.assertTrue(result['native_pixel_provenance_valid'])
            self.assertEqual(result['actual_rendered_vehicle_instance_masks'],1)
            self.assertFalse(result['individual_actor_mapping_verified'])
            self.assertFalse(result['raw_bgra_saved'])
            pixels[0,0,2]=12;Image.fromarray(pixels).save(run/'instance/00000100.png')
            self.assertFalse(quality.vehicle_mask_diagnostic(run,frame,20)['native_pixel_provenance_valid'])

    def test_user_mandatory_speed_requirement_cannot_be_lowered_in_template(self):
        with tempfile.TemporaryDirectory() as name:
            path=Path(name)/'thresholds.json';path.write_text('{"first_forward_speed_mps":3.9}',encoding='utf-8')
            with self.assertRaises(ValueError):quality.load_thresholds(path)

    def test_native_map_provenance_only_accepts_exact_bytes_or_crlf_serialization(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);run,execution,records,manifest,save=fixture(root,self.native_map)
            lf=(run/'map.xodr').read_bytes().replace(b'\r\n',b'\n')
            manifest['route_specification']['source_xodr_sha256']=hashlib.sha256(lf).hexdigest()
            (run/'map.xodr').write_bytes(lf.replace(b'\n',b'\r\n'));save()
            with contextlib.redirect_stdout(io.StringIO()):report=quality.audit(run,execution,root/'report.json')
            self.assertTrue(report['motion_quality_validated'],report['failed_checks'])
            diagnostic=report['details']['route_native_road_references_valid']['map_source_serialization']
            self.assertFalse(diagnostic['expected_matches_raw'])
            self.assertTrue(diagnostic['expected_matches_CRLF_to_LF_only'])
            # Valid XML with identical driving geometry, but an extra byte beyond
            # line endings must fail immutable source provenance.
            (run/'map.xodr').write_bytes((run/'map.xodr').read_bytes()+b' ')
            with contextlib.redirect_stdout(io.StringIO()):report=quality.audit(run,execution,root/'changed-map-report.json')
            self.assertIn('route_native_road_references_valid',report['failed_checks'])
            self.assertIn({'reason':'route_native_map_hash_mismatch'},report['details']['route_native_road_references_valid']['errors'])


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(MotionQualityTests))
    report={'passed':result.wasSuccessful(),'tests_run':result.testsRun,'failures':[str(t) for t,_ in result.failures],
        'errors':[str(t) for t,_ in result.errors],'auditor_sha256':quality.file_sha(Path(quality.__file__)),
        'native_offline_map_sha256':quality.file_sha(MAP_FILE),'fixtures':'analytical offline tests only; never delivery source',
        'server_connected_or_ticked':False}
    (quality.CARLA_DIR/'verification/motion-quality-offline-tests.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
