"""Fail-closed assembly gates using temporary, explicitly synthetic fixtures."""
from pathlib import Path
from datetime import datetime, timezone
import importlib.util
import json
import tempfile
import unittest

MODULE = Path(__file__).resolve().parents[1]/'carla/assemble_v2_delivery.py'
spec = importlib.util.spec_from_file_location('delivery_assembly', MODULE)
assembly = importlib.util.module_from_spec(spec); spec.loader.exec_module(assembly)


class DeliveryGateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.root = Path(self.temp.name).resolve()
        self.run = self.root/'carla/captures-v2/test'
        self.processed = self.root/'carla/processed-v2/test'
        self.replay = self.root/'carla/sumo-replay-v2/test'
        self.frames = [{'sample':i,'frame':100+i,'timestamp':3+i/20,'time_s':i/20} for i in range(3)]
        self.ids = [r['frame'] for r in self.frames]
        self.write(self.run/'manifest.json',{'status':'complete','native_cleanup_complete':True,
             'collision_count':0,'controller_plan_completed':True,'duration_s':.15,'fps':20})
        self.run.joinpath('frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.frames),encoding='utf-8')
        (self.run/'map.xodr').write_text('<OpenDRIVE/>',encoding='utf-8')
        self.hashes={k:assembly.sha(self.run/k) for k in ('manifest.json','frames.jsonl')}
        self.spec={'id':'highway','run':str(self.run)}
        self.report('execution',{'returncode':0,'native_process_returncode':0,'capture_run':str(self.run),
              'manifest_sha256':self.hashes['manifest.json'],'frames_jsonl_sha256':self.hashes['frames.jsonl']})
        for key,passed in [('motion_audit','motion_quality_validated'),('action_audit','scenario_actions_validated')]:
            self.report(key,{passed:True,'checks':{'fixture_only':True},'run':str(self.run),'input_sha256':self.hashes})
        self.write(self.processed/'processing-manifest.json',{'fixture_only':True})
        proc_sha=assembly.sha(self.processed/'processing-manifest.json')
        self.report('sensor_audit',{'overall_status':'automated_checks_passed_manual_review_needed',
             'capture_manifest_sha256':self.hashes['manifest.json'],'source_frames_jsonl_sha256':self.hashes['frames.jsonl'],
             'processed_manifest_sha256':proc_sha,'execution_sha256':assembly.sha(self.spec['execution']),
             'native_process_exit0_verified':True,'captured_frames':3})
        self.native_video=self.root/'carla/output-v2/fixture-native.mp4'; self.native_video.parent.mkdir(parents=True)
        self.native_video.write_bytes(b'NOT A VIDEO: unit test fixture for metadata binding')
        output=lambda p:{'file':str(p),'sha256':assembly.sha(p),'bytes':p.stat().st_size,
                         'frames':3,'fps':20,'duration_s':.15,'size':[1920,1080]}
        presentation={'added_text':False,'interpolated_frames':False,'playback_speed':1,'synthetic_sensor_data':False}
        self.report('native_video_manifest',dict(presentation,source_run=str(self.run),processed_run=str(self.processed),
             native_frame_ids=self.ids,capture_manifest_sha256=self.hashes['manifest.json'],
             processing_manifest_sha256=proc_sha,outputs=[output(self.native_video)]))
        self.report('native_video_audit',{'overall_status':'automated_checks_passed_manual_review_needed',
             'video_manifest_sha256':assembly.sha(self.spec['native_video_manifest']),'native_frame_ids':self.ids})
        self.network=self.root/'carla/net/fixture.xml'; self.network.parent.mkdir(parents=True); self.network.write_bytes(b'network')
        mapped=[]
        for r in self.frames:
            png=self.replay/'frames_native_sumo'/f"{r['frame']:08d}.png"
            png.parent.mkdir(parents=True,exist_ok=True); png.write_bytes(b'PNG fixture'+str(r['frame']).encode())
            mapped.append({'source_sample':r['sample'],'source_frame':r['frame'],'source_timestamp_s':r['timestamp'],
                  'display_time_s':r['time_s'],'native_sumo_png':str(png),'native_sumo_png_sha256':assembly.sha(png)})
        mapping=self.replay/'mapping.jsonl';mapping.write_text(''.join(json.dumps(r)+'\n' for r in mapped),encoding='utf-8')
        self.spec['sumo_manifest']=str(self.replay/'manifest.json')
        self.write(self.spec['sumo_manifest'],{'status':'complete','mode':'post_capture_synchronized_replay',
              'sumo_process_exit_code':0,'native_gui_only':True,'carla_connected_or_ticked':False,
              'two_way_live_control_claimed':False,'network':str(self.network),'network_sha256':assembly.sha(self.network),
              'mapping_sha256':assembly.sha(mapping),'source_manifest_sha256':self.hashes['manifest.json'],
              'source_frames_jsonl_sha256':self.hashes['frames.jsonl'],'source_execution_sha256':assembly.sha(self.spec['execution'])})
        self.report('sumo_audit',{'passed':True,'checks':{'fixture_only':True},'samples_checked':3,
              'native_pngs_fully_decoded':3,'input_sha256':{
              'replay_manifest':assembly.sha(self.spec['sumo_manifest']),'mapping':assembly.sha(mapping),
              'raw_manifest':self.hashes['manifest.json'],'raw_frames':self.hashes['frames.jsonl'],
              'raw_execution':assembly.sha(self.spec['execution']),'raw_xodr':assembly.sha(self.run/'map.xodr'),
              'network':assembly.sha(self.network)}})
        self.pane_video=self.root/'carla/output-v2/fixture-pane.mp4';self.pane_video.write_bytes(b'PANE unit test fixture')
        self.report('pane_manifest',dict(presentation,source_run=str(self.run),native_frame_ids=self.ids,
             native_video_manifest_sha256=assembly.sha(self.spec['native_video_manifest']),
             sumo_manifest_sha256=assembly.sha(self.spec['sumo_manifest']),sumo_mode='post_capture_synchronized_replay',
             source_folders=[str(self.run/'chase'),str(self.replay/'frames_native_sumo'),
                             str(self.processed/'image-processed'),str(self.processed/'lidar-3d')],outputs=[output(self.pane_video)]))
        samples=[]
        for i in range(3):
            image=self.root/'verification'/f'manual-{i}.png';image.parent.mkdir(parents=True,exist_ok=True);image.write_bytes(b'manual fixture'+str(i).encode())
            samples.append({'sample':i,'native_frame':self.ids[i],'image':str(image),
                            'image_sha256':assembly.sha(image),'observed':'Unit fixture only, no real visual acceptance'})
        self.report('pane_audit',{'automated_checks_passed':True,'manifest_sha256':assembly.sha(self.spec['pane_manifest']),
              'decoded_frames':3,'native_frames_expected':3,'samples':samples})
        self.report('manual_review',{'schema':'native_media_manual_review_v2','scene_id':'highway','accepted':True,
              'reviewer':'unit fixture','reviewed_utc':datetime.now(timezone.utc).isoformat(),
              'bindings':{'capture_manifest_sha256':self.hashes['manifest.json'],
                 'source_frames_jsonl_sha256':self.hashes['frames.jsonl'],
                 'pane_manifest_sha256':assembly.sha(self.spec['pane_manifest']),
                 'pane_video_sha256':assembly.sha(self.pane_video)},'samples':samples,
              'full_video_manually_watched':False,'presentation':{
                 'native_tesla_model_retained':True,'native_SUMO_viewport_retained':True,
                 'four_panes_legible':True,'added_titles_or_HUD':False}})

    def tearDown(self):self.temp.cleanup()
    def write(self,path,value):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(value),encoding='utf-8')
    def report(self,key,value):
        path=self.root/'verification'/(key+'.json');self.write(path,value);self.spec[key]=str(path)
    def change(self,key,**updates):
        value=assembly.read(self.spec[key]);value.update(updates);self.write(self.spec[key],value)

    def test_consistently_bound_temporary_fixture_passes_gate(self):
        result=assembly.scene_gate(self.spec,self.root)
        self.assertEqual(result['native_frame_ids'],self.ids)
        self.assertFalse(result['live_joint_simulation_verified'])

    def test_qualified_single_boundary_cannot_be_applied_to_highway(self):
        self.change('sumo_audit',audit_type='qualified_single_background_endpoint_native_SUMO_replay_mapping',
                    all_actor_lane_membership=False)
        with self.assertRaises(ValueError):assembly.scene_gate(self.spec,self.root)

    def test_unspecified_lane_failure_scope_is_rejected(self):
        self.change('sumo_audit',all_actor_lane_membership=False)
        with self.assertRaises(ValueError):assembly.scene_gate(self.spec,self.root)

    def test_nonzero_actual_exit_rejected_before_promoting_anything(self):
        self.change('execution',returncode=1)
        with self.assertRaises(ValueError):assembly.scene_gate(self.spec,self.root)
        self.assertFalse((self.root/'carla/output/delivery-manifest.json').exists())

    def test_stale_source_or_shortened_frame_cut_rejected(self):
        self.change('native_video_manifest',native_frame_ids=self.ids[:-1])
        with self.assertRaises(ValueError):assembly.scene_gate(self.spec,self.root)

    def test_changed_sumo_png_rejected_after_independent_audit(self):
        next((self.replay/'frames_native_sumo').glob('*.png')).write_bytes(b'changed')
        with self.assertRaises(ValueError):assembly.scene_gate(self.spec,self.root)

    def test_changed_video_rejected_after_independent_audit(self):
        self.pane_video.write_bytes(b'changed video')
        with self.assertRaises(ValueError):assembly.scene_gate(self.spec,self.root)

    def test_true_accept_without_real_sample_record_is_rejected(self):
        self.change('manual_review',samples=[])
        with self.assertRaises(ValueError):assembly.scene_gate(self.spec,self.root)

    def test_no_interpolation_or_live_relabeling_allowed(self):
        self.change('sumo_manifest',mode='live')
        with self.assertRaises(ValueError):assembly.scene_gate(self.spec,self.root)

    def test_partial_version_cannot_update_current(self):
        with self.assertRaises(ValueError):assembly.validate_config({'schema_version':1,'version':'v2','scenes':[self.spec]},self.root)

    def test_literal_path_cannot_escape_project(self):
        with self.assertRaises(ValueError):assembly.inside(self.root/'../outside',self.root)

    def test_promotion_preserves_exact_old_bytes_and_no_history_reference(self):
        current=self.root/'carla/output/delivery-manifest.json';current.parent.mkdir(parents=True)
        before=b'{"version":"v1","accepted":true}\r\n';current.write_bytes(before)
        assembly.promote({'version':'v2','scenes':[]},current,self.root)
        history=list((self.root/'verification/history').glob('*.json'))
        self.assertEqual(len(history),1);self.assertEqual(history[0].read_bytes(),before)
        self.assertNotIn('history',current.read_text(encoding='utf-8'))


if __name__=='__main__':unittest.main()
