"""Dispatcher contracts using synthetic temporary evidence; no simulator starts."""
from pathlib import Path
from types import SimpleNamespace
import json
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'carla'))
import build_sumo_pane_scene as pipeline


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name).resolve()
        self.run=self.root/'carla/captures-v2/fixture-left'
        self.processed=self.root/'carla/processed-v2/fixture-left'
        self.frames=[{'sample':i,'frame':100+i,'timestamp':10+i/20,'time_s':i/20} for i in range(120)]
        self.ids=[r['frame'] for r in self.frames]
        self.write(self.run/'manifest.json',{'map':'/Game/Carla/Maps/Town05','fps':20,'duration_s':6,
            'status':'complete','native_cleanup_complete':True,'controller_plan_completed':True,'collision_count':0})
        (self.run/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.frames),encoding='utf-8')
        (self.run/'map.xodr').write_bytes(b'<OpenDRIVE>\r\n</OpenDRIVE>\r\n')
        self.hashes={p:pipeline.sha(self.run/p) for p in ['manifest.json','frames.jsonl']}
        self.spec={'id':'left-turn','run':str(self.run)}
        self.report('execution',{'returncode':0,'native_process_returncode':0,'capture_run':str(self.run),
            'manifest_sha256':self.hashes['manifest.json'],'frames_jsonl_sha256':self.hashes['frames.jsonl']})
        for role,key in [('motion_audit','motion_quality_validated'),('action_audit','scenario_actions_validated')]:
            self.report(role,{key:True,'checks':{'fixture_only':True},'run':str(self.run),'input_sha256':self.hashes})
        self.write(self.processed/'processing-manifest.json',{'synthetic_unit_fixture':True})
        proc_sha=pipeline.sha(self.processed/'processing-manifest.json')
        self.report('sensor_audit',{'overall_status':'automated_checks_passed_manual_review_needed',
            'failures':[],'missing_evidence':[],'native_process_exit0_verified':True,
            'capture_manifest_sha256':self.hashes['manifest.json'],'source_frames_jsonl_sha256':self.hashes['frames.jsonl'],
            'captured_frames':120,'processed_manifest_sha256':proc_sha,'execution_sha256':pipeline.sha(self.spec['execution'])})
        native_video=self.root/'carla/output-v2/fixture.mp4';native_video.parent.mkdir(parents=True)
        native_video.write_bytes(b'NOT REAL MEDIA: metadata fixture for no-simulator unit tests')
        self.report('native_video_manifest',{'source_run':str(self.run),'processed_run':str(self.processed),
            'capture_manifest_sha256':self.hashes['manifest.json'],'processing_manifest_sha256':proc_sha,
            'native_frame_ids':self.ids,'added_text':False,'interpolated_frames':False,
            'playback_speed':1,'synthetic_sensor_data':False,'outputs':[{'file':str(native_video),
                'sha256':pipeline.sha(native_video),'bytes':native_video.stat().st_size,'frames':120,'fps':20,'duration_s':6}]})
        self.report('native_video_audit',{'overall_status':'automated_checks_passed_manual_review_needed',
            'failures':[],'video_manifest_sha256':pipeline.sha(self.spec['native_video_manifest']),'native_frame_ids':self.ids})
        for key,value in {'network_dir':'carla/sumo-network-v2/left-turn-fixture',
            'replay_dir':'carla/sumo-replay-v2/left-turn-fixture',
            'pane_dir':'carla/output-sumo-v2/left-turn-fixture',
            'sumo_process_file':'carla/sumo-replay-v2/left-turn-fixture-process.json',
            'sumo_audit':'carla/verification/left-turn-fixture-sync.json',
            'pane_audit':'carla/verification/left-turn-fixture-video.json',
            'pipeline_receipt':'carla/verification/left-turn-fixture-pipeline.json',
            'manual_review':'verification/left-turn-fixture-manual.json'}.items():
            self.spec[key]=value

    def tearDown(self):self.temp.cleanup()

    def write(self,path,value):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(value),encoding='utf-8')

    def report(self,key,value):
        path=self.root/'verification'/(key+'.json');self.write(path,value);self.spec[key]=str(path)

    def change(self,key,**updates):
        value=pipeline.read(self.spec[key]);value.update(updates);self.write(self.spec[key],value)

    def test_preflight_is_readonly_without_process_or_fake_manual_acceptance(self):
        with patch.object(pipeline.subprocess,'run') as run:
            context=pipeline.source_preflight(self.spec,self.root)
        run.assert_not_called()
        self.assertEqual(context['ids'],self.ids)
        self.assertEqual(context['source_sha256']['map.xodr'],pipeline.sha(self.run/'map.xodr'))
        self.assertFalse((self.root/self.spec['pipeline_receipt']).exists())
        self.assertFalse((self.root/self.spec['manual_review']).exists())

    def test_motion_interaction_failure_and_nonzero_exit_rejected(self):
        self.change('action_audit',scenario_actions_validated=False)
        with self.assertRaises(ValueError):pipeline.source_preflight(self.spec,self.root)
        self.change('action_audit',scenario_actions_validated=True,failed_checks=['actual_failed_check'])
        with self.assertRaises(ValueError):pipeline.source_preflight(self.spec,self.root)
        self.change('action_audit',failed_checks=[])
        self.change('execution',returncode=1)
        with self.assertRaises(ValueError):pipeline.source_preflight(self.spec,self.root)

    def test_current_source_changes_or_source_frame_cut_rejected(self):
        self.change('native_video_manifest',native_frame_ids=self.ids[:-1])
        with self.assertRaises(ValueError):pipeline.source_preflight(self.spec,self.root)
        self.change('native_video_manifest',native_frame_ids=self.ids)
        (self.run/'frames.jsonl').write_text('{}\n')
        with self.assertRaises((ValueError,KeyError)):pipeline.source_preflight(self.spec,self.root)

    def test_existing_outputs_other_scene_names_and_parent_directories_refused(self):
        path=self.root/self.spec['replay_dir'];path.mkdir(parents=True)
        with self.assertRaisesRegex(ValueError,'Never replace'):pipeline.source_preflight(self.spec,self.root)
        path.rmdir()
        self.spec['pane_dir']='carla/output-sumo-v2/highway'
        with self.assertRaisesRegex(ValueError,'scene-specific'):pipeline.source_preflight(self.spec,self.root)
        self.spec['pane_dir']='carla/output-sumo-v2'
        with self.assertRaisesRegex(ValueError,'namespace'):pipeline.source_preflight(self.spec,self.root)

    def test_outer_process_file_cannot_precreate_replay_directory(self):
        self.spec['sumo_process_file']=self.spec['replay_dir']+'/left-turn-process.json'
        with self.assertRaisesRegex(ValueError,'outside newly created'):pipeline.source_preflight(self.spec,self.root)

    def test_full_source_commands_never_cut_defer_or_connect_to_carla(self):
        context=pipeline.source_preflight(self.spec,self.root)
        commands=pipeline.command_plan(context)
        self.assertEqual(len(commands),5)
        conversion=commands[0][1]
        self.assertEqual(conversion[conversion.index('--xodr')+1],str(self.run/'map.xodr'))
        for _,command in commands:
            self.assertNotIn('--limit',command)
            self.assertNotIn('--samples',command)
            self.assertNotIn('--defer-sensor-video',command)
            self.assertNotIn('--promote-current',command)
            self.assertNotIn('capture_native.py',' '.join(command))
            self.assertNotIn('capture_live_cosim.py',' '.join(command))

    def test_lf_installed_hash_cannot_replace_actual_crlf_raw_source_hash(self):
        context=pipeline.source_preflight(self.spec,self.root)
        converter=self.root/'runtime/util/netconvert_carla.py';converter.parent.mkdir(parents=True)
        converter.write_bytes(b'official-converter-fixture')
        network=context['targets']['network_dir']/'Town05/official.net.xml'
        network.parent.mkdir(parents=True);network.write_bytes(b'synthetic-network')
        report={'official_converter_sha256':pipeline.sha(converter),'maps':[{'town':'Town05',
            'source_xodr':str(self.run/'map.xodr'),'source_xodr_sha256':pipeline.sha(self.run/'map.xodr'),
            'attempts':[{'kind':'official-carla','actual_returncode':0,'network':{'sha256':pipeline.sha(network)}}]}]}
        investigation=context['targets']['network_dir']/'investigation.json';self.write(investigation,report)
        runtime=SimpleNamespace(carla_sumo_root=self.root/'runtime')
        self.assertEqual(pipeline.converted_network_gate(context,runtime),network)
        lf=self.root/'installed-map.xodr';lf.write_bytes((self.run/'map.xodr').read_bytes().replace(b'\r\n',b'\n'))
        self.assertNotEqual(pipeline.sha(lf),pipeline.sha(self.run/'map.xodr'))
        report['maps'][0]['source_xodr_sha256']=pipeline.sha(lf);self.write(investigation,report)
        with self.assertRaisesRegex(ValueError,'exact actual raw XODR bytes'):
            pipeline.converted_network_gate(context,runtime)

    def test_real_nonzero_subprocess_result_is_recorded_and_stops_all_later_stages(self):
        context=pipeline.source_preflight(self.spec,self.root)
        with patch.object(pipeline,'ROOT',self.root),patch.object(pipeline.subprocess,'run',
                return_value=subprocess.CompletedProcess([],37)) as run,patch('builtins.print'):
            with self.assertRaisesRegex(ValueError,'exited nonzero'):
                pipeline.execute(context,SimpleNamespace())
        self.assertEqual(run.call_count,1)
        receipt=pipeline.read(context['targets']['pipeline_receipt'])
        self.assertEqual(receipt['status'],'failed')
        self.assertEqual(receipt['steps'][0]['actual_returncode'],37)
        self.assertFalse(receipt['completed_media_acceptance_claimed'])
        self.assertFalse(receipt['manual_review_approved'])
        self.assertFalse((self.root/self.spec['manual_review']).exists())


if __name__=='__main__':unittest.main()
