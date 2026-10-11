"""Synthetic temporary receipts exercise doc gates; no real publication is claimed.

No project media, credentials, network, Git or simulator is used by these tests.
"""
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import TestCase, main
from unittest.mock import patch
import copy
import importlib.util
import json

SOURCE=Path(__file__).resolve().parents[1]/'finalize_release_docs.py'
spec=importlib.util.spec_from_file_location('docs_finalizer',SOURCE)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class Gates(TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.root=Path(self.temp.name).resolve()
        self.put('maintenance.py',b'# synthetic test issuer only\n')
        self.readme=self.put('README.md',('# fixture\n\n## 视频预览与版本状态\n\npending\n\n'
            '## 两种 SUMO 流程\n\npost-capture replay remains replay\n').encode('utf-8'))
        self.delivery={'final_delivery_accepted':True,'version':'v2','scenes':[]}
        self.summary={'version':'v2','status':'accepted_local_media_publication_pending',
            'published':False,'scenes':[]}
        assets=[]
        for i,scene_id in enumerate(m.IDS):
            video=self.put('carla/release-v2/'+scene_id+'.mp4',b'synthetic-test-not-video-'+scene_id.encode())
            name=video.name
            asset={'name':name,'path':video.relative_to(self.root).as_posix(),
                'url':'https://github.com/example/showcase/releases/download/v2/'+name,
                'bytes':video.stat().st_size,'sha256':m.sha(video)}
            assets.append(asset)
            gif=self.put('docs/media/'+scene_id+'.gif',b'GIF89a-synthetic-test-not-media-'+scene_id.encode())
            manual={'accepted':True,'reviewed_sample_indices':[0,1,2],
                'reviewed_native_frames':[i*10,i*10+1,i*10+2],'full_video_manually_watched':False}
            pinned={'id':scene_id,'native_frames':3,'fps':20,'duration_s':.15,
                'native_frame_ids':[i*10,i*10+1,i*10+2],'source_sha256':{'frames.jsonl':'a'*64},
                'manual_review':manual,'requested_action_verified':True,'full_internal_plan_completed':True,
                'live_joint_simulation_verified':scene_id=='live-highway'}
            self.delivery['scenes'].append(pinned)
            self.summary['scenes'].append(dict(pinned,
                first_native_frame=i*10,last_native_frame=i*10+2,source_frame_cut=False,
                displayed_SUMO_viewport_mode='post_capture_synchronized_replay',
                media=[dict(asset,role='carla-sumo-four-pane')],
                preview={'path':gif.relative_to(self.root).as_posix(),'bytes':gif.stat().st_size,
                    'sha256':m.sha(gif),'source_video_sha256':asset['sha256'],'duration_s':6}))
        self.accepted=self.put_json('carla/output/delivery-manifest.json',self.delivery)
        self.summary_path=self.put_json('docs/media/accepted-v2-summary.json',self.summary)
        self.put_json('docs/media/preview-index.json',{'items':[{'id':i,'video_url':None} for i in m.IDS]})
        self.put('docs/media/preview-gallery.md',b'pending')
        self.manifest={'schema_version':1,'cleanup_trigger':'after_successful_publication','version':'v2',
            'repository':'example/showcase','tag':'v2','commit_sha':'a'*40,'assets':assets,
            'accepted_manifests':[{'path':self.accepted.relative_to(self.root).as_posix(),'sha256':m.sha(self.accepted)}]}
        self.pub=self.root/'maintenance/publications/test.json';self.receipt=self.root/'maintenance/reports/test.json'
        self.pin()

    def tearDown(self):self.temp.cleanup()
    def put(self,name,payload):
        p=self.root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(payload);return p
    def put_json(self,name,value):return self.put(name,(json.dumps(value,ensure_ascii=False)+'\n').encode('utf-8'))
    def pin(self):
        self.put_json(self.pub.relative_to(self.root),self.manifest)
        proof={'repository':self.manifest['repository'],'tag':self.manifest['tag'],
            'commit_sha':self.manifest['commit_sha'],'release_id':11,
            'release_url':'https://github.com/example/showcase/releases/tag/'+self.manifest['tag'],
            'published_at':'2026-10-12T00:00:00Z','verified_utc':'2026-10-12T00:01:00+00:00',
            'assets_streamed_and_hash_verified':[dict(a,github_asset_id=i+100) for i,a in enumerate(self.manifest['assets'])]}
        self.put_json(self.receipt.relative_to(self.root),{'schema_version':1,'status':'verified',
            'verification_method':m.METHOD,'publication_manifest_sha256':m.sha(self.pub),
            'verification_script_sha256':m.sha(self.root/'maintenance.py'),'publication':proof,
            'deleted_files':0,'docs_modified':False})
    def finalize(self,execute=False,**kwargs):return m.finalize(self.pub,self.receipt,self.root,execute,**kwargs)

    def test_validation_mode_leaves_docs_pending(self):
        original=self.readme.read_bytes();result=self.finalize()
        self.assertTrue(result['publication_verified']);self.assertFalse(result['public_docs_written'])
        self.assertEqual(self.readme.read_bytes(),original)
        self.assertFalse(m.read(self.summary_path)['published'])

    def test_no_receipt_no_write(self):
        self.receipt.write_text('{"publication_verified":true}')
        original=self.readme.read_bytes()
        with self.assertRaises(ValueError):self.finalize(True)
        self.assertEqual(self.readme.read_bytes(),original)

    def test_actual_receipt_pins_manifest_issuer_commit_and_all_assets(self):
        receipt=m.read(self.receipt)
        changes=[('publication_manifest_sha256','b'*64),('verification_script_sha256','b'*64),
                 ('status','pending'),('verification_method','caller_assertion')]
        for key,value in changes:
            altered=copy.deepcopy(receipt);altered[key]=value
            self.put_json(self.receipt.relative_to(self.root),altered)
            with self.assertRaises(ValueError):self.finalize()
        for field,value in [('commit_sha','b'*40),('release_id',0),('verified_utc','2026-10-11T00:00:00Z')]:
            altered=copy.deepcopy(receipt);altered['publication'][field]=value
            self.put_json(self.receipt.relative_to(self.root),altered)
            with self.assertRaises(ValueError):self.finalize()
        altered=copy.deepcopy(receipt);altered['publication']['assets_streamed_and_hash_verified'].pop()
        self.put_json(self.receipt.relative_to(self.root),altered)
        with self.assertRaises(ValueError):self.finalize()

    def test_changed_video_gif_delivery_and_source_counts_refused(self):
        video=self.root/self.manifest['assets'][0]['path'];old=video.read_bytes();video.write_bytes(b'changed')
        with self.assertRaises(ValueError):self.finalize()
        video.write_bytes(old)
        gif=self.root/'docs/media/highway.gif';old=gif.read_bytes();gif.write_bytes(b'changed')
        with self.assertRaises(ValueError):self.finalize()
        gif.write_bytes(old)
        value=copy.deepcopy(self.summary);value['scenes'][0]['native_frames']=5
        self.put_json(self.summary_path.relative_to(self.root),value)
        with self.assertRaises(ValueError):self.finalize()
        self.put_json(self.summary_path.relative_to(self.root),self.summary)
        self.accepted.write_text('{}')
        with self.assertRaises(ValueError):self.finalize()

    def test_stable_cannot_accept_partial_six_or_false_replay_scope(self):
        for modify in (lambda x:x['scenes'].pop(),
                       lambda x:x['scenes'][0].update(full_internal_plan_completed=False),
                       lambda x:x['scenes'][0].update(displayed_SUMO_viewport_mode='live'),
                       lambda x:x['scenes'][0].update(source_frame_cut=True)):
            value=copy.deepcopy(self.summary);modify(value)
            self.put_json(self.summary_path.relative_to(self.root),value)
            with self.assertRaises(ValueError):self.finalize()

    def pin_left_qualification(self):
        qualifier={'qualification':'qualified_single_background_endpoint_native_SUMO_replay_mapping',
            'all_actor_lane_membership':False,'offnetwork_actor_samples':1,'source_sample':501,
            'native_frame':1200430,'ego_all_lane_membership':True,
            'entire_offnetwork_actor_outside_actual_viewport':True,'exact_pose_preserved':True,
            'lane_field_filled':False,'source_modified':False}
        pinned=next(s for s in self.delivery['scenes'] if s['id']=='left-turn')
        public=next(s for s in self.summary['scenes'] if s['id']=='left-turn')
        pinned.update(native_frames=580,native_frame_ids=list(range(1199929,1200509)),duration_s=29,
                      sumo_replay_qualification=qualifier)
        public.update(native_frames=580,first_native_frame=1199929,last_native_frame=1200508,duration_s=29,
                      sumo_replay_qualification=copy.deepcopy(qualifier))
        self.put_json(self.accepted.relative_to(self.root),self.delivery)
        self.put_json(self.summary_path.relative_to(self.root),self.summary)
        self.manifest['accepted_manifests'][0]['sha256']=m.sha(self.accepted);self.pin()
        return public

    def test_qualified_public_summary_cannot_hide_pinned_limitation(self):
        public=self.pin_left_qualification();public.pop('sumo_replay_qualification')
        self.put_json(self.summary_path.relative_to(self.root),self.summary)
        with self.assertRaises(ValueError):self.finalize()

    def test_qualified_disclosure_is_automatically_rendered(self):
        self.pin_left_qualification();self.finalize(True)
        text=self.readme.read_text(encoding='utf-8')
        self.assertIn('原严格全车道审计 FAIL 仍保留',text)
        self.assertIn('未补写车道或改动姿态',text)

    def test_qualification_cannot_invent_a_frame_outside_accepted_source(self):
        public=self.pin_left_qualification();public['first_native_frame']=1199928
        public['last_native_frame']=1200507
        pinned=next(s for s in self.delivery['scenes'] if s['id']=='left-turn')
        pinned['native_frame_ids']=list(range(1199928,1200508))
        self.put_json(self.accepted.relative_to(self.root),self.delivery)
        self.put_json(self.summary_path.relative_to(self.root),self.summary)
        self.manifest['accepted_manifests'][0]['sha256']=m.sha(self.accepted);self.pin()
        with self.assertRaises(ValueError):self.finalize()

    def test_execute_synthetic_fixture_updates_verified_links_without_current_change(self):
        accepted=self.accepted.read_bytes();self.finalize(True)
        summary=m.read(self.summary_path);index=m.read(self.root/'docs/media/preview-index.json')
        self.assertTrue(summary['published']);self.assertEqual(summary['status'],'published_verified')
        self.assertEqual(len(index['items']),6)
        self.assertTrue(all(i['video_url'].startswith('https://github.com/example/showcase/releases/download/v2/')
                            for i in index['items']))
        text=self.readme.read_text(encoding='utf-8')
        self.assertEqual(text.count('!['),6);self.assertIn('**六片右上角均为录制后的原生 SUMO GUI 同步回放**',text)
        self.assertEqual(self.accepted.read_bytes(),accepted)

    def beta(self):
        self.manifest['version']=self.manifest['tag']='v2.0.0-beta.1'
        self.manifest['assets']=[a for a in self.manifest['assets'] if a['name'][:-4] in m.BETA_IDS]
        for a in self.manifest['assets']:
            a['url']='https://github.com/example/showcase/releases/download/v2.0.0-beta.1/'+a['name']
        config={'schema_version':1,'version':'v2','current_manifest_promoted':False,
            'scenes':[{'id':i} for i in m.BETA_IDS]}
        beta=self.put_json('carla/v2-runtime/partial.json',config)
        self.manifest['accepted_manifests'].append({'path':beta.relative_to(self.root).as_posix(),'sha256':m.sha(beta)})
        self.pin();return beta

    def fake_scene_gate(self,spec,root):
        pinned=next(s for s in self.delivery['scenes'] if s['id']==spec['id'])
        asset=next(a for a in self.manifest['assets'] if a['name']==spec['id']+'.mp4')
        return dict(pinned,pane={'outputs':[asset]})

    def test_beta_four_scene_execution_does_not_make_gifs_promote_or_mark_six_accepted(self):
        beta=self.beta();accepted=self.accepted.read_bytes()
        with patch.object(m,'actual_scene_gate',side_effect=self.fake_scene_gate) as gate:
            result=self.finalize(True,beta_spec=beta)
            self.assertEqual(gate.call_count,8) # Initial check and immediately before writing.
        self.assertEqual(result['verified_scenes'],4);self.assertFalse(result['six_scene_stable_accepted'])
        public=m.read(self.root/'docs/media/beta-public-summary.json')
        self.assertEqual(public['pending_scene_ids'],['left-turn','roundabout'])
        self.assertEqual(self.accepted.read_bytes(),accepted)
        self.assertFalse(m.read(self.summary_path)['published'])
        self.assertNotIn('![',self.readme.read_text(encoding='utf-8'))

    def test_beta_rejects_unpinned_specs_failed_gate_and_changed_mp4(self):
        beta=self.beta()
        with patch.object(m,'actual_scene_gate',side_effect=ValueError('actual source gate failure')):
            with self.assertRaises(ValueError):self.finalize(beta_spec=beta)
        with patch.object(m,'actual_scene_gate',side_effect=lambda s,r:dict(self.fake_scene_gate(s,r),
                pane={'outputs':[{'sha256':'b'*64,'bytes':1}]})):
            with self.assertRaises(ValueError):self.finalize(beta_spec=beta)
        config=m.read(beta);config['current_manifest_promoted']=True;self.put_json(beta.relative_to(self.root),config)
        with self.assertRaises(ValueError):self.finalize(beta_spec=beta)

    def test_beta_cannot_downgrade_published_stable_docs(self):
        beta=self.beta();value=copy.deepcopy(self.summary);value['status']='published_verified'
        self.put_json(self.summary_path.relative_to(self.root),value)
        with self.assertRaises(ValueError):self.finalize(beta_spec=beta)


if __name__=='__main__':main()
