"""Fill public media links only from a pinned, actual remote-SHA receipt.

No network, Git, simulator, publishing or cleanup. Default mode validates only.
The receipt must come from maintenance.py verify-publication, not a caller's
boolean. --execute updates only accepted public docs after every binding passes.
"""
from pathlib import Path
from datetime import datetime
import argparse
import hashlib
import importlib.util
import json
import os
import re
import urllib.parse

ROOT=Path(__file__).resolve().parent
METHOD='actual_public_github_tag_commit_and_full_resource_download_sha256'
IDS=('highway','ramp-entry','ramp-exit','left-turn','roundabout','live-highway')
BETA_IDS=('highway','ramp-entry','ramp-exit','live-highway')
LABELS={'highway':'高速换道','ramp-entry':'匝道驶入','ramp-exit':'匝道驶出',
        'left-turn':'宽路口无保护左转','roundabout':'环岛驶入驶出','live-highway':'官方双向实时协同'}


def require(condition,message):
    if not condition:raise ValueError(message)


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for part in iter(lambda:f.read(4*1024*1024),b''):h.update(part)
    return h.hexdigest()


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def public_clean(payload):
    # Generic documented install examples are portable. Do not confuse the
    # final s:/ inside https:// with a Windows drive path.
    return not re.search(r'(?i)(?<![a-z0-9])[a-z]:[\\/]+(?:Business|Users)[\\/]|/(?:Users|home)/',payload)


def inside(value,root=ROOT):
    root=Path(root).resolve();p=Path(value)
    if not p.is_absolute():p=root/p
    p=p.absolute()
    require(p.is_relative_to(root),'Path outside project refused')
    for q in (p,*p.parents):
        require(not q.is_symlink() and not(q.exists() and getattr(q.lstat(),'st_file_attributes',0)&0x400),
                'Symlink/reparse path refused')
        if q==root:break
    require(p.resolve().is_relative_to(root),'Resolved path outside project refused')
    return p.resolve()


def verify_receipt(publication_path,receipt_path,root=ROOT):
    root=Path(root).resolve();publication_path=inside(publication_path,root);receipt_path=inside(receipt_path,root)
    require(publication_path.is_relative_to(root/'maintenance/publications') and
            receipt_path.is_relative_to(root/'maintenance/reports'),'Require local publication/verification receipt locations')
    manifest=read(publication_path);receipt=read(receipt_path)
    require(receipt.get('schema_version')==1 and receipt.get('status')=='verified' and
            receipt.get('verification_method')==METHOD and
            receipt.get('publication_manifest_sha256')==sha(publication_path) and
            receipt.get('verification_script_sha256')==sha(root/'maintenance.py') and
            receipt.get('deleted_files')==0 and receipt.get('docs_modified') is False,
            'Missing or stale actual remote verification receipt')
    require(manifest.get('schema_version')==1 and
            manifest.get('cleanup_trigger')=='after_successful_publication',
            'Expected the pinned publication manifest')
    proof=receipt['publication'];repo=manifest['repository'];tag=manifest['tag'];commit=manifest['commit_sha']
    require(re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+',repo) and
            re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._/-]*',tag) and '..' not in tag and
            re.fullmatch(r'[0-9a-f]{40}',commit),'Invalid public repository/tag/commit')
    require(proof.get('repository')==repo and proof.get('tag')==tag and proof.get('commit_sha')==commit and
            isinstance(proof.get('release_id'),int) and proof['release_id']>0 and
            proof.get('release_url')=='https://github.com/'+repo+'/releases/tag/'+urllib.parse.quote(tag,safe='/'),
            'Receipt does not bind this exact published release and commit')
    verified_at=datetime.fromisoformat(proof['verified_utc'].replace('Z','+00:00'))
    published_at=datetime.fromisoformat(proof['published_at'].replace('Z','+00:00'))
    require(verified_at.tzinfo and published_at.tzinfo and verified_at>=published_at,
            'Verification must occur after actual publication')
    expected=manifest.get('assets',[]);actual=proof.get('assets_streamed_and_hash_verified',[])
    require(expected and len(expected)==len(actual) and len({a['name'] for a in expected})==len(expected) and
            len({a['name'] for a in actual})==len(actual),
            'Missing, duplicate or partial remote resource verification')
    verified={a['name']:a for a in actual}; assets={}
    for item in expected:
        name=item['name'];path=inside(item['path'],root)
        require(re.fullmatch(r'[A-Za-z0-9_.-]+',name) and path.name==name,'Require literal ASCII release filenames')
        url='https://github.com/'+repo+'/releases/download/'+urllib.parse.quote(tag,safe='')+'/'+urllib.parse.quote(name,safe='')
        observed=verified.get(name,{})
        require(item['url']==url and observed.get('url')==url and
                observed.get('bytes')==item['bytes']==path.stat().st_size and
                observed.get('sha256')==item['sha256']==sha(path) and
                isinstance(observed.get('github_asset_id'),int) and observed['github_asset_id']>0,
                'A release resource was not actually downloaded and hash-verified')
        assets[name]=dict(item)
    for accepted in manifest.get('accepted_manifests',[]):
        require(sha(inside(accepted['path'],root))==accepted['sha256'],'Accepted local delivery manifest changed')
    require(manifest.get('accepted_manifests'),'Require pinned current accepted delivery evidence')
    return manifest,receipt,assets


def verify_inputs(publication_path,receipt_path,root=ROOT):
    root=Path(root).resolve()
    manifest,receipt,assets=verify_receipt(publication_path,receipt_path,root)
    repo=manifest['repository'];tag=manifest['tag'];commit=manifest['commit_sha']
    summary_path=root/'docs/media/accepted-v2-summary.json';summary=read(summary_path)
    require(summary.get('version')=='v2' and summary.get('status') in {
            'accepted_local_media_publication_pending','published_verified'},'No accepted six-scene public summary')
    scenes=summary.get('scenes',[])
    require(len(scenes)==6 and {s['id'] for s in scenes}==set(IDS),'All six actual scenes are required')
    deliveries=[read(inside(a['path'],root)) for a in manifest['accepted_manifests']]
    deliveries=[d for d in deliveries if d.get('final_delivery_accepted') is True and d.get('version')=='v2'
                and len(d.get('scenes',[]))==6 and {s['id'] for s in d['scenes']}==set(IDS)]
    require(len(deliveries)==1,'Require one pinned complete six-scene v2 accepted delivery')
    for scene in scenes:
        pinned=next(s for s in deliveries[0]['scenes'] if s['id']==scene['id'])
        require(scene.get('requested_action_verified') is True and
                scene.get('full_internal_plan_completed') is True and
                scene.get('manual_review',{}).get('accepted') is True and
                scene.get('source_frame_cut') is False and
                scene.get('displayed_SUMO_viewport_mode')=='post_capture_synchronized_replay',
                'Scene evidence/presentation scope is incomplete')
        require(scene.get('native_frames')==pinned['native_frames'] and
                scene.get('fps')==pinned['fps']==20 and scene.get('duration_s')==pinned['duration_s'] and
                scene['duration_s']==scene['native_frames']/scene['fps'] and
                scene.get('first_native_frame')==pinned['native_frame_ids'][0] and
                scene.get('last_native_frame')==pinned['native_frame_ids'][-1] and
                scene.get('source_sha256')==pinned['source_sha256'] and
                scene.get('sumo_replay_qualification')==pinned.get('sumo_replay_qualification') and
                scene.get('manual_review')==pinned['manual_review'] and
                pinned.get('requested_action_verified') is True and pinned.get('full_internal_plan_completed') is True and
                pinned.get('live_joint_simulation_verified') is (scene['id']=='live-highway'),
                'Public scene summary differs from immutable accepted delivery evidence')
        qualification=scene.get('sumo_replay_qualification')
        if qualification is not None:
            left=scene['id']=='left-turn' and (
                    qualification.get('qualification')=='qualified_single_background_endpoint_native_SUMO_replay_mapping'
                    and qualification.get('all_actor_lane_membership') is False
                    and qualification.get('offnetwork_actor_samples')==1
                    and qualification.get('source_sample')==501 and qualification.get('native_frame')==1200430
                    and scene['native_frames']>501 and scene['first_native_frame']+501==1200430
                    and qualification.get('ego_all_lane_membership') is True
                    and qualification.get('entire_offnetwork_actor_outside_actual_viewport') is True
                    and qualification.get('exact_pose_preserved') is True
                    and qualification.get('lane_field_filled') is False
                    and qualification.get('source_modified') is False)
            roundabout=scene['id']=='roundabout' and (
                    qualification.get('qualification')=='qualified_source_pose_native_GUI_roundabout_lane_metadata_limitations'
                    and qualification.get('all_actor_lane_membership') is False
                    and qualification.get('ego_lane_membership_all_frames') is False
                    and qualification.get('offnetwork_actor_samples')==16
                    and qualification.get('ego_offnetwork_samples')==2 and qualification.get('background_offnetwork_samples')==14
                    and qualification.get('background_sample_start')==41 and qualification.get('background_sample_end_inclusive')==54
                    and [s['source_sample'] for s in qualification.get('ego_endpoint_samples',[])]==[157,473]
                    and scene['native_frames']>473
                    and all(s['native_frame']==scene['first_native_frame']+s['source_sample']
                            for s in qualification.get('ego_endpoint_samples',[]))
                    and all(s['connected_endpoint_distance_m']<=.1 and s['native_GUI_ego_visible'] is True
                            for s in qualification.get('ego_endpoint_samples',[]))
                    and qualification.get('all_gap_background_bodies_outside_actual_viewport') is True
                    and qualification.get('exact_source_pose_time_and_original_PNGs_verified') is True
                    and qualification.get('native_GUI_repeat_render_max_changed_pixels',999)<=4
                    and qualification.get('lane_field_filled') is False and qualification.get('source_modified') is False)
            require(left or roundabout,'Public qualification must preserve the exact accepted limitation and unmodified source')
        media=next((a for a in scene['media'] if a.get('role')=='carla-sumo-four-pane'),{})
        verified_media=assets.get(media.get('name'),{})
        require(verified_media and media['sha256']==verified_media['sha256'] and
                media['bytes']==verified_media['bytes'],'A displayed full video is not in the actual verified release')
        for recorded in scene['media']:
            actual_media=assets.get(recorded.get('name'),{})
            require(actual_media and recorded['sha256']==actual_media['sha256'] and
                    recorded['bytes']==actual_media['bytes'],'Every indexed video must belong to this verified release')
        preview=scene['preview'];gif=inside(preview['path'],root)
        require(gif.parent==root/'docs/media' and gif.name==scene['id']+'.gif' and
                gif.stat().st_size==preview['bytes'] and sha(gif)==preview['sha256'] and
                preview['source_video_sha256']==media['sha256'] and
                preview.get('duration_s')==6,'A native-source preview changed')
    if summary.get('published'):
        old=summary.get('publication',{})
        require(old.get('repository')==repo and old.get('tag')==tag and old.get('commit_sha')==commit,
                'Do not relabel already published docs to another release')
    return manifest,receipt,summary,assets


def actual_scene_gate(spec,root):
    source=Path(root)/'carla/assemble_v2_delivery.py'
    module_spec=importlib.util.spec_from_file_location('verified_release_assembly',source)
    module=importlib.util.module_from_spec(module_spec);module_spec.loader.exec_module(module)
    return module.scene_gate(spec,root)


def verify_beta(publication_path,receipt_path,beta_spec,root=ROOT):
    root=Path(root).resolve();manifest,receipt,assets=verify_receipt(publication_path,receipt_path,root)
    public_stable=root/'docs/media/accepted-v2-summary.json'
    if public_stable.is_file():
        require(read(public_stable).get('status')!='published_verified',
                'A beta cannot downgrade published six-scene stable documentation')
    require(re.fullmatch(r'v2\.0\.0-beta\.\d+',manifest['version']) and
            manifest['tag']==manifest['version'],'Beta docs need an explicit matching beta version/tag')
    beta_spec=inside(beta_spec,root);config=read(beta_spec)
    require(any(inside(a['path'],root)==beta_spec and a['sha256']==sha(beta_spec)
                for a in manifest['accepted_manifests']),'Beta scene specs must be publication-pinned')
    require(config.get('schema_version')==1 and config.get('version')=='v2' and
            config.get('current_manifest_promoted') is False,'Beta cannot promote the six-scene current version')
    specs=config.get('scenes',[])
    require(len(specs)==4 and {s['id'] for s in specs}==set(BETA_IDS) and len(assets)==4,
            'Beta accepts exactly the four independently accepted scene videos')
    scenes=[]
    for scene_id in BETA_IDS:
        scene=actual_scene_gate(next(s for s in specs if s['id']==scene_id),root)
        video=scene['pane']['outputs'][0]
        verified=assets.get(scene_id+'.mp4',{})
        require(verified.get('sha256')==video['sha256'] and verified.get('bytes')==video['bytes'],
                'Beta release copy differs from the actually accepted four-pane MP4')
        scenes.append({'id':scene_id,'native_frames':scene['native_frames'],
            'native_frame_first':scene['native_frame_ids'][0],'native_frame_last':scene['native_frame_ids'][-1],
            'fps':scene['fps'],'duration_s':scene['duration_s'],'source_frame_cut':False,
            'requested_action_verified':True,'full_internal_plan_completed':True,
            'displayed_SUMO_viewport_mode':'post_capture_synchronized_replay',
            'live_joint_simulation_verified':scene['live_joint_simulation_verified'],
            'manual_review':scene['manual_review'],
            'video':{k:verified[k] for k in ('name','url','bytes','sha256')}})
    return manifest,receipt,scenes


def render_readme(readme,scenes,assets,proof):
    start=readme.index('## 视频预览与版本状态');end=readme.index('## 两种 SUMO 流程',start)
    part='## 视频预览与版本状态\n\n'
    part+='本版本的六条实际录制、动作、传感器和完整视频已通过自动核验及真实人工抽样审阅。全部原速 20 fps，不裁剪完整源帧、不插值，不添加标题或 HUD。\n\n'
    part+='[已验证 GitHub Release]('+proof['release_url']+')；公开源代码版本 `'+proof['commit_sha']+'`。下方 GIF 是对应完整视频中连续六秒的真实解码画面，以 10 fps 展示，播放时长仍为六秒。\n\n'
    part+='| 场景 | 实际源帧 | 完整时长 | 高清视频 |\n| --- | ---: | ---: | --- |\n'
    ordered=[next(s for s in scenes if s['id']==scene_id) for scene_id in IDS]
    for s in ordered:
        media=next(m for m in s['media'] if m.get('role')=='carla-sumo-four-pane')
        part+='| '+LABELS[s['id']]+' | '+str(s['native_frames'])+' | '+f"{s['duration_s']:g}"+' 秒 | [MP4]('+assets[media['name']]['url']+') |\n'
    part+='\n五条普通场景源于 CARLA 原生物理录制；live-highway 源录制另有实际官方双向协同与 SUMO 自主车辆运动证据。**六片右上角均为录制后的原生 SUMO GUI 同步回放**，不能把展示回放称为重新运行的实时控制。图像框来自仿真真值，点云来自原生 LiDAR 处理；没有训练检测或驾驶模型。\n\n'
    if any(s['id']=='left-turn' and s.get('sumo_replay_qualification') is not None for s in ordered):
        part+='左转 SUMO 回放有一处明确的表示限制：后台车辆在相连道路端点附近的一个样本（501，原生帧 1200430）返回空车道字段，距端点约 5.8 厘米；整辆车位于实际原生 GUI 视口外。原严格全车道审计 FAIL 仍保留，未补写车道或改动姿态。主车全部车道字段及全部源姿态、时间、截图已核验；不能声称所有车辆所有样本车道归属均通过。\n\n'
    if any(s['id']=='roundabout' and s.get('sumo_replay_qualification') is not None for s in ordered):
        part+='环岛 SUMO 回放保留 16 个原始空车道字段：主车在相连道路端点处的两个单帧（157、473，距端点均小于 0.1 米，原生截图主车仍可见），以及一个后台车辆在转换路网 road44/-5 上连续 14 帧（41–54，整车位于实际视口外）。全部源位置、朝向、时间及原始截图 SHA 已核验，但全车辆/全主车样本的车道归属均不能声称通过；原 strict FAIL 保留，未补字段或移动车辆。独立 GUI 重现的少量栅格差异单独实测，原截图始终不变；此片展示的是录制后同步回放。\n\n'
    for s in ordered:
        media=next(m for m in s['media'] if m.get('role')=='carla-sumo-four-pane')
        part+='### '+LABELS[s['id']]+'\n\n!['+LABELS[s['id']]+']('+s['preview']['path']+')\n\n[完整原速视频]('+assets[media['name']]['url']+')\n\n'
    part+='帧数、SHA-256、源采集与展示范围见 [公开验收摘要](docs/media/accepted-v2-summary.json) 和 [预览索引](docs/media/preview-index.json)。人工审阅范围在摘要中逐片披露，不冒充全片人工观看。\n\n'
    return readme[:start]+part+readme[end:]


def atomic_text(path,payload):
    temp=path.with_name(path.name+'.tmp')
    with temp.open('w',encoding='utf-8') as f:f.write(payload);f.flush();os.fsync(f.fileno())
    temp.replace(path)


def finalize_beta(publication_path,receipt_path,beta_spec,root=ROOT,execute=False):
    root=Path(root).resolve();manifest,receipt,scenes=verify_beta(publication_path,receipt_path,beta_spec,root)
    proof=receipt['publication']
    public={'schema_version':1,'version':manifest['version'],'status':'published_verified_beta',
        'published':True,'six_scene_stable_accepted':False,'current_manifest_promoted':False,
        'pending_scene_ids':['left-turn','roundabout'],'scenes':scenes,
        'trained_policy':False,'image_boxes_are_native_ground_truth':True,
        'displayed_SUMO_viewports_are_post_capture_replay':True,
        'publication':{k:proof[k] for k in ('repository','tag','commit_sha','release_url','published_at','verified_utc')},
        'publication_manifest_sha256':sha(inside(publication_path,root)),
        'verification_receipt_sha256':sha(inside(receipt_path,root))}
    readme=(root/'README.md').read_text(encoding='utf-8')
    start=readme.index('## 视频预览与版本状态');end=readme.index('## 两种 SUMO 流程',start)
    part='## 视频预览与版本状态\n\n'
    part+=manifest['version']+' 的四条视频已实际上传 GitHub，并核验公开 tag/commit、所有资源完整下载的字节数与 SHA-256。宽路口左转和环岛尚待完成，完整六场景正式版尚未验收或替换当前交付清单。\n\n'
    part+='[已验证 Beta Release]('+proof['release_url']+')；源代码版本 `'+proof['commit_sha']+'`。\n\n'
    part+='| 场景 | 实际源帧 | 完整时长 | 原速高清录像 |\n| --- | ---: | ---: | --- |\n'
    for s in scenes:
        part+='| '+LABELS[s['id']]+' | '+str(s['native_frames'])+' | '+f"{s['duration_s']:g}"+' 秒 | [MP4]('+s['video']['url']+') |\n'
    part+='\n四条视频均保留全部 20 Hz 源帧，未经裁剪、插值或加速播放；没有额外标题或 HUD。画面包括原生 Tesla 跟车、原生 SUMO GUI、图像处理及点云处理。右上角 SUMO 均是**录制后的同步回放**；live-highway 的源录制另有实际官方双向协同证据。图像框使用原生真值，点云来自真实 LiDAR 返回，没有训练检测或驾驶模型。\n\n'
    part+='真实抽样审阅范围、各视频 SHA 与待完成场景见 [Beta 验收摘要](docs/media/beta-public-summary.json)。抽样审阅不代表全片人工观看。\n\n'
    content={root/'README.md':readme[:start]+part+readme[end:],
             root/'docs/media/beta-public-summary.json':json.dumps(public,ensure_ascii=False,indent=2)+'\n'}
    for payload in content.values():
        require(public_clean(payload),'Local machine path in public docs')
    if execute:
        verify_beta(publication_path,receipt_path,beta_spec,root)
        for path,payload in content.items():
            path.parent.mkdir(parents=True,exist_ok=True);atomic_text(path,payload)
    return {'verified_scenes':4,'public_docs_written':execute,'publication_verified':True,
        'six_scene_stable_accepted':False,'current_manifest_promoted':False,
        'tag':proof['tag'],'commit_sha':proof['commit_sha'],'git_or_network_or_cleanup_used':False}


def finalize(publication_path,receipt_path,root=ROOT,execute=False,beta_spec=None):
    if beta_spec is not None:return finalize_beta(publication_path,receipt_path,beta_spec,root,execute)
    root=Path(root).resolve();manifest,receipt,summary,assets=verify_inputs(publication_path,receipt_path,root)
    proof=receipt['publication'];summary['published']=True;summary['status']='published_verified'
    summary['publication']={k:proof[k] for k in ('repository','tag','commit_sha','release_url','published_at','verified_utc')}
    summary['publication_manifest_sha256']=sha(inside(publication_path,root))
    summary['verification_receipt_sha256']=sha(inside(receipt_path,root))
    index=read(root/'docs/media/preview-index.json')
    require(len(index.get('items',[]))==6 and {i['id'] for i in index['items']}==set(IDS),'Invalid six-scene preview index')
    index['status']='published_verified';index['publication']=summary['publication']
    for scene in summary['scenes']:
        media=next(m for m in scene['media'] if m.get('role')=='carla-sumo-four-pane')
        url=assets[media['name']]['url'];scene['verified_video_url']=url
        for recorded in scene['media']:recorded['url']=assets[recorded['name']]['url']
        item=next(i for i in index['items'] if i['id']==scene['id'])
        item.update(video_url=url,sha256=media['sha256'],poster_path=scene['preview']['path'],
                    preview_sha256=scene['preview']['sha256'],release_asset_name=media['name'],
                    native_frames=scene['native_frames'],duration_s=scene['duration_s'])
    readme=render_readme((root/'README.md').read_text(encoding='utf-8'),summary['scenes'],assets,proof)
    gallery='# Verified native v2 previews\n\n[Verified Release]('+proof['release_url']+')\n\n'
    for scene_id in IDS:
        s=next(s for s in summary['scenes'] if s['id']==scene_id)
        gallery+='## '+LABELS[scene_id]+'\n\n!['+LABELS[scene_id]+']('+scene_id+'.gif)\n\n[Full video]('+s['verified_video_url']+')\n\n'
    gallery+='All displayed SUMO GUI panes remain post-capture replay. Live acquisition is independently verified for live-highway only.\n'
    if any(s['id']=='left-turn' and s.get('sumo_replay_qualification') is not None for s in summary['scenes']):
        gallery+='Left-turn retains one unchanged empty background lane field at an invisible connected-endpoint sample. The original strict all-lane FAIL is preserved; all-actor lane membership is false. Ego lane membership, exact source poses/times and native images remain verified.\n'
    if any(s['id']=='roundabout' and s.get('sumo_replay_qualification') is not None for s in summary['scenes']):
        gallery+='Roundabout retains two visible ego endpoint empty lane fields and fourteen invisible background empty lane fields. All-actor and all-ego lane membership are false; the original strict FAIL remains. Exact full source poses/times and original native PNG hashes are verified; independent GUI repeat raster variation is disclosed.\n'
    content={root/'README.md':readme,root/'docs/media/preview-gallery.md':gallery,
             root/'docs/media/preview-index.json':json.dumps(index,ensure_ascii=False,indent=2)+'\n',
             root/'docs/media/accepted-v2-summary.json':json.dumps(summary,ensure_ascii=False,indent=2)+'\n'}
    for payload in content.values():
        require(public_clean(payload),'Local machine path in public docs')
    if execute:
        # Revalidate immutable local inputs immediately before doc writes.
        verify_inputs(publication_path,receipt_path,root)
        for path,payload in content.items():atomic_text(path,payload)
    return {'verified_scenes':6,'public_docs_written':execute,'publication_verified':True,
            'tag':proof['tag'],'commit_sha':proof['commit_sha'],'git_or_network_or_cleanup_used':False}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--publication',type=Path,required=True);p.add_argument('--receipt',type=Path,required=True)
    p.add_argument('--beta-spec',type=Path,help='Four-scene beta only; never promotes current or weakens six-scene stable gates')
    p.add_argument('--execute',action='store_true');a=p.parse_args()
    print(json.dumps(finalize(a.publication,a.receipt,execute=a.execute,beta_spec=a.beta_spec)),flush=True)


if __name__=='__main__':main()
