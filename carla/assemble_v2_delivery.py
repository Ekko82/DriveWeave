"""Validate six immutable recordings and assemble release media without CARLA.

Default mode only validates existing evidence. --execute creates byte-identical
ASCII MP4 copies and native-source GIFs. --promote-current separately archives
the exact previous manifest and atomically promotes the fully checked version.
No Git, network publication, simulation, training, or cleanup is performed.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import importlib.util
import json
import math
import os
import re
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
IDS = ('highway', 'ramp-entry', 'ramp-exit', 'left-turn', 'roundabout', 'live-highway')
REPORT_KEYS = ('execution', 'motion_audit', 'action_audit', 'sensor_audit',
               'native_video_manifest', 'native_video_audit', 'sumo_manifest',
               'sumo_audit', 'pane_manifest', 'pane_audit', 'manual_review')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def rows(path):
    with Path(path).open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def inside(value, root=ROOT):
    root = Path(root).resolve()
    path = Path(value)
    if not path.is_absolute():
        path = root / path
    path = path.absolute()
    require(path.is_relative_to(root), 'Delivery input/output escaped the project')
    for parent in (path, *path.parents):
        require(not parent.is_symlink() and not (parent.exists() and
                getattr(parent.lstat(), 'st_file_attributes', 0) & 0x400),
                'Symlink or reparse point refused')
        if parent == root:
            break
    require(path.resolve().is_relative_to(root), 'Resolved path escaped the project')
    return path.resolve()


def atomic(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    with temp.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    temp.replace(path)


def clean_report(report, success_key=None):
    require(not report.get('failures') and not report.get('missing_evidence')
            and not report.get('failed_checks') and not report.get('failures_count'),
            'A required audit contains failures or missing evidence')
    if success_key:
        require(report.get(success_key) is True, 'A required audit did not pass')
    elif 'overall_status' in report:
        require(report['overall_status'] == 'automated_checks_passed_manual_review_needed',
                'A required sensor/video audit did not pass')
    checks = report.get('checks')
    if checks is not None:
        require(isinstance(checks, dict) and bool(checks) and
                all(value is True for value in checks.values()), 'Incomplete audit checks')


def manual_gate(review, scene_id, ids, source_hashes, pane_path, pane, pane_audit, root=ROOT):
    require(review.get('schema') == 'native_media_manual_review_v2' and
            review.get('scene_id') == scene_id and review.get('accepted') is True,
            'Each scene needs its own actual v2 manual review')
    require(isinstance(review.get('reviewer'), str) and review['reviewer'].strip(),
            'Manual reviewer is missing')
    date = datetime.fromisoformat(review.get('reviewed_utc', ''))
    require(date.tzinfo is not None, 'Manual review needs a timezone-aware timestamp')
    expected = dict(source_hashes, pane_manifest_sha256=sha(pane_path),
                    pane_video_sha256=pane['outputs'][0]['sha256'])
    require(review.get('bindings') == expected, 'Manual review belongs to different source/media')
    presentation = review.get('presentation', {})
    require(all(presentation.get(k) is True for k in
                ('native_tesla_model_retained', 'native_SUMO_viewport_retained', 'four_panes_legible'))
            and presentation.get('added_titles_or_HUD') is False,
            'Manual presentation review is incomplete')
    require(isinstance(review.get('full_video_manually_watched'), bool),
            'Disclose whether the full video was watched')
    audited = {item['sample']: item for item in pane_audit['samples']}
    samples = review.get('samples', [])
    indices = [item['sample'] for item in samples]
    require(len(set(indices)) >= 3 and len(indices) == len(set(indices)) and
            0 in indices and len(ids)-1 in indices and
            any(len(ids)//4 <= index <= 3*len(ids)//4 for index in indices),
            'Manual review must include actual opening, middle and closing samples')
    for item in samples:
        index = item['sample']
        require(index in audited and 0 <= index < len(ids) and item['native_frame'] == ids[index],
                'Manual sample is not an audited native world frame')
        image = inside(item['image'], root)
        require(image == inside(audited[index]['image'], root) and
                sha(image) == item['image_sha256'], 'Manual sample image changed')
        require(isinstance(item.get('observed'), str) and item['observed'].strip(),
                'Record the actual visual observation, not a blanket approval')
    return {'accepted': True, 'reviewed_sample_indices': indices,
            'reviewed_native_frames': [ids[i] for i in indices],
            'full_video_manually_watched': review['full_video_manually_watched']}


def scene_gate(spec, root=ROOT):
    root = Path(root).resolve()
    scene_id = spec['id']
    require(scene_id in IDS, 'Unknown scene ID')
    run = inside(spec['run'], root)
    files = {key: inside(spec[key], root) for key in REPORT_KEYS}
    reports = {key: read(path) for key, path in files.items()}
    raw = read(run / 'manifest.json'); frames = rows(run / 'frames.jsonl')
    ids = [r['frame'] for r in frames]
    require(bool(ids) and all(b == a+1 for a, b in zip(ids, ids[1:])),
            'Actual source world frames must be nonempty and consecutive')
    hashes = {'manifest.json': sha(run/'manifest.json'), 'frames.jsonl': sha(run/'frames.jsonl')}
    fps = raw['fps']
    require(fps == 20 and raw.get('status') == 'complete' and
            raw.get('native_cleanup_complete') is True and raw.get('collision_count') == 0 and
            raw.get('controller_plan_completed') is True and
            raw.get('duration_s') == len(ids)/fps,
            'Native capture, cleanup, collision or full-plan gate failed')
    execution = reports['execution']
    require(execution.get('returncode') == 0 and
            execution.get('native_process_returncode') == 0 and
            inside(execution['capture_run'], root) == run and
            execution.get('manifest_sha256') == hashes['manifest.json'] and
            execution.get('frames_jsonl_sha256') == hashes['frames.jsonl'],
            'Actual external process exit/source binding failed')
    for role, key in (('motion_audit', 'motion_quality_validated'),
                      ('action_audit', 'scenario_actions_validated')):
        report = reports[role]; clean_report(report, key)
        require(inside(report['run'], root) == run and
                all(report['input_sha256'].get(k) == v for k, v in hashes.items()),
                'Motion/action report refers to different actual source')
    native = reports['native_video_manifest']; sensor = reports['sensor_audit']
    clean_report(sensor); video = reports['native_video_audit']; clean_report(video)
    processed = inside(native['processed_run'], root)
    processing = processed/'processing-manifest.json'
    require(native['native_frame_ids'] == ids and inside(native['source_run'], root) == run and
            native['capture_manifest_sha256'] == hashes['manifest.json'] and
            native['processing_manifest_sha256'] == sha(processing), 'Native video/source binding failed')
    require(sensor['capture_manifest_sha256'] == hashes['manifest.json'] and
            sensor['source_frames_jsonl_sha256'] == hashes['frames.jsonl'] and
            sensor['processed_manifest_sha256'] == sha(processing) and
            sensor['execution_sha256'] == sha(files['execution']) and
            sensor['native_process_exit0_verified'] is True and sensor['captured_frames'] == len(ids),
            'Sensor audit has stale source, processing or process evidence')
    require(video['video_manifest_sha256'] == sha(files['native_video_manifest']) and
            video['native_frame_ids'] == ids, 'Native video audit is stale or cropped')
    pane = reports['pane_manifest']; pane_audit = reports['pane_audit']
    clean_report(pane_audit, 'automated_checks_passed')
    sumo = reports['sumo_manifest']; sync = reports['sumo_audit']; clean_report(sync, 'passed')
    qualification = None
    if sync.get('audit_type') == 'qualified_single_background_endpoint_native_SUMO_replay_mapping':
        require(scene_id == 'left-turn', 'The approved single-boundary qualification is specific to left-turn')
        module_spec = importlib.util.spec_from_file_location('sumo_boundary_qualification',
                         Path(__file__).with_name('audit_sumo_sync_boundary_qualified.py'))
        qualifier = importlib.util.module_from_spec(module_spec); module_spec.loader.exec_module(qualifier)
        qualification = qualifier.revalidate_report(sync)
    elif sync.get('audit_type') == 'qualified_source_pose_native_GUI_roundabout_lane_metadata_limitations':
        require(scene_id == 'roundabout', 'The approved roundabout metadata scope cannot apply to other scenes')
        module_spec = importlib.util.spec_from_file_location('sumo_round_qualification',
                         Path(__file__).with_name('audit_sumo_sync_round_qualified.py'))
        qualifier = importlib.util.module_from_spec(module_spec); module_spec.loader.exec_module(qualifier)
        qualification = qualifier.revalidate_report(sync)
    else:
        require(sync.get('all_actor_lane_membership') is not False,
                'An all-lane failure needs the explicitly validated qualified scope')
    require(sumo.get('status') == 'complete' and sumo.get('mode') == 'post_capture_synchronized_replay'
            and sumo.get('sumo_process_exit_code') == 0 and sumo.get('native_gui_only') is True
            and sumo.get('carla_connected_or_ticked') is False
            and sumo.get('two_way_live_control_claimed') is False,
            'SUMO pane must be a successful native GUI post-capture replay')
    mapping_file = files['sumo_manifest'].parent/'mapping.jsonl'
    network = inside(sumo['network'], root)
    bindings = {'replay_manifest': sha(files['sumo_manifest']), 'mapping': sha(mapping_file),
                'raw_manifest': hashes['manifest.json'], 'raw_frames': hashes['frames.jsonl'],
                'raw_execution': sha(files['execution']), 'raw_xodr': sha(run/'map.xodr'),
                'network': sha(network)}
    require(all(sync['input_sha256'].get(k) == v for k, v in bindings.items()) and
            sync['samples_checked'] == len(ids) and sync['native_pngs_fully_decoded'] == len(ids),
            'SUMO all-frame mapping audit is stale or has a source cut')
    require(sumo['mapping_sha256'] == bindings['mapping'] and
            sumo['source_manifest_sha256'] == hashes['manifest.json'] and
            sumo['source_frames_jsonl_sha256'] == hashes['frames.jsonl'] and
            sumo['source_execution_sha256'] == bindings['raw_execution'] and
            sumo['network_sha256'] == bindings['network'], 'SUMO manifest source binding failed')
    mapped = rows(mapping_file)
    require([r['source_frame'] for r in mapped] == ids, 'SUMO mapping source sequence is cropped')
    for raw_row, mapped_row in zip(frames, mapped):
        png = inside(mapped_row['native_sumo_png'], root)
        require(mapped_row['source_sample'] == raw_row['sample'] and
                mapped_row['source_timestamp_s'] == raw_row['timestamp'] and
                mapped_row['display_time_s'] == raw_row['time_s'] and
                png.name == f"{raw_row['frame']:08d}.png" and
                sha(png) == mapped_row['native_sumo_png_sha256'],
                'A SUMO screenshot, source timestamp or native frame changed')
    require(pane['native_frame_ids'] == ids and inside(pane['source_run'], root) == run and
            pane['native_video_manifest_sha256'] == sha(files['native_video_manifest']) and
            pane['sumo_manifest_sha256'] == bindings['replay_manifest'] and
            pane['sumo_mode'] == 'post_capture_synchronized_replay' and
            pane['source_folders'] == [str(run/'chase'), str(files['sumo_manifest'].parent/'frames_native_sumo'),
                                       str(processed/'image-processed'), str(processed/'lidar-3d')],
            'Four-pane sources or frame sequence differ from accepted recordings')
    require(pane_audit['manifest_sha256'] == sha(files['pane_manifest']) and
            pane_audit['decoded_frames'] == len(ids) and pane_audit['native_frames_expected'] == len(ids),
            'Four-pane whole-video decoding audit is stale or cropped')
    for manifest in (native, pane):
        require(manifest.get('added_text') is False and manifest.get('interpolated_frames') is False and
                manifest.get('playback_speed') == 1 and manifest.get('synthetic_sensor_data') is False,
                'Unexpected synthetic sensor/presentation change')
        for item in manifest['outputs']:
            path = inside(item['file'], root)
            require(path.suffix.lower() == '.mp4' and path.stat().st_size == item['bytes'] and
                    sha(path) == item['sha256'] and item['frames'] == len(ids) and
                    item['fps'] == fps and item['duration_s'] == len(ids)/fps,
                    'Accepted MP4 bytes, SHA or full frame range changed')
    require(len(pane['outputs']) == 1 and pane['outputs'][0]['size'] == [1920, 1080],
            'Expected one full 1080p four-pane video')
    live = None
    if scene_id == 'live-highway':
        live_path = inside(spec['live_audit'], root); live = read(live_path)
        clean_report(live, 'live_joint_simulation_verified')
        require(inside(live['run'], root) == run and live['frames'] == len(ids) and
                live['execution_sha256'] == sha(files['execution']) and
                all(live['input_sha256'].get(k) == v for k, v in hashes.items()) and
                live['input_sha256'].get('sumo-sync.jsonl') == sha(run/'sumo-sync.jsonl'),
                'Live two-way acquisition evidence has a stale source or phase log')
        files['live_audit'] = live_path
    manual = manual_gate(reports['manual_review'], scene_id, ids,
                         {'capture_manifest_sha256': hashes['manifest.json'],
                          'source_frames_jsonl_sha256': hashes['frames.jsonl']},
                         files['pane_manifest'], pane, pane_audit, root)
    return {'id': scene_id, 'run': run, 'source_run': str(run), 'processed_run': str(processed),
            'native_frame_ids': ids, 'native_frames': len(ids), 'fps': fps, 'duration_s': len(ids)/fps,
            'source_sha256': dict(hashes, **{'map.xodr': bindings['raw_xodr']}),
            'files': files, 'reports': reports, 'native': native, 'pane': pane,
            'manual_review': manual, 'live_joint_simulation_verified': live is not None,
            'requested_action_verified': True, 'full_internal_plan_completed': True,
            'sumo_view_mode': 'post_capture_synchronized_replay',
            'sumo_replay_qualification': qualification}


def validate_config(config, root=ROOT):
    require(config.get('schema_version') == 1 and config.get('version') == 'v2',
            'Expected the explicit v2 assembly configuration')
    specs = config.get('scenes', [])
    require(len(specs) == len(IDS) and {s['id'] for s in specs} == set(IDS),
            'Exactly all six independent v2 scenes are required')
    return [scene_gate(next(s for s in specs if s['id'] == scene_id), root) for scene_id in IDS]


def gif_from_video(video, destination, start_sample, fps, native_ids):
    # Use actual decoded MP4 frames, selecting every second 20 Hz source frame.
    # 100 ms/frame preserves the original six-second interval; no interpolation.
    import imageio_ffmpeg
    import numpy as np
    from PIL import Image
    count = int(6*fps)
    require(fps == 20 and start_sample >= 0 and start_sample+count <= len(native_ids),
            'GIF preview needs six actual source seconds')
    reader = imageio_ffmpeg.read_frames(str(video), pix_fmt='rgb24'); meta = next(reader)
    w, h = meta['size']; images = []
    try:
        for index, payload in enumerate(reader):
            if index >= start_sample+count:
                break
            if index >= start_sample and (index-start_sample) % 2 == 0:
                im = Image.fromarray(np.frombuffer(payload, np.uint8).reshape(h,w,3))
                images.append(im.resize((480,270), Image.Resampling.LANCZOS).quantize(colors=128))
    finally:
        reader.close()
    require(len(images) == 60, 'Incomplete GIF source decode')
    images[0].save(destination, save_all=True, append_images=images[1:],
                   duration=100, loop=0, disposal=2, optimize=False)
    with Image.open(destination) as gif:
        total = 0
        for index in range(gif.n_frames):
            gif.seek(index); gif.load(); total += gif.info.get('duration',0)
        require(total == 6000 and gif.size == (480,270), 'GIF timebase or dimensions changed')
    require(destination.stat().st_size <= 4*1024*1024, 'README preview exceeds publication size limit')
    return {'file': destination.name, 'bytes': destination.stat().st_size, 'sha256': sha(destination),
            'duration_s': 6, 'fps': 10, 'size': [480,270], 'interpolated_frames': False,
            'source_sample_start': start_sample, 'source_sample_end_exclusive': start_sample+count,
            'selected_native_frame_ids': native_ids[start_sample:start_sample+count:2],
            'source_video_sha256': sha(video)}


def promote(delivery, current, root=ROOT):
    current = inside(current, root)
    require(current == Path(root).resolve()/'carla/output/delivery-manifest.json',
            'Promotion target must be the maintenance-protected current manifest')
    if current.exists():
        payload = current.read_bytes(); digest = hashlib.sha256(payload).hexdigest()
        history = Path(root)/'verification/history'/('delivery-before-v2-'+digest+'.json')
        history.parent.mkdir(parents=True, exist_ok=True)
        if history.exists():
            require(history.read_bytes() == payload, 'Conflicting immutable history record')
        else:
            with history.open('xb') as stream:
                stream.write(payload); stream.flush(); os.fsync(stream.fileno())
        require(sha(history) == digest, 'Previous accepted manifest backup failed')
    # History is deliberately absent from current's path closure. It remains a
    # permanent audit record but cannot keep obsolete binary caches accepted.
    atomic(current, delivery)


def execute(config, accepted, root=ROOT, promote_current=False):
    root = Path(root).resolve()
    release = inside(config.get('release_dir', 'carla/release-v2'), root)
    require(not release.exists(), 'Never overwrite an existing release-media directory')
    require(release.parent.is_dir(), 'Release parent must already exist')
    previews = root/'docs/media'
    require(previews.is_dir() and not (previews/'accepted-v2-summary.json').exists(),
            'Never overwrite an existing accepted public summary')
    for scene in accepted:
        require(not (previews/(scene['id']+'.gif')).exists(), 'Never overwrite an accepted preview')
    stage = release.with_name(release.name+'.staging-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f'))
    stage.mkdir()
    public_scenes, local_scenes, release_outputs = [], [], []
    for scene in accepted:
        outputs = [*scene['native']['outputs'], scene['pane']['outputs'][0]]
        media = []
        for number, item in enumerate(outputs):
            role = 'carla-sumo-four-pane' if number == len(outputs)-1 else ('native-'+str(number+1))
            dest = stage/(scene['id']+'-'+role+'.mp4')
            shutil.copyfile(inside(item['file'],root),dest)
            require(dest.stat().st_size == item['bytes'] and sha(dest) == item['sha256'],
                    'Byte-identical release copy failed')
            final = release/dest.name
            record = {k:item[k] for k in ('bytes','sha256','frames','fps','duration_s','size')}
            record.update({'name':dest.name,'path':final.relative_to(root).as_posix(),'role':role})
            media.append(record); release_outputs.append(record)
        spec = next(s for s in config['scenes'] if s['id'] == scene['id'])
        preview = gif_from_video(inside(scene['pane']['outputs'][0]['file'],root),
                                 stage/(scene['id']+'.gif'), spec.get('preview_start_sample',0),
                                 scene['fps'], scene['native_frame_ids'])
        preview['path'] = 'docs/media/'+preview.pop('file')
        report_records = {k:{'path':p.relative_to(root).as_posix(),'sha256':sha(p)}
                          for k,p in scene['files'].items()}
        local = {k:scene[k] for k in ('id','source_run','processed_run','native_frame_ids','native_frames',
                  'fps','duration_s','source_sha256','manual_review','live_joint_simulation_verified',
                  'requested_action_verified','full_internal_plan_completed','sumo_view_mode',
                  'sumo_replay_qualification')}
        local.update({'verification_reports':list(report_records.values()),'release_outputs':media,
                      'native_manifest':str(scene['files']['native_video_manifest']),
                      'sumo_manifest':str(scene['files']['sumo_manifest']),
                      'pane_manifest':str(scene['files']['pane_manifest']),
                      'sensor_video':str(inside(scene['pane']['outputs'][0]['file'],root))})
        local_scenes.append(local)
        public_scenes.append({'id':scene['id'],'native_frames':scene['native_frames'],'fps':scene['fps'],
              'duration_s':scene['duration_s'],'first_native_frame':scene['native_frame_ids'][0],
              'last_native_frame':scene['native_frame_ids'][-1], 'source_sha256':scene['source_sha256'],
              'source_acquisition':'official_two_way_live_cosimulation' if scene['live_joint_simulation_verified'] else 'native_CARLA_physics',
              'displayed_SUMO_viewport_mode':'post_capture_synchronized_replay',
              'source_frame_cut':False,'added_text_or_HUD':False,'trained_model':False,
              'requested_action_verified':True,'full_internal_plan_completed':True,
              'manual_review':scene['manual_review'],'audit_sha256':{k:v['sha256'] for k,v in report_records.items()},
              'media':media,'preview':preview,
              'sumo_replay_qualification':scene['sumo_replay_qualification']})
    public = {'schema_version':1,'version':'v2','status':'accepted_local_media_publication_pending',
        'scenes':public_scenes,'trained_policy':False,'image_boxes_are_native_ground_truth':True,
        'pointcloud_is_native_sensor_processing':True,'published':False,
        'limitations':['SUMO GUI panes are post-capture replay, including the independently verified live source.',
                       'Simulator-label boxes are ground truth assisted image processing, not learned detection.',
                       'Vehicle separation checks sample recorded world frames, not continuous-time safety guarantees.',
                       'Manual sample review is disclosed separately from full automated video decoding.']}
    if any(s['id']=='left-turn' and s['sumo_replay_qualification'] is not None for s in public_scenes):
        public['limitations'].append('Left-turn SUMO replay retains one empty lane field for a background vehicle at a connected road endpoint (sample 501, native frame 1200430, 0.058394 m from the endpoint). Its complete footprint is outside the actual native GUI viewport. All ego lane fields and all source poses, times and images remain verified; all-actor lane membership is explicitly false. The original strict FAIL is preserved and no lane or pose was filled or modified.')
    if any(s['id']=='roundabout' and s['sumo_replay_qualification'] is not None for s in public_scenes):
        public['limitations'].append('Roundabout SUMO replay retains 16 empty lane fields: two visible ego samples at connected normal/internal endpoints within 0.1 m and 14 consecutive samples (41-54) of background road44/-5 outside the actual viewport. All-actor and all-ego lane membership are false, while exact full source poses/times and original PNG hashes pass. The original strict FAIL remains; lanes and poses were not repaired. Independent native GUI repeats match actual viewport and all poses and differ by at most 4 pixels; measured PSNR is disclosed, not claimed identical bytes. This is post-capture GUI representation, not a new closed-loop roundabout run.')
    # Every public field is constructed explicitly. Do not recursively publish
    # arbitrary audit details or free-form manual notes containing local paths.
    blob = json.dumps(public,ensure_ascii=False)
    require(not re.search(r'(?i)[a-z]:[\\/]|/(?:Users|home)/',blob), 'Private path in public summary')
    index = {'schema_version':1,'version':'v2','status':'accepted_local_media_publication_pending',
         'no_placeholder_media':True,'items':[{'id':s['id'],'video_url':None,
              'poster_path':s['preview']['path'],'sha256':s['media'][-1]['sha256'],
              'release_asset_name':s['media'][-1]['name'],'preview_sha256':s['preview']['sha256']}
             for s in public_scenes],
         'update_rule':'Add only actual verified release URLs after upload; GUI replay is not live acquisition.'}
    gallery = '# Verified v2 native previews\n\n'
    gallery += ('Each GIF shows six seconds from the accepted full video at its original speed. '
                'All SUMO GUI panes are post-capture replay. The live-highway source separately '
                'has audited official two-way live acquisition.\n\n')
    for scene in public_scenes:
        gallery += ('## '+scene['id']+'\n\n!['+scene['id']+']('+scene['id']+'.gif)\n\n'
                    +str(scene['native_frames'])+' source frames, '+str(scene['duration_s'])+' s. '
                    'Full release asset: `'+scene['media'][-1]['name']+'`.\n\n')
        if scene['id']=='left-turn' and scene['sumo_replay_qualification'] is not None:
            gallery += ('SUMO representation qualification: one invisible background endpoint sample has an '
                        'unchanged empty lane field. The original strict all-lane audit remains FAIL; '
                        'ego lanes, exact poses/times and native images are verified.\n\n')
        if scene['id']=='roundabout' and scene['sumo_replay_qualification'] is not None:
            gallery += ('SUMO representation qualification: two visible ego endpoint lane fields and 14 invisible '
                        'background lane fields remain empty. All-actor and all-ego lane membership are false; '
                        'the original strict FAIL remains. Exact poses/times and all native source PNG hashes are verified.\n\n')
    gallery += ('Release URLs remain pending until actual upload and remote SHA verification. '
                'See preview-index.json and accepted-v2-summary.json for hashes and scope.\n')
    atomic(stage/'accepted-v2-summary.json',public)
    atomic(stage/'preview-index.json',index)
    (stage/'preview-gallery.md').write_text(gallery,encoding='utf-8')
    notes = '# Accepted local v2 media scope\n\n' + '\n\n'.join(public['limitations']) + '\n'
    (stage/'release-notes.md').write_text(notes,encoding='utf-8')
    stage.rename(release)
    for scene in public_scenes:
        shutil.copyfile(release/(scene['id']+'.gif'),previews/(scene['id']+'.gif'))
    atomic(previews/'accepted-v2-summary.json',public)
    atomic(previews/'preview-index.json',index)
    (previews/'preview-gallery.md').write_text(gallery,encoding='utf-8')
    delivery={'schema_version':2,'version':'v2','final_delivery_accepted':True,'scenes':local_scenes,
         'release_outputs':release_outputs,'all_requested_core_actions_verified':True,
         'all_extra_internal_demonstration_events_passed':True,
         'sumo_panes_are_synchronized_replay':True,'added_text':False,'trained_policy':False,
         'published':False,'media_uploaded':False,'public_summary':'docs/media/accepted-v2-summary.json'}
    atomic(release/'delivery-manifest.local.json',delivery)
    if promote_current:
        promote(delivery,root/'carla/output/delivery-manifest.json',root)
    return {'accepted_scenes':len(accepted),'release_media_files':len(release_outputs),
            'current_manifest_promoted':promote_current,'published':False}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--promote-current',action='store_true')
    args=p.parse_args()
    require(not args.promote_current or args.execute, '--promote-current requires --execute')
    config=read(inside(args.config)); accepted=validate_config(config)
    result=execute(config,accepted,promote_current=args.promote_current) if args.execute else {
        'all_six_scene_gates_passed':True,'read_only':True,'current_manifest_promoted':False}
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    main()
