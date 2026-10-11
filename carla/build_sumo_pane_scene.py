"""Build one source-gated native SUMO replay and a complete four-pane video.

Default mode is a read-only preflight. --execute runs the installed official
offline converter, SUMO-GUI replay, mapping auditor, encoder and video auditor.
It never creates a CARLA Client, ticks CARLA, publishes, promotes a delivery,
cleans assets or approves a manual review. Failed outputs are never replaced.
"""
from datetime import datetime,timezone
from pathlib import Path
from types import SimpleNamespace
import argparse
import json
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(BASE))
from runtime_config import RuntimeConfig,hidden_process_options
from assemble_v2_delivery import inside,read,rows,sha,require,clean_report
from run_sumo_replay_logged import source_gates

TOWNS={'left-turn':'Town05','roundabout':'Town03'}
SOURCE_ROLES=('execution','motion_audit','action_audit','sensor_audit',
              'native_video_manifest','native_video_audit')
TARGET_ROLES=('network_dir','replay_dir','pane_dir','sumo_process_file',
              'sumo_audit','pane_audit','pipeline_receipt')


def source_preflight(spec,root=ROOT):
    """Recheck current full-source evidence before any simulator process starts."""
    scene_id=spec.get('id')
    require(scene_id in TOWNS,'This dispatcher handles only actual left-turn/roundabout v2 sources')
    run=inside(spec['run'],root)
    paths={key:inside(spec[key],root) for key in SOURCE_ROLES}
    reports={key:read(path) for key,path in paths.items()}
    manifest=read(run/'manifest.json');frames=rows(run/'frames.jsonl')
    ids=[frame['frame'] for frame in frames]
    require(bool(ids) and len(ids)>=120 and all(b==a+1 for a,b in zip(ids,ids[1:]))
            and [r['sample'] for r in frames]==list(range(len(frames))),
            'Source must contain a full consecutive native frame/sample sequence')
    require(manifest.get('map','').split('/')[-1]==TOWNS[scene_id],
            'Scene must use its actually recorded native map')
    require(manifest.get('fps')==20 and manifest.get('duration_s')==len(ids)/20
            and manifest.get('status')=='complete' and manifest.get('native_cleanup_complete') is True
            and manifest.get('controller_plan_completed') is True and manifest.get('collision_count')==0,
            'Capture, cleanup, collision, duration or complete-plan source gate failed')
    execution=reports['execution']
    require(execution.get('returncode')==0 and execution.get('native_process_returncode')==0
            and inside(execution['capture_run'],root)==run
            and execution.get('manifest_sha256')==sha(run/'manifest.json')
            and execution.get('frames_jsonl_sha256')==sha(run/'frames.jsonl'),
            'Actual source execution or immutable raw hashes failed')
    for role,key in [('motion_audit','motion_quality_validated'),
                     ('action_audit','scenario_actions_validated')]:
        clean_report(reports[role],key)
    clean_report(reports['sensor_audit']);clean_report(reports['native_video_audit'])
    gates=source_gates(SimpleNamespace(run=run,motion_audit=paths['motion_audit'],
        action_audit=paths['action_audit'],sensor_audit=paths['sensor_audit'],
        video_audit=paths['native_video_audit'],native_video_manifest=paths['native_video_manifest'],
        defer_sensor_video=False),frames)
    native=reports['native_video_manifest'];sensor=reports['sensor_audit'];video=reports['native_video_audit']
    processed=inside(native['processed_run'],root)
    require(native['processing_manifest_sha256']==sha(processed/'processing-manifest.json')
            and sensor['processed_manifest_sha256']==sha(processed/'processing-manifest.json')
            and sensor['execution_sha256']==sha(paths['execution'])
            and video['native_frame_ids']==ids,'Stale processing, native video or execution evidence')
    for output in native['outputs']:
        path=inside(output['file'],root)
        require(path.stat().st_size==output['bytes'] and sha(path)==output['sha256']
                and output['frames']==len(ids) and output['fps']==20
                and output['duration_s']==len(ids)/20,'Native video bytes or full frame range changed')
    require(native.get('added_text') is False and native.get('interpolated_frames') is False
            and native.get('playback_speed')==1 and native.get('synthetic_sensor_data') is False,
            'Source video must preserve native sensor/presentation evidence')
    require(bool(native.get('outputs')),'Native video manifest must contain actual audited outputs')
    targets={key:inside(spec[key],root) for key in TARGET_ROLES}
    namespaces={'network_dir':'carla/sumo-network-v2','replay_dir':'carla/sumo-replay-v2',
                'pane_dir':'carla/output-sumo-v2','sumo_process_file':'carla/sumo-replay-v2',
                'sumo_audit':'carla/verification','pane_audit':'carla/verification',
                'pipeline_receipt':'carla/verification'}
    require(len(set(targets.values()))==len(targets),'Output paths must be independent')
    for key,path in targets.items():
        namespace=Path(root).resolve()/namespaces[key]
        require(path!=namespace and path.is_relative_to(namespace),'Output escaped its scene namespace')
        require(scene_id in path.name,'Use a scene-specific new output name')
        require(not path.exists(),'Never replace an existing replay, media, audit or process record')
    for key in ('sumo_process_file','sumo_audit','pane_audit','pipeline_receipt'):
        require(targets[key].suffix=='.json','Process/audit outputs must be explicit JSON files')
        require(not any(targets[key].is_relative_to(targets[d]) for d in
            ('network_dir','replay_dir','pane_dir')),'Process/audit records must be outside newly created media directories')
    manual=inside(spec['manual_review'],root)
    require(manual.suffix=='.json' and manual.is_relative_to(Path(root).resolve()/'verification')
            and scene_id in manual.name,'Manual review must have its own local verification path')
    require(not manual.exists(),'Do not reuse an old manual review for newly encoded media')
    name=scene_id+'-v2'
    return {'id':scene_id,'town':TOWNS[scene_id],'run':run,'frames':frames,'ids':ids,
        'paths':paths,'targets':targets,'manual_review':manual,'name':name,
        'source_gate_reports':gates,'source_sha256':{p:sha(run/p) for p in
            ['manifest.json','frames.jsonl','map.xodr']}}


def command_plan(context):
    c=context;t=c['targets'];p=c['paths'];network=t['network_dir']/c['town']/'official.net.xml'
    replay_manifest=t['replay_dir']/'manifest.json'
    pane_manifest=t['pane_dir']/(c['name']+'-CARLA与SUMO视频清单.json')
    return [
        ('actual_raw_xodr_conversion',[sys.executable,str(BASE/'convert_native_map.py'),
            '--xodr',str(c['run']/'map.xodr'),'--map',c['town'],'--out',str(t['network_dir'])]),
        ('full_source_native_SUMO_GUI_replay',[sys.executable,'-u',str(BASE/'run_sumo_replay_logged.py'),
            '--run',str(c['run']),'--network',str(network),'--out',str(t['replay_dir']),
            '--process-file',str(t['sumo_process_file']),'--motion-audit',str(p['motion_audit']),
            '--action-audit',str(p['action_audit']),'--sensor-audit',str(p['sensor_audit']),
            '--video-audit',str(p['native_video_audit']),'--native-video-manifest',str(p['native_video_manifest'])]),
        ('all_frame_SUMO_mapping_audit',[sys.executable,str(BASE/'audit_sumo_sync_replay.py'),
            '--replay',str(t['replay_dir']),'--process-file',str(t['sumo_process_file']),
            '--out',str(t['sumo_audit'])]),
        ('full_source_four_pane_encode',[sys.executable,str(BASE/'encode_sumo_panes.py'),
            '--native-manifest',str(p['native_video_manifest']),'--sumo-manifest',str(replay_manifest),
            '--sumo-frames',str(t['replay_dir']/'frames_native_sumo'),
            '--out',str(t['pane_dir']),'--name',c['name']]),
        ('whole_video_decode_and_source_pane_audit',[sys.executable,str(BASE/'audit_sumo_pane_video.py'),
            '--manifest',str(pane_manifest),'--out',str(t['pane_audit'])])]


def converted_network_gate(context,runtime):
    c=context;report=read(c['targets']['network_dir']/'investigation.json')
    map_record=next(m for m in report['maps'] if m['town']==c['town'])
    network=c['targets']['network_dir']/c['town']/'official.net.xml'
    attempt=next(a for a in map_record['attempts'] if a['kind']=='official-carla')
    require(Path(map_record['source_xodr']).resolve()==c['run']/'map.xodr'
            and map_record['source_xodr_sha256']==c['source_sha256']['map.xodr']
            and sha(c['run']/'map.xodr')==c['source_sha256']['map.xodr'],
            'Converted network must originate from exact actual raw XODR bytes')
    require(attempt['actual_returncode']==0 and attempt['network']['sha256']==sha(network)
            and report['official_converter_sha256']==sha(runtime.carla_sumo_root/'util/netconvert_carla.py'),
            'Official conversion, network or converter hash failed')
    return network


def post_step_gate(stage,context,runtime):
    c=context;t=c['targets']
    if stage=='actual_raw_xodr_conversion':converted_network_gate(c,runtime)
    elif stage=='all_frame_SUMO_mapping_audit':
        report=read(t['sumo_audit']);clean_report(report,'passed')
        require(report['samples_checked']==len(c['ids']) and report['native_pngs_fully_decoded']==len(c['ids'])
                and report['first_source_frame']==c['ids'][0] and report['last_source_frame']==c['ids'][-1]
                and report['native_png_dimensions']==[[1280,720]],'SUMO mapping must cover every native source frame')
    elif stage=='whole_video_decode_and_source_pane_audit':
        report=read(t['pane_audit']);clean_report(report,'automated_checks_passed')
        require(report['decoded_frames']==len(c['ids']) and report['native_frames_expected']==len(c['ids']),
                'Every full four-pane video frame must be decoded and audited')


def atomic(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name(path.name+'.tmp')
    with temporary.open('w',encoding='utf-8') as stream:
        json.dump(value,stream,indent=2,ensure_ascii=False,allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    temporary.replace(path)


def execute(context,runtime):
    receipt=context['targets']['pipeline_receipt']
    require(not receipt.exists(),'Never replace a pipeline execution receipt')
    record={'schema_version':1,'scene':context['id'],'status':'running',
        'source_run':str(context['run']),'source_sha256':context['source_sha256'],
        'source_gate_reports':context['source_gate_reports'],'steps':[],
        'mode':'post_capture_synchronized_replay','CARLA_connected_or_ticked':False,
        'publication_or_cleanup_performed':False,'manual_review_approved':False}
    atomic(receipt,record)
    try:
        for stage,command in command_plan(context):
            log=receipt.with_name(receipt.stem+'-'+stage+'.log')
            require(not log.exists(),'Never replace a previous pipeline process log')
            started=datetime.now(timezone.utc).isoformat()
            print(json.dumps({'scene':context['id'],'stage':stage,'status':'starting'}),flush=True)
            with log.open('xb') as stream:
                result=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,
                    check=False,**hidden_process_options())
            step={'stage':stage,'actual_returncode':result.returncode,'command':command,
                'started_utc':started,'finished_utc':datetime.now(timezone.utc).isoformat(),
                'log':str(log),'log_sha256':sha(log)}
            record['steps'].append(step);atomic(receipt,record)
            require(result.returncode==0,'Pipeline stage exited nonzero: '+stage)
            post_step_gate(stage,context,runtime)
        pane_path=context['targets']['pane_dir']/(context['name']+'-CARLA与SUMO视频清单.json')
        pane=read(pane_path)
        require(pane['native_frame_ids']==context['ids'] and pane['sumo_mode']=='post_capture_synchronized_replay'
                and pane['added_text'] is False and pane['interpolated_frames'] is False
                and pane['playback_speed']==1,'Four-pane manifest must preserve every native frame')
        require(len(pane['outputs'])==1,'Expected exactly one complete four-pane video')
        video=pane['outputs'][0];path=inside(video['file'])
        require(path.stat().st_size==video['bytes'] and sha(path)==video['sha256']
                and video['frames']==len(context['ids']) and video['fps']==20
                and video['duration_s']==len(context['ids'])/20,'Final actual media hash/duration differs')
        record.update({'status':'automated_checks_passed_manual_review_needed',
            'full_source_frame_count':len(context['ids']),'first_source_frame':context['ids'][0],
            'last_source_frame':context['ids'][-1],'pane_manifest':str(pane_path),
            'pane_manifest_sha256':sha(pane_path),'video':video,
            'manual_review_pending':True,'expected_manual_review_path':str(context['manual_review'])})
        atomic(receipt,record)
        print(json.dumps({'scene':context['id'],'status':record['status'],'media_sha256':video['sha256'],
            'source_frames':len(context['ids']),'manual_review_approved':False}),flush=True)
        return record
    except Exception:
        record.update({'status':'failed','failed_stage':stage if 'stage' in locals() else None,
            'manual_review_pending':True,'completed_media_acceptance_claimed':False})
        atomic(receipt,record)
        raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec',type=Path,required=True,help='One actual scene/source and fresh output-path specification')
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    spec=read(inside(args.spec));context=source_preflight(spec)
    print(json.dumps({'scene':context['id'],'status':'actual_source_preflight_passed',
        'source_frames':len(context['ids']),'map':context['town'],
        'map_xodr_actual_byte_sha256':context['source_sha256']['map.xodr'],
        'new_actual_raw_network_conversion_required':True,'execute':args.execute,
        'CARLA_connected_or_ticked':False,'manual_review_approved':False}),flush=True)
    if not args.execute:return 0
    require(os.environ.get('CARLA_ROOT') and os.environ.get('SUMO_HOME'),
            'Set CARLA_ROOT and SUMO_HOME explicitly before native SUMO GUI execution')
    runtime=RuntimeConfig.resolve(require_bridge=True,require_sumo=True,require_gui=True,require_ffmpeg=True)
    execute(context,runtime)
    return 0


if __name__=='__main__':raise SystemExit(main())
