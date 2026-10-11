"""Run a full-source native SUMO GUI replay after independent source gates.

Never connects to CARLA. The visualization remains post-capture replay, even
when its immutable source was real live co-simulation. No source files change.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import os
import subprocess
import sys


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def atomic(path,value):
    temp=path.with_name(path.name+'.tmp')
    with temp.open('w',encoding='utf-8') as stream:
        json.dump(value,stream,indent=2)
        stream.flush();os.fsync(stream.fileno())
    temp.replace(path)


def source_gates(args,rows):
    manifest_sha,frames_sha=sha(args.run/'manifest.json'),sha(args.run/'frames.jsonl')
    for role,path,key in [('motion',args.motion_audit,'motion_quality_validated'),
                          ('action',args.action_audit,'scenario_actions_validated')]:
        report=read(path)
        hashes=report.get('input_sha256',{})
        if not(report.get(key) is True and hashes.get('manifest.json')==manifest_sha
               and hashes.get('frames.jsonl')==frames_sha and Path(report['run']).resolve()==args.run.resolve()):
            raise ValueError(f'Actual {role} source audit failed or references a different recording')
    if args.defer_sensor_video:
        # Operator-authorized parallel preparation only. This must remain
        # explicitly pending until final sensor/video delivery gates pass.
        return {role:{'path':str(path.resolve()),'sha256':sha(path)} for role,path in
                [('motion',args.motion_audit),('action',args.action_audit)]}
    sensor=read(args.sensor_audit)
    if not(sensor.get('overall_status')=='automated_checks_passed_manual_review_needed'
            and not sensor['failures'] and not sensor['missing_evidence']
            and sensor['native_process_exit0_verified'] is True
            and sensor['capture_manifest_sha256']==manifest_sha
            and sensor['source_frames_jsonl_sha256']==frames_sha
            and sensor['captured_frames']==len(rows)):
        raise ValueError('Actual sensor/process audit failed or references a different recording')
    video=read(args.video_audit);native=read(args.native_video_manifest)
    if not(video.get('overall_status')=='automated_checks_passed_manual_review_needed'
            and not video['failures'] and video['video_manifest_sha256']==sha(args.native_video_manifest)
            and Path(native['source_run']).resolve()==args.run.resolve()
            and native['capture_manifest_sha256']==manifest_sha
            and native['native_frame_ids']==[r['frame'] for r in rows]):
        raise ValueError('Native video audit must pass and bind the full actual source frame sequence')
    return {role:{'path':str(path.resolve()),'sha256':sha(path)} for role,path in
        [('motion',args.motion_audit),('action',args.action_audit),('sensor',args.sensor_audit),
         ('video',args.video_audit),('native_video_manifest',args.native_video_manifest)]}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['run','network','out','process-file','motion-audit','action-audit']:
        p.add_argument('--'+name,required=True,type=Path)
    for name in ['sensor-audit','video-audit','native-video-manifest']:
        p.add_argument('--'+name,type=Path)
    p.add_argument('--defer-sensor-video',action='store_true',help='Explicit parallel replay preparation after motion/action acceptance; final source sensor/video gates remain pending')
    p.add_argument('--view-width-m',type=float,default=160)
    args=p.parse_args()
    if not args.defer_sensor_video and any(value is None for value in [args.sensor_audit,args.video_audit,args.native_video_manifest]):
        p.error('Require sensor/video/native manifest gates, or explicitly --defer-sensor-video for preparatory replay')
    args.run=args.run.resolve();args.out=args.out.resolve();args.process_file=args.process_file.resolve()
    rows=[json.loads(line) for line in (args.run/'frames.jsonl').read_text().splitlines()]
    gates=source_gates(args,rows)
    raw_manifest=read(args.run/'manifest.json')
    live_source=raw_manifest.get('live_co_simulation',{})
    source_was_live=(live_source.get('mode')=='live_official_bidirectional_actor_synchronization'
                     and live_source.get('postcapture_replay') is False)
    if args.out.exists():raise FileExistsError('Never replace an existing replay')
    history=read(args.process_file) if args.process_file.exists() else []
    target=args.out/'manifest.json'
    if any(Path(item['manifest']).resolve()==target for item in history):
        raise ValueError('Duplicate outer execution record')
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.process_file.parent.mkdir(parents=True,exist_ok=True)
    log=args.out.parent/(args.out.name+'.process.log')
    if log.exists():raise FileExistsError('Never replace an existing process log')
    base=Path(__file__).resolve().parent
    command=[sys.executable,'-u',str(base/'sumo_sync_replay.py'),'--run',str(args.run),
             '--network',str(args.network.resolve()),'--out',str(args.out),'--samples',str(len(rows)),
             '--width','1280','--height','720','--view-width-m',str(args.view_width_m)]
    started=datetime.now(timezone.utc).isoformat()
    print(json.dumps({'starting':args.out.name,'full_source_samples':len(rows),
                      'source_first_frame':rows[0]['frame'],'source_last_frame':rows[-1]['frame']}),flush=True)
    with log.open('wb') as stream:
        result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,check=False)
    item={'scene':args.out.name,'returncode':result.returncode,'manifest':str(target),
          'log':str(log),'command':command,'process_start_utc':started,
          'process_end_utc':datetime.now(timezone.utc).isoformat(),
          'log_sha256':sha(log),'source_gate_reports':gates,'source_gates_automated_passed':True,
          'source_gate_phase':'motion_action_preparation' if args.defer_sensor_video else 'all_source_gates_passed',
          'source_sensor_video_acceptance_pending':args.defer_sensor_video,
          'manual_visual_review_claimed':False,'mode':'post_capture_synchronized_replay',
          'source_was_live_capture':source_was_live,
          'source_capture_mode':live_source.get('mode') if source_was_live else 'native_CARLA_physics',
          'source_manifest_sha256':sha(args.run/'manifest.json'),
          'this_render_is_live':False,'CARLA_connected_or_ticked':False,
          'manifest_sha256':sha(target) if target.exists() else None,
          'mapping_sha256':sha(args.out/'mapping.jsonl') if (args.out/'mapping.jsonl').exists() else None}
    atomic(args.process_file,[*history,item])
    print(json.dumps({'scene':args.out.name,'actual_returncode':result.returncode,
                      'mode':item['mode'],'mapped_source_samples':len(rows)}),flush=True)
    return result.returncode


if __name__=='__main__':
    raise SystemExit(main())
