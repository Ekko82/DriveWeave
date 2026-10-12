"""Full decode and real-source sample audit for the six native v3 views.

Unlike the legacy three-output index auditor, roles choose the exact actual
camera/processed source. No role inference from output ordering is permitted.
"""
from pathlib import Path
import argparse
import collections
import json
import math
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent))
import audit_showcase as common
import imageio_ffmpeg
import numpy as np
from PIL import Image

ROLES={'chase','front','rear','image-processing','lidar-processing','sensors'}


def source_folders(run,processed):
    return {'chase':run/'chase','front':run/'rgb','rear':run/'rear',
        'image-processing':processed/'image-processed','lidar-processing':processed/'lidar-3d'}


def reference_frame(role, frame, folders):
    if role=='sensors':
        image=Image.new('RGB',(1920,1080))
        for index,name in enumerate(['chase','front','image-processing','lidar-processing']):
            with Image.open(folders[name]/f'{frame:08d}.png') as picture:
                image.paste(picture.convert('RGB').resize((960,540),Image.Resampling.LANCZOS),((index%2)*960,(index//2)*540))
        return np.asarray(image)
    if role not in folders:
        raise ValueError('Unsupported explicit native v3 media role')
    with Image.open(folders[role]/f'{frame:08d}.png') as image:
        return np.asarray(image.convert('RGB'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.out.exists():raise ValueError('Fresh report path required')
    spec=json.loads(args.manifest.read_text(encoding='utf-8'))
    run=Path(spec['source_run']);processed=Path(spec['processed_run'])
    raw=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    derived=json.loads((processed/'processing-manifest.json').read_text(encoding='utf-8'))
    source=[json.loads(x) for x in (run/'frames.jsonl').read_text(encoding='utf-8').splitlines()]
    ids=spec['native_frame_ids'];actual_ids=[row['frame'] for row in source]
    checks={}
    def check(name,passed):checks[name]=bool(passed)
    check('all_actual_source_frames_without_trim',bool(ids) and ids==actual_ids==derived['native_frame_ids']
        and len(ids)==raw['frames_captured'] and all(b==a+1 for a,b in zip(ids,ids[1:])))
    check('encoder_input_hashes_match_actual_source',
        common.evidence_hash(run/'manifest.json')==spec['capture_manifest_sha256']
        and common.evidence_hash(run/'frames.jsonl')==spec['source_frames_sha256']
        and common.evidence_hash(processed/'processing-manifest.json')==spec['processing_manifest_sha256'])
    check('processing_input_hashes_match_actual_source',derived['source_manifest_sha256']==common.evidence_hash(run/'manifest.json')
        and derived['source_frames_jsonl_sha256']==common.evidence_hash(run/'frames.jsonl'))
    check('native_completed_source_and_original_speed',raw['status']=='complete' and raw.get('native_cleanup_complete') is True
        and raw['synthetic_sensor_fallback'] is False and derived['generated_points'] is False
        and spec['added_text'] is False and spec['playback_speed']==1 and spec['interpolated_frames'] is False
        and spec['raw_sources_modified'] is False)
    roles=[output.get('role') for output in spec['outputs']]
    check('six_unique_explicit_roles',len(roles)==6 and set(roles)==ROLES)
    if not all(checks.values()):
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps({'overall_status':'fail','checks':checks,'failed_checks':[n for n,p in checks.items() if not p]},indent=2)+'\n',encoding='utf-8')
        return 2
    folders=source_folders(run,processed);fps=raw['fps']
    selection={0,len(ids)//4,len(ids)//2,3*len(ids)//4,len(ids)-1}
    evidence=args.out.with_suffix('');evidence.mkdir(parents=True,exist_ok=False)
    videos=[]
    for output in spec['outputs']:
        role=output['role'];path=Path(output['file'])
        report={'role':role,'video':str(path),'checks':{}}
        common.video_checks(path,report,fps,output['size'],len(ids)/fps,[])
        sha=common.evidence_hash(path)
        common.add_check(report,'video_matches_actual_encoder_bytes_and_sha',sha==output['sha256'] and path.stat().st_size==output['bytes'])
        common.add_check(report,'encoder_declares_actual_full_source_count',output['frames']==len(ids) and output['fps']==fps and output['duration_s']==len(ids)/fps)
        reader=imageio_ffmpeg.read_frames(str(path),pix_fmt='rgb24');metadata=next(reader)
        width,height=metadata['size'];samples=[];decoded=0
        for index,data in enumerate(reader):
            decoded+=1
            if index not in selection:continue
            actual=np.frombuffer(data,np.uint8).reshape(height,width,3)
            native=ids[index];reference=reference_frame(role,native,folders)
            if reference.shape!=actual.shape:
                raise ValueError('Actual encoded view differs from its exact role source dimensions')
            mse=float(np.square(actual.astype(np.float64)-reference.astype(np.float64)).mean())
            psnr=10*math.log10(255**2/mse) if mse else 99.
            image=evidence/f'{role}-sample-{index:05d}-native-{native:08d}.png'
            Image.fromarray(actual).save(image)
            samples.append({'sample':index,'native_frame':native,'png':str(image),'png_sha256':common.evidence_hash(image),'psnr_db':psnr})
        common.add_check(report,'all_decoded_frame_count_matches_source',decoded==len(ids),{'decoded':decoded,'source':len(ids)})
        common.add_check(report,'decoded_samples_match_explicit_actual_role_source',len(samples)==len(selection) and all(s['psnr_db']>=30 for s in samples),
            {'sample_count':len(samples),'minimum_psnr_db':min(s['psnr_db'] for s in samples)})
        report.update(samples=samples,video_sha256=sha,video_bytes=path.stat().st_size)
        report['status_counts']=dict(collections.Counter(c['status'] for c in report['checks'].values()))
        videos.append(report)
        print(json.dumps({'role':role,'status_counts':report['status_counts'],'decoded_frames':decoded}),flush=True)
    counts=collections.Counter(c['status'] for v in videos for c in v['checks'].values())
    result={'overall_status':'fail' if counts['error'] or counts['warning'] else 'automated_checks_passed_manual_review_needed',
        'checks':checks,'videos':videos,'status_counts':dict(counts),'native_frame_ids':ids,
        'source_run':str(run.resolve()),'processed_run':str(processed.resolve()),
        'media_manifest_sha256':common.evidence_hash(args.manifest),'auditor_sha256':common.evidence_hash(Path(__file__)),
        'original_speed':True,'trimmed':False,'added_text_or_hud':False,
        'manual_review_needed':True,'full_video_manually_watched':False,
        'ground_truth_masks_not_trained_detector':True,'pointcloud_native_returns_not_generated':True}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({'report':str(args.out),'overall_status':result['overall_status'],'videos':6,'counts':dict(counts)}),flush=True)
    return 2 if counts['error'] or counts['warning'] else 0


if __name__=='__main__':
    raise SystemExit(main())
