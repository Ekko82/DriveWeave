"""Full-decode audit of encoded native CARLA videos, with real-source samples."""
from pathlib import Path
import argparse
import collections
import json
import math
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent))

import imageio_ffmpeg
import numpy as np
from PIL import Image

import audit_showcase as audit


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True,help='encode_native.py actual video manifest')
    p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    spec=json.loads(args.manifest.read_text(encoding='utf-8'))
    run,processed=Path(spec['source_run']),Path(spec['processed_run'])
    raw_manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    derived=json.loads((processed/'processing-manifest.json').read_text(encoding='utf-8'))
    ids=spec['native_frame_ids']
    failures=[]
    if not ids or any(b!=a+1 for a,b in zip(ids,ids[1:])):
        failures.append('Encoded native IDs are missing or not contiguous')
    if audit.evidence_hash(run/'manifest.json')!=spec['capture_manifest_sha256'] or audit.evidence_hash(processed/'processing-manifest.json')!=spec['processing_manifest_sha256']:
        failures.append('Encoder manifest references different raw/processed data')
    if not set(ids).issubset(derived['native_frame_ids']):
        failures.append('Encoded native frames missing from processing manifest')
    if spec.get('playback_speed')!=1 or spec.get('interpolated_frames') is not False or spec.get('synthetic_sensor_data') is not False or spec.get('added_text') is not False:
        failures.append('Raw real-time/no-synthetic/no-text declaration differs')
    fps=raw_manifest['fps']
    folders=[run/'chase',run/'rgb',processed/'image-processed',processed/'lidar-3d']
    selection=set([0,len(ids)//4,len(ids)//2,3*len(ids)//4,len(ids)-1]) if ids else set()
    evidence=args.out.with_suffix('')
    evidence.mkdir(parents=True,exist_ok=True)
    videos=[]
    for number,item in enumerate(spec['outputs']):
        video=Path(item['file'])
        report={'checks':{},'video':str(video)}
        audit.video_checks(video,report,fps,item['size'],len(ids)/fps,[])
        current=audit.evidence_hash(video)
        report['checks']['video_matches_encoder_hash']={'status':'pass' if current==item['sha256'] else 'error'}
        reader=imageio_ffmpeg.read_frames(str(video),pix_fmt='rgb24')
        metadata=next(reader)
        width,height=metadata['size']
        samples=[]
        for index,data in enumerate(reader):
            if index not in selection:
                continue
            actual=np.frombuffer(data,np.uint8).reshape(height,width,3)
            frame=ids[index]
            if number==0:
                reference=Image.new('RGB',(1920,1080))
                for n,folder in enumerate(folders):
                    with Image.open(folder/f'{frame:08d}.png') as picture:
                        reference.paste(picture.convert('RGB').resize((960,540),Image.Resampling.LANCZOS),((n%2)*960,(n//2)*540))
                reference=np.asarray(reference)
            else:
                reference=np.asarray(Image.open(folders[number-1]/f'{frame:08d}.png').convert('RGB'))
            difference=actual.astype(np.float32)-reference.astype(np.float32)
            mse=float(np.square(difference).mean())
            psnr=10*math.log10(255**2/mse) if mse else 99.
            target=evidence/f'video-{number}-sample-{index:05d}-native-{frame:08d}.png'
            Image.fromarray(actual).save(target)
            samples.append({'video_frame':index,'native_frame':frame,'png':str(target),'psnr_db':psnr})
        report['checks']['decoded_samples_match_native_source']={'status':'pass' if len(samples)==len(selection) and all(s['psnr_db']>=30 for s in samples) else 'error','details':{'samples':len(samples),'minimum_psnr_db':min([s['psnr_db'] for s in samples],default=0)}}
        report['samples']=samples
        report['video_sha256']=current
        report['status_counts']=dict(collections.Counter(c['status'] for c in report['checks'].values()))
        videos.append(report)
    counts=collections.Counter(c['status'] for video in videos for c in video['checks'].values())
    result={'overall_status':'fail' if failures or counts['error'] else ('incomplete' if counts['missing'] else 'automated_checks_passed_manual_review_needed'),'failures':failures,'status_counts':dict(counts),'videos':videos,'native_frame_ids':ids,'video_manifest_sha256':audit.evidence_hash(args.manifest),'manual_review_needed':True,'scenario_action_completion_verified':False,'image_boxes_from_native_ground_truth':True,'point_cloud_from_native_lidar':True}
    args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'report':str(args.out),'status':result['overall_status'],'counts':dict(counts),'videos':len(videos)}))
    return int(bool(failures or counts['error'] or counts['missing']))


if __name__=='__main__':
    raise SystemExit(main())
