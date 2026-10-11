"""Parallel CPU wrapper around the unchanged native sensor processor."""
from pathlib import Path
import os
for name in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name]='1'
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'verification'))
import argparse
import concurrent.futures
import hashlib
import json
import multiprocessing
import process_carla_sensors as processor


def worker(arguments):
    run,out,record,manifest,voxel=arguments
    return processor.process_frame(Path(run),Path(out),record,manifest,voxel)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',required=True,type=Path)
    p.add_argument('--out',required=True,type=Path)
    p.add_argument('--workers',type=int,default=4)
    p.add_argument('--voxel-m',type=float,default=.18)
    args=p.parse_args()
    if not 1<=args.workers<=6 or not 0<args.voxel_m<=1:raise ValueError('Invalid CPU workers or voxel spacing')
    run=args.run.resolve();out=args.out.resolve()
    if out==run or run in out.parents:raise ValueError('Output must be outside raw capture')
    manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('status')!='complete' or manifest.get('synthetic_sensor_fallback') is not False:raise ValueError('Only completed native capture')
    records=[json.loads(s) for s in (run/'frames.jsonl').read_text(encoding='utf-8').splitlines() if s.strip()]
    if len(records)!=manifest['frames_captured'] or not records:raise ValueError('Native frame log count mismatch')
    out.parent.mkdir(parents=True,exist_ok=True);out.mkdir(exist_ok=False)
    summaries=[]
    with concurrent.futures.ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('spawn')) as pool:
        futures=[pool.submit(worker,(str(run),str(out),record,manifest,args.voxel_m)) for record in records]
        for future in concurrent.futures.as_completed(futures):
            summaries.append(future.result())
            if len(summaries)%25==0 or len(summaries)==len(records):print(f'CPU_PROCESSED {len(summaries)}/{len(records)}',flush=True)
    summaries.sort(key=lambda s:s['frame'])
    if [s['frame'] for s in summaries]!=[s['frame'] for s in records]:raise ValueError('Processed native frame IDs mismatch')
    result={'status':'processed_actual_capture_audit_pending','source_run':str(run),'source_manifest_sha256':sha(run/'manifest.json'),
        'source_frames_jsonl_sha256':sha(run/'frames.jsonl'),'script_sha256':sha(processor.__file__),'batch_wrapper_sha256':sha(__file__),
        'native_frame_ids':[s['frame'] for s in summaries],'voxel_m':args.voxel_m,'cpu_workers':args.workers,'cpu_threads_per_worker':1,
        'training_or_model_inference':False,'image_processing_uses_native_ground_truth':True,'point_processing_uses_native_lidar':True,
        'generated_points':False,'added_text_or_hud':False,'raw_capture_unchanged':True,
        'cluster_method':'Connected neighboring 0.7m voxel cells; geometric obstacle clusters, no semantic classification.',
        'ground_method':'Seeded RANSAC plus SVD refinement; flat-road plane within15deg; failure is explicit.',
        'projection_coordinates':'native CARLA x-forward/y-right/z-up -> optical right/down/forward via (y,-z,x); measured sensor 4x4 and K',
        'limits':['Native ground-truth masks are not a trained detection or perception model.',
            'Geometric point clusters may merge adjacent objects; not identity tracking or full autonomy.',
            'Sparse projection shows only measured returns; it does not fill surfaces or reconstruct missing points.']}
    (out/'processing-manifest.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({'out':str(out),'native_frames':len(records),'cpu_workers':args.workers,'generated_points':False}),flush=True)


if __name__=='__main__':
    multiprocessing.freeze_support()
    main()
