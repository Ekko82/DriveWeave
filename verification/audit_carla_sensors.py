"""Independently audit actual native capture and derived point-index provenance.

Does not connect to CARLA, start a GPU, synthesize data, or approve unrun code.
"""
from pathlib import Path
import argparse
import collections
import hashlib
import json
import math

import numpy as np
from PIL import Image


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def audit(run,processed=None,require_raw_bgra=False,execution=None,live_evidence=None):
    failures,missing,stats=[],[],collections.Counter()
    manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    records=[json.loads(line) for line in (run/'frames.jsonl').read_text(encoding='utf-8').splitlines() if line]
    def require(ok,message):
        stats['assertions_evaluated']+=1
        if ok:
            stats['assertions_passed']+=1
        if not ok:
            failures.append(message)
    require(manifest.get('status')=='complete','Capture did not complete')
    require(manifest.get('native_cleanup_complete') is True,'Native callback stop/destroy and cleanup completion evidence missing')
    require(manifest.get('server_version')==manifest.get('client_version'),'CARLA client/server versions differ')
    require(manifest.get('ego_blueprint')=='vehicle.tesla.model3','Capture is not the native Tesla Model3 blueprint')
    require(manifest.get('synthetic_sensor_fallback') is False,'No-fallback native capture declaration missing')
    source=manifest.get('source')
    native_source=source=='native CARLA sensor callbacks after synchronous world.tick'
    official_live_source=source=='native CARLA sensor callbacks after official live SimulationSynchronization.tick'
    require(native_source or official_live_source,'Capture source does not identify a supported native synchronous callback path')
    if official_live_source:
        live=manifest.get('live_co_simulation',{})
        require(live.get('mode')=='live_official_bidirectional_actor_synchronization','Live callback source has no official bidirectional synchronizer')
        require(live.get('postcapture_replay') is False,'Live callback source was replayed')
        official=Path(live.get('official_sync_source',''))
        require(official.is_file() and sha(official)==live.get('official_sync_sha256'),'Official live tick source hash differs')
        if live_evidence is None:
            missing.append('Live callback path requires independently verified actual joint execution evidence')
        else:
            proof=json.loads(Path(live_evidence).read_text(encoding='utf-8'))
            require(proof.get('live_joint_simulation_verified') is True and not proof.get('failed_checks'),'Independent live execution audit failed')
            require(bool(proof.get('checks')) and all(v is True for v in proof['checks'].values()),'Independent live execution checks are incomplete')
            require(Path(proof.get('run','')).resolve()==run.resolve(),'Independent live audit references another raw run')
            require(proof.get('frames')==manifest.get('frames_captured') and proof.get('fps')==manifest.get('fps'),'Independent live audit has another timebase')
            for filename,expected in proof.get('input_sha256',{}).items():
                require((run/filename).is_file() and sha(run/filename)==expected,'Independent live source hash differs: '+filename)
            require(set(['manifest.json','frames.jsonl','sumo-sync.jsonl']).issubset(proof.get('input_sha256',{})),'Live audit does not bind actual raw sensor/world and joint-state records')
            require(execution is not None and sha(execution)==proof.get('execution_sha256'),'Live audit is not bound to actual process exit evidence')
    execution_evidence=None
    if execution is None:
        missing.append('Independent outer-process execution evidence missing; capture complete does not prove native client exited0')
    else:
        execution_evidence=json.loads(Path(execution).read_text(encoding='utf-8'))
        require(execution_evidence.get('returncode')==0,'Actual native capture process returned nonzero')
        require(Path(execution_evidence.get('capture_run','')).resolve()==run.resolve(),'Execution evidence references another capture run')
        require(execution_evidence.get('manifest_sha256')==sha(run/'manifest.json'),'Execution manifest SHA differs from current native capture')
        require(execution_evidence.get('frames_jsonl_sha256')==sha(run/'frames.jsonl'),'Execution frame-log SHA differs from current native capture')
    require(len(records)==manifest.get('frames_captured') and bool(records),'Captured frame count differs or is zero')
    fps=float(manifest['fps'])
    K=np.asarray(manifest['camera_intrinsic'],dtype=float)
    require(K.shape==(3,3) and np.isfinite(K).all(),'Invalid native camera intrinsic')
    collisions=run/'collisions.json'
    if not collisions.exists():
        missing.append('Native collision events file missing')
    else:
        require(not json.loads(collisions.read_text(encoding='utf-8')),'Native collision events are present')
    previous=None
    previous_ego=None
    ego_positions=[]
    selected_records={}
    processed_manifest=None
    if processed:
        processed_manifest=json.loads((processed/'processing-manifest.json').read_text(encoding='utf-8'))
        require(processed_manifest.get('source_manifest_sha256')==sha(run/'manifest.json'),'Processing references a different source manifest')
        require(processed_manifest.get('source_frames_jsonl_sha256')==sha(run/'frames.jsonl'),'Processing references different capture frame records')
        require(processed_manifest.get('generated_points') is False,'Processed points provenance declaration missing')
        require(processed_manifest.get('training_or_model_inference') is False,'Non-model image processing declaration inconsistent')
        requested=set(processed_manifest['native_frame_ids'])
        require(len(requested)==len(processed_manifest['native_frame_ids']),'Duplicate processed frame IDs')
    else:
        requested=set()
    for record in records:
        frame=record['frame']
        if previous:
            require(frame==previous['frame']+1,f'Native world frame gap at {frame}')
            require(abs(record['timestamp']-previous['timestamp']-1/fps)<1e-5,f'Native time step differs at {frame}')
        previous=record
        ego=[v for v in record['vehicles'] if v['ego']]
        require(len(ego)==1,f'Expected one ego at {frame}')
        if len(ego)!=1:
            continue
        ego=ego[0]
        require(ego['blueprint']=='vehicle.tesla.model3',f'Actual ego blueprint differs at {frame}')
        ego_matrix=np.asarray(ego['transform']['world_from_local'],float)
        require(ego_matrix.shape==(4,4) and np.isfinite(ego_matrix).all(),f'Native ego transform invalid {frame}')
        if previous_ego is not None:
            elapsed=record['timestamp']-previous_ego['timestamp']
            travel_step=float(np.linalg.norm(ego_matrix[:3,3]-previous_ego['position']))
            allowed=max(float(ego['speed_mps']),previous_ego['speed_mps'])*elapsed+.2
            require(travel_step<=allowed,f'Native ego position jumps beyond physical speed at {frame}')
        previous_ego={'timestamp':record['timestamp'],'position':ego_matrix[:3,3].copy(),'speed_mps':float(ego['speed_mps'])}
        ego_positions.append(ego_matrix[:3,3])
        sensors=record['sensors']
        require(all(name in sensors for name in ['rgb','semantic','lidar']),f'Native sensor stream missing at {frame}')
        for name,sensor in sensors.items():
            require(sensor['frame']==frame and abs(sensor['timestamp']-record['timestamp'])<1e-5,f'Sensor/world mismatch {name} at {frame}')
            actual=np.asarray(sensor['transform']['world_from_local'],float)
            require(actual.shape==(4,4) and np.isfinite(actual).all(),f'Invalid extrinsic {name} at {frame}')
            if actual.shape!=(4,4):
                continue
            require(np.allclose(actual[3],[0,0,0,1],atol=1e-6),f'Invalid homogeneous transform {name} at {frame}')
            require(np.allclose(actual[:3,:3].T@actual[:3,:3],np.eye(3),atol=1e-4),f'Non-rigid sensor rotation {name} at {frame}')
            relative=np.asarray(manifest['sensors'][name]['relative_transform']['world_from_local'],float)
            require(np.allclose(actual,ego_matrix@relative,atol=.002),f'Native rigid ego/sensor calibration differs {name} at {frame}')
        rgb=np.asarray(Image.open(run/'rgb'/f'{frame:08d}.png').convert('RGB'))
        labels=np.asarray(Image.open(run/'semantic_labels'/f'{frame:08d}.png'))
        height,width=rgb.shape[:2]
        require(width==sensors['rgb']['width'] and height==sensors['rgb']['height'],f'Actual saved RGB dimensions differ from native camera {frame}')
        require(rgb.std()>1,f'Uniform/empty RGB frame {frame}')
        require(labels.shape==(height,width) and np.isin(np.unique(labels),np.arange(29)).all(),f'Invalid semantic label taxonomy {frame}')
        fov=float(sensors['rgb']['fov'])
        focal=width/(2*math.tan(math.radians(fov)/2))
        expected_K=np.asarray([[focal,0,width/2],[0,focal,height/2],[0,0,1]])
        require(np.allclose(K,expected_K,atol=1e-6),f'Camera K/FOV/dimensions differ at {frame}')
        require(np.allclose(sensors['rgb']['transform']['world_from_local'],sensors['semantic']['transform']['world_from_local'],atol=1e-6),f'RGB and semantic calibrations differ at {frame}')
        raw=np.load(run/'lidar'/f'{frame:08d}.npy',allow_pickle=False)
        require(raw.dtype==np.dtype('float32') and raw.ndim==2 and raw.shape[1]==4 and len(raw)>0,f'Invalid native XYZI frame {frame}')
        require(np.isfinite(raw).all(),f'Nonfinite native LiDAR {frame}')
        require(np.all(raw[:,3]>=-1e-6) and np.all(raw[:,3]<=1+1e-6),f'Native LiDAR intensity range invalid {frame}')
        require(hashlib.sha256(raw.astype('<f4',copy=False).tobytes()).hexdigest()==sensors['lidar']['raw_sha256'],f'Native callback LiDAR SHA mismatch {frame}')
        require(len(raw)==sensors['lidar']['points']==sum(sensors['lidar']['counts_per_channel']),f'Native LiDAR counts differ {frame}')
        stats['raw_lidar_points']+=len(raw)
        for name in sensors:
            if name=='lidar':
                continue
            bgra=run/'raw_bgra'/f'{name}_{frame:08d}.bin'
            if not bgra.exists():
                stats['camera_frames_without_retained_raw_bgra']+=1
                continue
            data=bgra.read_bytes()
            require(hashlib.sha256(data).hexdigest()==sensors[name]['raw_sha256'],f'Native callback camera SHA mismatch {name} {frame}')
            pixels=np.frombuffer(data,np.uint8).reshape(height,width,4)
            if name=='semantic':
                require(np.array_equal(pixels[:,:,2],labels),f'Semantic image differs from native red channel at {frame}')
            else:
                saved=np.asarray(Image.open(run/name/f'{frame:08d}.png').convert('RGB'))
                require(np.array_equal(pixels[:,:,[2,1,0]],saved),f'Saved RGB differs from raw BGRA conversion {name} {frame}')
        if 'instance' in sensors:
            instance=np.asarray(Image.open(run/'instance'/f'{frame:08d}.png').convert('RGB'))
            require(instance.shape==rgb.shape and np.isin(np.unique(instance[:,:,0]),np.arange(29)).all(),f'Invalid native instance dimensions/taxonomy at {frame}')
            # Check both actual pipelines independently; tiny class-edge raster differences
            # are possible even with identical native frame and camera transform.
            difference=instance[:,:,0]!=labels
            edges=np.zeros_like(difference)
            for actual in [labels,instance[:,:,0]]:
                edge=np.zeros_like(difference)
                edge[1:]|=actual[1:]!=actual[:-1]
                edge[:-1]|=actual[:-1]!=actual[1:]
                edge[:,1:]|=actual[:,1:]!=actual[:,:-1]
                edge[:,:-1]|=actual[:,:-1]!=actual[:,1:]
                padded=np.pad(edge,1)
                for dy in range(3):
                    for dx in range(3):
                        edges|=padded[dy:dy+height,dx:dx+width]
            interior=int((difference&~edges).sum())
            require(np.allclose(sensors['instance']['transform']['world_from_local'],sensors['rgb']['transform']['world_from_local'],atol=1e-6),f'Instance and RGB calibration differs at {frame}')
            stats['native_semantic_instance_mismatch_pixels']+=int(difference.sum())
            stats['native_semantic_instance_mismatch_frames']+=int(difference.any())
            stats['native_semantic_instance_nonboundary_mismatch_pixels']+=interior
            stats['native_semantic_instance_maximum_mismatch_pixels_per_frame']=max(stats['native_semantic_instance_maximum_mismatch_pixels_per_frame'],int(difference.sum()))
        if frame not in requested:
            continue
        selected_records[frame]=record
        result=np.load(processed/'data'/f'{frame:08d}.npz',allow_pickle=False)
        summary=json.loads((processed/'data'/f'{frame:08d}.json').read_text(encoding='utf-8'))
        require(summary['frame']==frame and abs(summary['timestamp']-record['timestamp'])<1e-5,f'Processed/native sync differs {frame}')
        require(summary['native_lidar_raw_sha256']==sensors['lidar']['raw_sha256'],f'Processed LiDAR source SHA differs {frame}')
        kept=result['kept_raw_indices']
        sampled=result['sampled_raw_indices']
        valid_indexes=lambda values: values.dtype.kind in 'iu' and len(values)>0 and len(np.unique(values))==len(values) and values.min()>=0 and values.max()<len(raw)
        require(valid_indexes(kept) and valid_indexes(sampled),f'Invalid/duplicated raw LiDAR index provenance {frame}')
        if not valid_indexes(kept) or not valid_indexes(sampled):
            continue
        require(np.isin(sampled,kept).all(),f'Sampled points did not come from filtered native indices {frame}')
        xyz=raw[sampled,:3].astype(float)
        require(np.all(np.linalg.norm(raw[kept,:3],axis=1)>=2-1e-5) and np.all(np.linalg.norm(raw[kept,:3],axis=1)<=70+1e-5),f'Range filtering failed {frame}')
        ground=result['ground_mask']
        cluster=result['cluster_labels']
        require(ground.shape==cluster.shape==(len(sampled),),f'Ground/cluster cardinality differs {frame}')
        require(np.all(cluster[ground]==-1),f'Ground returns also labelled obstacle clusters {frame}')
        plane=result['ground_plane_abcd']
        require(plane.shape==(4,) and abs(np.linalg.norm(plane[:3])-1)<1e-4 and plane[2]>=math.cos(math.radians(15)),f'Ground plane invalid {frame}')
        residual=abs(xyz@plane[:3]+plane[3])
        require(np.array_equal(residual<.18,ground),f'Ground mask inconsistent with fitted plane {frame}')
        inds=result['projection_sample_indices']
        require(inds.dtype.kind in 'iu' and np.all(inds>=0) and np.all(inds<len(sampled)),f'Projection source indices invalid {frame}')
        if not np.all(inds>=0) or not np.all(inds<len(sampled)):
            continue
        lidar_matrix=np.asarray(sensors['lidar']['transform']['world_from_local'])
        camera_matrix=np.asarray(sensors['rgb']['transform']['world_from_local'])
        world=(lidar_matrix@np.column_stack((xyz[inds],np.ones(len(inds)))).T).T
        native_camera=(np.linalg.inv(camera_matrix)@world.T).T
        depth=native_camera[:,0]
        expected_uv=np.column_stack((focal*native_camera[:,1]/depth+width/2,-focal*native_camera[:,2]/depth+height/2))
        require(np.allclose(depth,result['projection_depth_m'],atol=1e-4) and np.allclose(expected_uv,result['projection_uv'],atol=1e-3),f'Native calibrated projection differs {frame}')
        require(np.all(depth>.1) and np.all(expected_uv>=0) and np.all(expected_uv[:,0]<width) and np.all(expected_uv[:,1]<height),f'Projected points outside valid camera field {frame}')
        cloud_indices=result['cloud3d_sample_indices']
        require(cloud_indices.dtype.kind in 'iu' and np.all(cloud_indices>=0) and np.all(cloud_indices<len(sampled)),f'3D view point provenance invalid {frame}')
        if np.all(cloud_indices>=0) and np.all(cloud_indices<len(sampled)):
            observer=(result['cloud3d_camera_from_local']@np.column_stack((xyz[cloud_indices],np.ones(len(cloud_indices)))).T).T
            optical=np.column_stack((observer[:,1],-observer[:,2],observer[:,0]))
            projected=(result['cloud3d_K']@optical.T).T
            uv3d=projected[:,:2]/projected[:,2,None]
            require(np.allclose(observer[:,0],result['cloud3d_depth_m'],atol=1e-4) and np.allclose(uv3d,result['cloud3d_uv'],atol=1e-3),f'3D view differs from real point projection {frame}')
            require(np.all(observer[:,0]>.1) and np.all(uv3d>=0) and np.all(uv3d[:,0]<width) and np.all(uv3d[:,1]<height),f'3D view points not in observation field {frame}')
        for box in summary['image_boxes']:
            x0,y0,x1,y1=box['bbox_xyxy']
            require(0<=x0<=x1<width and 0<=y0<=y1<height,f'Image target box invalid {frame}')
            if not (0<=x0<=x1<width and 0<=y0<=y1<height):
                continue
            tag=box['class_id']
            box_labels=instance[:,:,0] if 'native_instance_code_gb' in box else labels
            mask=box_labels[y0:y1+1,x0:x1+1]==tag
            if 'native_instance_code_gb' in box:
                native_ids=instance[:,:,1].astype(np.uint16)*256+instance[:,:,2].astype(np.uint16)
                mask &= native_ids[y0:y1+1,x0:x1+1]==box['native_instance_code_gb']
            require(int(mask.sum())>=box['pixel_count']>=20,f'Box not supported by native label pixels {frame}')
        for folder in ['image-processed','rgb-edges','lidar-perspective','lidar-3d','lidar-projected-rgb']:
            picture=np.asarray(Image.open(processed/folder/f'{frame:08d}.png'))
            require(picture.shape[:2]==(height,width),f'Processed output resolution differs {folder} {frame}')
        stats['processed_frames']+=1
        stats['processed_indexed_native_points']+=len(sampled)
        stats['ground_points']+=int(ground.sum())
        stats['geometric_clusters']+=len(summary['clusters'])
        stats['native_truth_image_boxes']+=len(summary['image_boxes'])
        stats['projected_native_points']+=len(inds)
    if require_raw_bgra and stats['camera_frames_without_retained_raw_bgra']:
        missing.append('Exact native camera BGRA not retained for every frame; use capture --keep-raw-bgra')
    if processed:
        require(set(selected_records)==requested,'Processed manifest requests nonexistent captured native frames')
    travel=sum(float(np.linalg.norm(b-a)) for a,b in zip(ego_positions,ego_positions[1:]))
    require(travel>3,'Tesla does not visibly drive more than3m during capture')
    return {'scope':'Native CARLA capture and processed sensor provenance audit; manual visual review and action-specific route acceptance remain separate.','overall_status':'fail' if failures else ('incomplete' if missing else 'automated_checks_passed_manual_review_needed'),'failures':failures,'missing_evidence':missing,'stats':dict(stats),'captured_frames':len(records),'duration_s':len(records)/fps,'native_ego_travel_m':travel,'capture_manifest_sha256':sha(run/'manifest.json'),'source_frames_jsonl_sha256':sha(run/'frames.jsonl'),'processed_manifest_sha256':sha(processed/'processing-manifest.json') if processed else None,'execution_evidence':str(execution) if execution else None,'execution_sha256':sha(execution) if execution else None,'native_process_exit0_verified':bool(execution_evidence and execution_evidence.get('returncode')==0),'manual_review_needed':True,'scenario_action_completion_verified':False,'trained_model_inference_verified':False,'provenance_limits':['Hashes and calibrations audit saved native callbacks; they cannot authenticate an arbitrary forged capture source.','Semantic/instance boxes are native simulator ground truth; not trained neural detections.','Point clusters are geometric components; no classification or tracking claim.']}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--processed',type=Path)
    p.add_argument('--require-raw-bgra',action='store_true')
    p.add_argument('--execution',type=Path,help='Outer runner JSON generated after actual child-process wait/exit')
    p.add_argument('--live-evidence',type=Path,help='Required actual independent joint execution proof for the official live tick callback path')
    p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    report=audit(args.run,args.processed,args.require_raw_bgra,args.execution,args.live_evidence)
    if args.live_evidence:
        report['live_execution_evidence']={'path':str(args.live_evidence.resolve()),'sha256':sha(args.live_evidence)}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'report':str(args.out),'status':report['overall_status'],'failures':len(report['failures']),'missing':len(report['missing_evidence']),'frames':report['captured_frames']}))
    return int(bool(report['failures'] or report['missing_evidence']))


if __name__=='__main__':
    raise SystemExit(main())
