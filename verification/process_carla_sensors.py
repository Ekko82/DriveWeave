"""CPU processing of captured native CARLA sensors; never synthesizes sensor data.

Supports capture_native.py's frames.jsonl. Processing is separate from raw data.
Semantic/instance labels are simulator ground truth, not model predictions.
"""
from pathlib import Path
import argparse
import collections
import hashlib
import itertools
import json
import math

import numpy as np
from PIL import Image, ImageDraw

TAGS = {12:'Pedestrian',13:'Rider',14:'Car',15:'Truck',16:'Bus',18:'Motorcycle',19:'Bicycle'}
COLORS = np.asarray([[255,165,45],[30,220,245],[235,60,180],[110,220,60],[150,100,255],[255,230,75]],dtype=np.uint8)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sensor_path(run, sensor, frame):
    folder = {'semantic':'semantic_labels'}.get(sensor,sensor)
    return run / folder / (f'{frame:08d}.npy' if sensor=='lidar' else f'{frame:08d}.png')


def matrix(sensor):
    result = np.asarray(sensor['transform']['world_from_local'],dtype=float)
    if result.shape != (4,4) or not np.isfinite(result).all():
        raise ValueError('Missing/invalid native sensor calibration matrix')
    return result


def project(points_xyz, world_from_lidar, world_from_camera, K, width, height):
    homogeneous = np.column_stack((points_xyz,np.ones(len(points_xyz))))
    camera = (np.linalg.inv(world_from_camera) @ world_from_lidar @ homogeneous.T).T[:,:3]
    # Unreal x-forward/y-right/z-up -> pinhole right/down/forward.
    optical = np.column_stack((camera[:,1],-camera[:,2],camera[:,0]))
    depth = optical[:,2]
    uvh = (np.asarray(K) @ optical.T).T
    uv = uvh[:,:2] / np.where(abs(depth)>1e-12,depth,np.nan)[:,None]
    valid = (depth>0.1) & np.isfinite(uv).all(axis=1) & (uv[:,0]>=0) & (uv[:,0]<width) & (uv[:,1]>=0) & (uv[:,1]<height)
    return np.flatnonzero(valid),uv[valid],depth[valid]


def fit_ground(xyz, seed=0, threshold=.18, iterations=160):
    # Local ground search excludes roofs and distant building facades.
    roi = (abs(xyz[:,1])<8) & (xyz[:,0]>-15) & (xyz[:,0]<35) & (xyz[:,2]<-.5)
    candidates = xyz[roi]
    if len(candidates)<40:
        raise ValueError('Not enough actual road returns to fit a ground plane')
    low = np.quantile(candidates[:,2],.25)
    candidates = candidates[abs(candidates[:,2]-low)<.65]
    if len(candidates)<30:
        raise ValueError('Not enough low-height road returns for ground RANSAC')
    rng = np.random.default_rng(seed)
    sample = candidates[rng.choice(len(candidates),min(4096,len(candidates)),replace=False)]
    best, best_count = None,0
    for _ in range(iterations):
        a,b,c = sample[rng.choice(len(sample),3,replace=False)]
        normal = np.cross(b-a,c-a)
        norm = np.linalg.norm(normal)
        if norm<1e-8:
            continue
        normal /= norm
        if normal[2]<0:
            normal = -normal
        if normal[2]<math.cos(math.radians(15)):
            continue
        intercept = -normal@a
        count = int(np.count_nonzero(abs(sample@normal+intercept)<threshold))
        if count>best_count:
            best,best_count = (normal,intercept),count
    if best is None or best_count < max(30,int(.25*len(sample))):
        raise ValueError('Ground plane unsupported by native points; no fabricated fallback')
    inliers = abs(candidates@best[0]+best[1])<threshold
    cloud = candidates[inliers]
    center = cloud.mean(axis=0)
    _,_,basis = np.linalg.svd(cloud-center,full_matrices=False)
    normal = basis[-1]
    if normal[2]<0:
        normal = -normal
    intercept = -normal@center
    if normal[2]<math.cos(math.radians(15)):
        raise ValueError('Refined ground normal exceeds supported flat-road slope')
    return normal,intercept,abs(xyz@normal+intercept)<threshold


def voxel_representatives(points, raw_indices, voxel_m):
    keys = np.floor(points[:,:3]/voxel_m).astype(np.int64)
    # Pick an existing native return; centroids would create new points.
    _,first = np.unique(keys,axis=0,return_index=True)
    first.sort()
    return raw_indices[first]


def cluster_voxels(xyz, nonground, cell_m=.7, minimum=6):
    labels = np.full(len(xyz),-1,dtype=np.int32)
    buckets = collections.defaultdict(list)
    for i in np.flatnonzero(nonground):
        buckets[tuple(np.floor(xyz[i]/cell_m).astype(int))].append(int(i))
    offsets = list(itertools.product((-1,0,1),repeat=3))
    unseen = set(buckets)
    clusters = []
    while unseen:
        start = min(unseen)
        unseen.remove(start)
        pending = [start]
        members = []
        while pending:
            key = pending.pop()
            members += buckets[key]
            for off in offsets:
                neighbor = tuple(key[d]+off[d] for d in range(3))
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    pending.append(neighbor)
        if len(members)<minimum:
            continue
        label = len(clusters)
        labels[members] = label
        p = xyz[members]
        clusters.append({'cluster':label,'points':len(members),'bounds_min_xyz_m':p.min(axis=0).tolist(),'bounds_max_xyz_m':p.max(axis=0).tolist()})
    return labels,clusters


def semantic_components(labels, minimum=20):
    """Run-length union components, avoiding OpenCV/SciPy dependencies."""
    boxes = []
    for tag in TAGS:
        parent, nodes, prior = {},{},[]
        def find(i):
            while parent[i]!=i:
                parent[i]=parent[parent[i]]
                i=parent[i]
            return i
        for y in range(labels.shape[0]):
            mask = labels[y]==tag
            boundaries = np.diff(np.pad(mask.astype(np.int8),(1,1)))
            current = []
            for x0,xend in zip(np.flatnonzero(boundaries==1),np.flatnonzero(boundaries==-1)):
                i=len(nodes)
                parent[i]=i
                nodes[i]=(int(x0),int(y),int(xend-1),int(y),int(xend-x0))
                current.append((x0,xend-1,i))
                for left,right,j in prior:
                    if left>xend:
                        break
                    if right+1>=x0 and left<=xend:
                        a,b=find(i),find(j)
                        if a!=b:
                            parent[b]=a
            prior=current
        merged = {}
        for i,box in nodes.items():
            root=find(i)
            if root not in merged:
                merged[root]=list(box)
            else:
                old=merged[root]
                merged[root]=[min(old[0],box[0]),min(old[1],box[1]),max(old[2],box[2]),max(old[3],box[3]),old[4]+box[4]]
        boxes += [{'class_id':tag,'class_name':TAGS[tag],'bbox_xyxy':b[:4],'pixel_count':b[4],'origin':'native semantic connected component; touching objects may merge'} for b in merged.values() if b[4]>=minimum]
    return boxes


def semantic_instance_consistency(labels,instance_labels):
    if labels.shape!=instance_labels.shape:
        raise ValueError('Native instance/semantic image dimensions differ')
    mismatch=labels!=instance_labels
    boundaries=np.zeros_like(mismatch)
    for actual in [labels,instance_labels]:
        edge=np.zeros_like(mismatch)
        edge[1:]|=actual[1:]!=actual[:-1]
        edge[:-1]|=actual[:-1]!=actual[1:]
        edge[:,1:]|=actual[:,1:]!=actual[:,:-1]
        edge[:,:-1]|=actual[:,:-1]!=actual[:,1:]
        padded=np.pad(edge,1)
        for dy in range(3):
            for dx in range(3):
                boundaries|=padded[dy:dy+labels.shape[0],dx:dx+labels.shape[1]]
    count=int(mismatch.sum())
    interior=int((mismatch&~boundaries).sum())
    result={'mismatch_pixels':count,'total_pixels':labels.size,'mismatch_fraction':count/labels.size,'outside_one_pixel_class_boundaries':interior,'policy':'Diagnostic comparison of independent native camera rasterizations only. Each camera retains its own class truth; raw bytes, frame, timestamp and calibration remain independently verified. No cross-camera pixel equality is assumed.','diagnostic_only':True}
    return result


def instance_boxes(instance_rgb, labels, minimum=20):
    semantic_instance_consistency(labels,instance_rgb[:,:,0])
    # Each native camera keeps its own class truth; do not overwrite either stream.
    labels=instance_rgb[:,:,0]
    instance = instance_rgb[:,:,1].astype(np.uint16)*256+instance_rgb[:,:,2].astype(np.uint16)
    boxes=[]
    for tag in TAGS:
        for value in np.unique(instance[labels==tag]):
            y,x=np.where((labels==tag)&(instance==value))
            if len(x)>=minimum:
                boxes.append({'class_id':tag,'class_name':TAGS[tag],'native_instance_code_gb':int(value),'bbox_xyxy':[int(x.min()),int(y.min()),int(x.max()),int(y.max())],'pixel_count':len(x),'origin':'native CARLA instance mask; code is not assumed equal to actor ID'})
    return boxes


def rgb_edges(rgb):
    gray=rgb.astype(np.float32)@np.asarray([.299,.587,.114],np.float32)
    pad=np.pad(gray,1,mode='edge')
    gx=-pad[:-2,:-2]+pad[:-2,2:]-2*pad[1:-1,:-2]+2*pad[1:-1,2:]-pad[2:,:-2]+pad[2:,2:]
    gy=-pad[:-2,:-2]-2*pad[:-2,1:-1]-pad[:-2,2:]+pad[2:,:-2]+2*pad[2:,1:-1]+pad[2:,2:]
    magnitude=np.hypot(gx,gy)
    threshold=max(40.,float(np.quantile(magnitude,.9)))
    return np.uint8(magnitude>=threshold)*255,threshold


def draw_cloud(width,height,uv,depth,colors,radius=0):
    result=np.zeros((height,width,3),np.uint8)
    # Existing native points projected with z ordering; no filled obstacle shapes.
    if radius:
        picture=Image.fromarray(result)
        draw=ImageDraw.Draw(picture)
        for i in np.argsort(depth)[::-1]:
            x,y=np.floor(uv[i]).astype(int)
            draw.ellipse((x-radius,y-radius,x+radius,y+radius),fill=tuple(colors[i]))
        result=np.asarray(picture)
    else:
        for i in np.argsort(depth)[::-1]:
            x,y=np.floor(uv[i]).astype(int)
            result[y,x]=colors[i]
    return result


def virtual_cloud_camera(width,height):
    # A fixed observer in LiDAR-local coordinates; not a synthesized sensor.
    position=np.asarray([-12.,-18.,13.])
    target=np.asarray([20.,0.,-2.3])
    forward=target-position
    forward/=np.linalg.norm(forward)
    right=np.cross([0.,0.,1.],forward)
    right/=np.linalg.norm(right)
    up=np.cross(forward,right)
    transform=np.eye(4)
    transform[:3,:3]=np.column_stack((forward,right,up))
    transform[:3,3]=position
    focal=width/(2*math.tan(math.radians(70)/2))
    intrinsic=np.asarray([[focal,0,width/2],[0,focal,height/2],[0,0,1.]])
    return transform,intrinsic


def process_frame(run,out,record,manifest,voxel_m=.18):
    frame=record['frame']
    sensors=record['sensors']
    if not all(name in sensors for name in ['rgb','semantic','lidar']):
        raise ValueError('Required native sensor stream absent')
    for name in sensors:
        if sensors[name]['frame']!=frame or abs(sensors[name]['timestamp']-record['timestamp'])>1e-5:
            raise ValueError(f'Unsynchronized {name} at native frame {frame}')
    if not np.allclose(matrix(sensors['rgb']),matrix(sensors['semantic']),atol=1e-6):
        raise ValueError('RGB/semantic sensor poses differ')
    rgb=np.asarray(Image.open(sensor_path(run,'rgb',frame)).convert('RGB'))
    labels=np.asarray(Image.open(sensor_path(run,'semantic',frame)))
    raw=np.load(sensor_path(run,'lidar',frame),allow_pickle=False)
    if raw.ndim!=2 or raw.shape[1]!=4 or raw.dtype!=np.dtype('float32') or not len(raw):
        raise ValueError('LiDAR must be a nonempty native float32 Nx4 XYZI array')
    if labels.shape!=rgb.shape[:2] or not np.isin(np.unique(labels),np.arange(29)).all():
        raise ValueError('Wrong native semantic image shape/class taxonomy')
    if hashlib.sha256(raw.astype('<f4',copy=False).tobytes()).hexdigest()!=sensors['lidar']['raw_sha256']:
        raise ValueError('LiDAR array bytes differ from capture callback SHA')
    distance=np.linalg.norm(raw[:,:3],axis=1)
    keep=np.isfinite(raw).all(axis=1)&(distance>=2)&(distance<=70)&(raw[:,2]>-5)&(raw[:,2]<5)
    # Remove exact ego bounding box in the sensor frame, with a small skin margin.
    ego=next(v for v in record['vehicles'] if v['ego'])
    ego_from_lidar=np.linalg.inv(np.asarray(ego['transform']['world_from_local']))@matrix(sensors['lidar'])
    local=(ego_from_lidar@np.column_stack((raw[:,:3],np.ones(len(raw)))).T).T[:,:3]
    bbox=ego['bounding_box']
    inside=np.all(abs(local-np.asarray(bbox['center_local_m']))<=np.asarray(bbox['extent_m'])+.15,axis=1)
    if any(abs(float(a))>1e-6 for a in bbox.get('rotation_rpy_deg',[0,0,0])):
        raise ValueError('Rotated ego bounding box needs an explicit bbox transform before self-filtering')
    keep &= ~inside
    kept=np.flatnonzero(keep)
    if len(kept)<40:
        raise ValueError('Too few genuine points remain after range/self filtering')
    sampled=voxel_representatives(raw[kept],kept,voxel_m)
    xyz=raw[sampled,:3]
    normal,offset,ground=fit_ground(xyz,seed=frame)
    cluster_labels,clusters=cluster_voxels(xyz,~ground)
    height,width=rgb.shape[:2]
    K=np.asarray(manifest['camera_intrinsic'])
    projected,uv,depth=project(xyz,matrix(sensors['lidar']),matrix(sensors['rgb']),K,width,height)
    instance_rgb=np.asarray(Image.open(sensor_path(run,'instance',frame)).convert('RGB')) if 'instance' in sensors else None
    boxes=instance_boxes(instance_rgb,labels) if instance_rgb is not None else semantic_components(labels)
    edges,edge_threshold=rgb_edges(rgb)
    processed=rgb.copy()
    for tag in TAGS:
        mask=labels==tag
        color=COLORS[list(TAGS).index(tag)%len(COLORS)]
        processed[mask]=np.uint8(.65*processed[mask]+.35*color)
    picture=Image.fromarray(processed)
    draw=ImageDraw.Draw(picture)
    for box in boxes:
        draw.rectangle(box['bbox_xyxy'],outline=tuple(COLORS[list(TAGS).index(box['class_id'])%len(COLORS)]),width=2)
    point_colors=COLORS[np.maximum(cluster_labels,0)%len(COLORS)].copy()
    point_colors[cluster_labels<0]=[145,155,165]
    point_colors[ground]=[65,190,100]
    cloud=draw_cloud(width,height,uv,depth,point_colors[projected])
    cloud_camera,cloud_K=virtual_cloud_camera(width,height)
    cloud_projected,cloud_uv,cloud_depth=project(xyz,np.eye(4),cloud_camera,cloud_K,width,height)
    cloud_3d=draw_cloud(width,height,cloud_uv,cloud_depth,point_colors[cloud_projected],radius=2)
    fused=rgb.copy()
    visible=np.any(cloud,axis=2)
    fused[visible]=cloud[visible]
    token=f'{frame:08d}'
    for folder in ['image-processed','rgb-edges','lidar-perspective','lidar-3d','lidar-projected-rgb','data']:
        (out/folder).mkdir(exist_ok=True)
    picture.save(out/'image-processed'/(token+'.png'))
    Image.fromarray(edges).save(out/'rgb-edges'/(token+'.png'))
    Image.fromarray(cloud).save(out/'lidar-perspective'/(token+'.png'))
    Image.fromarray(cloud_3d).save(out/'lidar-3d'/(token+'.png'))
    Image.fromarray(fused).save(out/'lidar-projected-rgb'/(token+'.png'))
    np.savez_compressed(out/'data'/(token+'.npz'),kept_raw_indices=kept,sampled_raw_indices=sampled,ground_mask=ground,cluster_labels=cluster_labels,projection_sample_indices=projected,projection_uv=uv,projection_depth_m=depth,ground_plane_abcd=np.r_[normal,offset],cloud3d_sample_indices=cloud_projected,cloud3d_uv=cloud_uv,cloud3d_depth_m=cloud_depth,cloud3d_camera_from_local=np.linalg.inv(cloud_camera),cloud3d_K=cloud_K)
    summary={'frame':frame,'timestamp':record['timestamp'],'native_lidar_raw_sha256':sensors['lidar']['raw_sha256'],'input_lidar_count':len(raw),'kept_count':len(kept),'sampled_count':len(sampled),'ground_count':int(ground.sum()),'nonground_count':int((~ground).sum()),'projected_count':len(projected),'clusters':clusters,'image_boxes':boxes,'image_box_method':'native-instance-ground-truth' if 'instance' in sensors else 'native-semantic-ground-truth-connected-components','rgb_edge_method':'Sobel magnitude percentile threshold','rgb_edge_threshold':edge_threshold,'ground_plane_abcd':np.r_[normal,offset].tolist(),'point_provenance':'All processed points are indexed rows of the saved native LiDAR array; no generated or hand-drawn sensor returns.'}
    summary['cloud3d_view']={'method':'Fixed virtual observation camera applied to real indexed native LiDAR points, not an extra sensor','observer_lidar_local_xyz_m':cloud_camera[:3,3].tolist(),'fov_horizontal_deg':70,'visible_points':len(cloud_projected),'point_radius_pixels':2,'depth_rule':'Far to near projected depth; nearest measured point disc occludes farther discs','world_geometry_added':False,'text_added':False}
    if instance_rgb is not None:
        summary['semantic_instance_consistency']=semantic_instance_consistency(labels,instance_rgb[:,:,0])
    (out/'data'/(token+'.json')).write_text(json.dumps(summary,indent=2),encoding='utf-8')
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--frames',type=int,nargs='+',help='Native CARLA frame IDs; default all captured frames')
    p.add_argument('--voxel-m',type=float,default=.18)
    args=p.parse_args()
    manifest=json.loads((args.run/'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('status')!='complete' or manifest.get('synthetic_sensor_fallback') is not False:
        raise ValueError('Only a completed native capture with no sensor fallback can be processed')
    if not 0<args.voxel_m<=1:
        raise ValueError('Voxel size must be in (0,1] metres')
    if args.out.resolve()==args.run.resolve() or args.run.resolve() in args.out.resolve().parents:
        raise ValueError('Processed output must be outside raw capture directory')
    args.out.mkdir(parents=True,exist_ok=False)
    selected=set(args.frames) if args.frames else None
    summaries=[]
    for line in (args.run/'frames.jsonl').read_text(encoding='utf-8').splitlines():
        record=json.loads(line)
        if selected is None or record['frame'] in selected:
            summaries.append(process_frame(args.run,args.out,record,manifest,args.voxel_m))
            print(f"PROCESSED native-frame={record['frame']}",flush=True)
    if not summaries or selected is not None and {s['frame'] for s in summaries}!=selected:
        raise ValueError('Requested native frames are missing; no stand-in data will be generated')
    result={'status':'processed_actual_capture_audit_pending','source_run':str(args.run.resolve()),'source_manifest_sha256':sha(args.run/'manifest.json'),'source_frames_jsonl_sha256':sha(args.run/'frames.jsonl'),'script_sha256':sha(__file__),'native_frame_ids':[s['frame'] for s in summaries],'voxel_m':args.voxel_m,'training_or_model_inference':False,'image_processing_uses_native_ground_truth':True,'point_processing_uses_native_lidar':True,'generated_points':False,'added_text_or_hud':False,'raw_capture_unchanged':True,'cluster_method':'Connected neighboring 0.7m voxel cells; geometric obstacle clusters, no semantic classification.','ground_method':'Seeded RANSAC plus SVD refinement; flat-road plane within15deg; failure is explicit.','projection_coordinates':'native CARLA x-forward/y-right/z-up -> optical right/down/forward via (y,-z,x); measured sensor 4x4 and K','limits':['Native ground-truth masks are not a trained detection or perception model.','Geometric point clusters may merge adjacent objects; not identity tracking or full autonomy.','Sparse projection shows only measured returns; it does not fill surfaces or reconstruct missing points.']}
    (args.out/'processing-manifest.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({'status':result['status'],'frames':len(summaries),'out':str(args.out)}))


if __name__=='__main__':
    main()
