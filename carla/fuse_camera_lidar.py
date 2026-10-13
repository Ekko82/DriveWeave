"""Offline RGB detector / measured LiDAR association; no simulator labels as input.

This is a geometric late-fusion baseline, not a trained 3D detector. Distances
refer to the median observed surface return, not the unobserved object center.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial import cKDTree
from scipy.optimize import linear_sum_assignment

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'verification'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from process_carla_sensors import matrix, project

PALETTE = [(37, 229, 246), (255, 180, 48), (227, 100, 245),
           (110, 239, 114), (245, 115, 124), (125, 167, 255)]


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def components(xyz, radius):
    """Euclidean components of actual returns inside one image frustum."""
    tree = cKDTree(xyz)
    unseen = set(range(len(xyz)))
    groups = []
    while unseen:
        seed = min(unseen)
        unseen.remove(seed)
        queue, members = [seed], []
        while queue:
            current = queue.pop()
            members.append(current)
            for neighbor in tree.query_ball_point(xyz[current], radius):
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    queue.append(neighbor)
        if len(members) >= 3:
            groups.append(np.asarray(members, dtype=int))
    return groups


def associate(detections, xyz, raw_indices, ground, projected, uv, depth):
    """Associate supported frustum components, retaining explicit unmatched boxes.

    Each measured point can support at most one detection. A component must have
    central image support and plausible observed extent. Low-support or ambiguous
    depth components are rejected rather than assigned a fabricated distance.
    """
    results, used = [], set()
    for detection in sorted(detections, key=lambda d: -d['score']):
        result = dict(detection, fusion_status='camera_only', sampled_point_indices=[],
                      raw_point_indices=[], range_m=None, observed_center_lidar_m=None)
        x0, y0, x1, y1 = detection['bbox_xyxy']
        width, height = x1-x0, y1-y0
        inside = ((uv[:, 0] >= x0) & (uv[:, 0] <= x1) &
                  (uv[:, 1] >= y0) & (uv[:, 1] <= y1) & ~ground[projected] &
                  np.isfinite(uv).all(axis=1) & np.isfinite(depth) & (depth > .1))
        visible = np.flatnonzero(inside)
        visible = np.asarray([i for i in visible if int(projected[i]) not in used], dtype=int)
        candidates = []
        minimum_support = 6
        if len(visible) >= minimum_support:
            radius = float(np.clip(np.median(depth[visible]) * .018, .65, 1.25))
            for group in components(xyz[projected[visible]], radius):
                if len(group) < minimum_support:
                    continue
                indices = visible[group]
                sample_ids = projected[indices]
                points = xyz[sample_ids]
                extent = np.ptp(points, axis=0)
                if max(extent[:2]) > 10 or extent[2] > 4.5:
                    continue
                pixel = uv[indices]
                center_support = ((abs(pixel[:, 0] - (x0+x1)/2) < width*.40) &
                                  (abs(pixel[:, 1] - (y0+y1)/2) < height*.43))
                if np.count_nonzero(center_support) < 2:
                    continue
                span = np.ptp(pixel, axis=0)
                if span[0] < max(2.,width*.20) or span[1] < max(2.,height*.12):
                    continue
                if max(extent) < .25:
                    continue
                coverage = float(np.clip(span[0]/max(width,1),0,1) *
                                 np.clip(span[1]/max(height,1),0,1))
                score = float(np.sqrt(len(group)) * (.25 + coverage) *
                              (.5 + np.mean(center_support)))
                candidates.append((score, indices, float(np.median(depth[indices]))))
        candidates.sort(key=lambda item: -item[0])
        if candidates:
            best = candidates[0]
            ambiguous = any(item[0] > .5*best[0] and abs(item[2]-best[2]) > 3
                            for item in candidates[1:])
            if ambiguous:
                result['fusion_status'] = 'ambiguous_depth'
            else:
                sample_ids = projected[best[1]]
                center = np.median(xyz[sample_ids], axis=0)
                result.update(fusion_status='associated',
                    sampled_point_indices=sample_ids.tolist(),
                    raw_point_indices=raw_indices[sample_ids].tolist(),
                    range_m=float(np.median(np.linalg.norm(xyz[sample_ids], axis=1))),
                    observed_center_lidar_m=center.tolist(),
                    observed_bounds_min_m=xyz[sample_ids].min(axis=0).tolist(),
                    observed_bounds_max_m=xyz[sample_ids].max(axis=0).tolist(),
                    association_support_points=len(sample_ids),
                    association_score=best[0])
                used.update(map(int, sample_ids))
        results.append(result)
    return results


def iou(a, b):
    a, b = np.asarray(a), np.asarray(b)
    overlap = np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2]))
    intersection = np.prod(overlap)
    return float(intersection / max(np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2])-intersection, 1e-8))


class AssociationTracker:
    """Short-lived matching for display colors. Never draws unobserved tracks."""
    def __init__(self):
        self.tracks, self.next_id, self.last_time = {}, 1, None

    def update(self, detections, timestamp, world_from_lidar):
        if not np.isfinite(timestamp) or (self.last_time is not None and timestamp <= self.last_time):
            raise ValueError('Tracker requires strictly increasing finite sensor timestamps')
        self.last_time = timestamp
        active = {key: item for key, item in self.tracks.items() if timestamp-item['time'] <= .5}
        for item in detections:
            center = item.get('observed_center_lidar_m')
            item['world_observed_center_m'] = ((world_from_lidar @ np.r_[center,1])[:3].tolist()
                                               if center is not None else None)
        keys = list(active)
        cost = np.full((len(keys),len(detections)), 1e6)
        for row, key in enumerate(keys):
            prior = active[key]
            for col, item in enumerate(detections):
                if item['class_id'] != prior['class_id']:
                    continue
                overlap = iou(prior['bbox_xyxy'], item['bbox_xyxy'])
                a, b = prior.get('world_observed_center_m'), item.get('world_observed_center_m')
                if a is not None and b is not None:
                    distance = np.linalg.norm(np.asarray(a)-b)
                    gate = 3 + 25*(timestamp-prior['time'])
                    if distance < gate:
                        cost[row,col] = distance/gate + .25*(1-overlap)
                elif overlap > .25:
                    cost[row,col] = 1-overlap
        assigned = {}
        if cost.size:
            for row,col in zip(*linear_sum_assignment(cost)):
                if cost[row,col] < 1.3:
                    assigned[col] = keys[row]
        for col,item in enumerate(detections):
            identifier = assigned.get(col)
            if identifier is None:
                identifier, self.next_id = self.next_id, self.next_id+1
            item['display_track_id'] = identifier
            active[identifier] = dict(item,time=timestamp)
        self.tracks = active


def font(size=17):
    try:
        return ImageFont.truetype('DejaVuSans.ttf',size)
    except OSError:
        return ImageFont.load_default(size=size)


def label(draw, xy, text, color, size=17):
    face = font(size)
    x,y = xy
    box = draw.textbbox((x,y), text, font=face)
    draw.rectangle((box[0]-3,box[1]-2,box[2]+3,box[3]+2),fill=(9,16,24))
    draw.text((x,y),text,font=face,fill=color)


def render(rgb, xyz, ground, projected, uv, detections):
    picture = Image.fromarray(rgb)
    draw = ImageDraw.Draw(picture)
    colors = np.full((len(xyz),3),[111,132,151],np.uint8)
    colors[ground] = [48,99,84]
    pixel_lookup = {int(index):pixel for index,pixel in zip(projected,uv)}
    for item in detections:
        matched = item['fusion_status']=='associated'
        color = PALETTE[(item['display_track_id']-1)%len(PALETTE)] if matched else (215,220,228)
        ids = np.asarray(item['sampled_point_indices'],dtype=int)
        colors[ids] = color
        for index in ids:
            x,y = pixel_lookup[int(index)]
            draw.ellipse((x-1,y-1,x+1,y+1),fill=color)
        x0,y0,x1,y1 = item['bbox_xyxy']
        draw.rectangle((x0,y0,x1,y1),outline=color,width=2)
        text = f"#{item['display_track_id']} {item['class_name']} {item['score']:.2f}"
        if matched:
            text += f"  {item['range_m']:.1f}m"
        label(draw,(max(4,min(x0,970)),max(2,y0-24)),text,color)
    # Fixed 3D observer, entirely derived from actual LiDAR returns.
    position, target = np.array([-18.,-28.,24.]), np.array([15.,0.,-2.])
    forward = target-position
    forward /= np.linalg.norm(forward)
    right = np.cross([0,0,1.],forward)
    right /= np.linalg.norm(right)
    transform = np.eye(4)
    transform[:3,:3] = np.column_stack((forward,right,np.cross(forward,right)))
    transform[:3,3] = position
    K = np.array([[440.,0,320],[0,440.,330],[0,0,1.]])
    indices,pixels,depth = project(xyz,np.eye(4),transform,K,640,660)
    lidar = Image.new('RGB',(640,1080),(8,15,23))
    cloud = ImageDraw.Draw(lidar)
    for i in np.argsort(depth)[::-1]:
        x,y = pixels[i]
        cloud.ellipse((x-1,y-1,x+1,y+1),fill=tuple(map(int,colors[indices[i]])))
    # Bird's-eye projection of observed returns: x [-20,60], y [-60,60].
    cloud.rectangle((0,660,639,1079),fill=(11,21,30))
    def bev(points):
        return np.column_stack((320+points[:,1]*5.1, 970-points[:,0]*5.1))
    for radius in [10,20,40,60]:
        r=radius*5.1
        cloud.arc((320-r,970-r,320+r,970+r),180,360,fill=(30,46,59),width=1)
    cloud.line((320,665,320,1079),fill=(30,46,59))
    pixels = bev(xyz)
    valid = (pixels[:,0]>=0)&(pixels[:,0]<640)&(pixels[:,1]>=664)&(pixels[:,1]<1080)
    for index in np.flatnonzero(valid):
        x,y = pixels[index]
        cloud.point((x,y),fill=tuple(map(int,colors[index])))
    for item in detections:
        if item['fusion_status'] != 'associated':
            continue
        color=PALETTE[(item['display_track_id']-1)%len(PALETTE)]
        ids=np.asarray(item['sampled_point_indices'],dtype=int)
        points=xyz[ids]
        small=bev(points)
        x0,y0=small.min(axis=0)
        x1,y1=small.max(axis=0)
        if x1>=0 and x0<640 and y1>=664 and y0<1080:
            cloud.rectangle((max(0,x0-2),max(664,y0-2),min(639,x1+2),min(1079,y1+2)),outline=color,width=2)
            label(cloud,(max(4,min(510,x0)),max(666,min(1055,y0-20))),
                  f"#{item['display_track_id']} {item['range_m']:.1f}m",color,15)
        center=np.median(points,axis=0)[None,:]
        keep,pos,_=project(center,np.eye(4),transform,K,640,660)
        if len(keep):
            label(cloud,(max(4,min(555,pos[0,0])),max(2,min(632,pos[0,1]-22))),
                  f"#{item['display_track_id']}",color,15)
    return picture,lidar


def process(args):
    from detect_rgb_onnx import Detector
    script_sha = sha(__file__)
    detector_source = Path(__file__).with_name('detect_rgb_onnx.py')
    detector_sha = sha(detector_source)
    run, processed, out = args.run.resolve(),args.processed.resolve(),args.out.resolve()
    if out==run or run in out.parents or out==processed or processed in out.parents:
        raise ValueError('Derived output must be outside preserved sensor input folders')
    manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    processing=json.loads((processed/'processing-manifest.json').read_text(encoding='utf-8'))
    if manifest.get('status')!='complete' or manifest.get('synthetic_sensor_fallback') is not False:
        raise ValueError('A completed native sensor capture is required')
    if processing['source_frames_jsonl_sha256'] != sha(run/'frames.jsonl'):
        raise ValueError('Processed points belong to a different capture')
    rows=[json.loads(line) for line in (run/'frames.jsonl').read_text().splitlines() if line.strip()]
    selected=rows[args.start_sample:args.end_sample]
    if not selected:
        raise ValueError('No selected frames')
    out.mkdir(parents=True,exist_ok=False)
    (out/'frames').mkdir()
    detector=Detector(args.model,threads=args.threads,confidence=.3)
    if not detector.provenance.get('matches_known_official_model_sha256'):
        raise ValueError('This baseline requires the hash-bound official YOLOX-S model')
    tracker=AssociationTracker()
    records,begin=[],time.perf_counter()
    with (out/'associations.jsonl').open('w',encoding='utf-8') as stream:
        for row in selected:
            frame=row['frame']; token=f'{frame:08d}'; sensors=row['sensors']
            for name in ('rgb','lidar'):
                if sensors[name]['frame']!=frame or abs(sensors[name]['timestamp']-row['timestamp'])>1e-6:
                    raise ValueError('Camera/LiDAR time mismatch')
            rgb_path=run/'rgb'/f'{token}.png'
            rgb=np.asarray(Image.open(rgb_path).convert('RGB'))
            raw=np.load(run/'lidar'/f'{token}.npy',allow_pickle=False)
            if hashlib.sha256(raw.astype('<f4',copy=False).tobytes()).hexdigest()!=sensors['lidar']['raw_sha256']:
                raise ValueError('Native LiDAR bytes changed')
            data_path=processed/'data'/f'{token}.npz'
            with np.load(data_path,allow_pickle=False) as data:
                raw_indices=data['sampled_raw_indices'].copy()
                ground=data['ground_mask'].copy()
                xyz=raw[raw_indices,:3]
                if not np.allclose(ground,abs(xyz.astype(float)@data['ground_plane_abcd'][:3]+data['ground_plane_abcd'][3])<.18):
                    raise ValueError('Ground mask does not match measured plane')
            projected,uv,depth=project(xyz,matrix(sensors['lidar']),matrix(sensors['rgb']),
                                       manifest['camera_intrinsic'],rgb.shape[1],rgb.shape[0])
            tick=time.perf_counter()
            detections=detector.detect(rgb)
            inference_ms=(time.perf_counter()-tick)*1000
            fused=associate(detections,xyz,raw_indices,ground,projected,uv,depth)
            tracker.update(fused,row['timestamp'],matrix(sensors['lidar']))
            camera,lidar=render(rgb,xyz,ground,projected,uv,fused)
            camera_path=out/'frames'/f'{token}-camera.png'
            lidar_path=out/'frames'/f'{token}-lidar.png'
            camera.save(camera_path,compress_level=1)
            lidar.save(lidar_path,compress_level=1)
            record={'sample':row['sample'],'frame':frame,'timestamp':row['timestamp'],
                'rgb_file_sha256':sha(rgb_path),'native_lidar_raw_sha256':sensors['lidar']['raw_sha256'],
                'geometric_points_npz_sha256':sha(data_path),'inference_ms':inference_ms,
                'detections':fused,'camera_sha256':sha(camera_path),'lidar_sha256':sha(lidar_path)}
            stream.write(json.dumps(record,separators=(',',':'))+'\n'); stream.flush()
            records.append(record)
            if len(records)%40==0 or len(records)==len(selected):
                print(json.dumps({'processed':len(records),'total':len(selected),
                    'elapsed_s':round(time.perf_counter()-begin,1)}),flush=True)
    if sha(__file__) != script_sha or sha(detector_source) != detector_sha:
        raise ValueError('Source changed during processing; output is not accepted')
    summary={'status':'processed_pending_independent_review','frames':len(records),
        'source_run':str(run),'source_processed':str(processed),
        'native_frame_ids':[r['frame'] for r in records],
        'source_manifest_sha256':sha(run/'manifest.json'),'source_frames_sha256':sha(run/'frames.jsonl'),
        'source_processing_manifest_sha256':sha(processed/'processing-manifest.json'),
        'model_sha256':sha(args.model),'model':'YOLOX-S official COCO ONNX',
        'detector_provenance':detector.provenance,
        'script_sha256':script_sha,'detector_script_sha256':detector_sha,
        'associations_sha256':sha(out/'associations.jsonl'),
        'uses_simulator_labels_for_detection_or_association':False,
        'uses_ego_geometry_for_preexisting_point_self_filter':True,
        'trained_image_detector_inference':True,'trained_fusion_model':False,
        'offline_processing':True,'processing_controls_vehicle':False,
        'confidence_threshold':.3,'camera_and_lidar_visible_every_frame':True,
        'detected_boxes':sum(len(r['detections']) for r in records),
        'associated_boxes':sum(d['fusion_status']=='associated' for r in records for d in r['detections']),
        'frames_with_association':sum(any(d['fusion_status']=='associated' for d in r['detections']) for r in records),
        'median_inference_ms':float(np.median([r['inference_ms'] for r in records])),
        'added_text':'Only per-object class, score, display ID and measured surface range; no titles or explanatory captions.',
        'limits':['Pretrained RGB inference plus geometric frustum association, not a jointly trained fusion network.',
                  'Observed surface range and bounds are not complete 3D object dimensions or object-center distance.',
                  'Sparse returns, occlusion and domain shift can cause missed or incorrect associations.',
                  'Display IDs use short-term geometric matching; they are not simulator actor IDs or a tracking benchmark.',
                  'Native 20 fps playback is not a claim of 20 fps online inference.']}
    (out/'fusion-manifest.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run','processed','out','model'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--start-sample',type=int,default=0)
    parser.add_argument('--end-sample',type=int)
    parser.add_argument('--threads',type=int,default=4)
    process(parser.parse_args())


if __name__=='__main__':
    main()
