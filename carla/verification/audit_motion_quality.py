"""Offline v2 capture motion/traffic quality audit. Never connects or ticks CARLA.

Projected vehicle boxes are geometric image candidates, not an occlusion test.
Native instance pixels, when present, are counted separately and no instance code
is assumed equal to an actor ID. Thresholds are declared project quality targets,
not legal/industry standards. Raw and windowed derivatives are both retained.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys

import carla
import numpy as np
from PIL import Image

CARLA_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CARLA_DIR))
from audit_native_actions import angle_difference, file_sha, obb, obb_signed_clearance


def numpy_scalar(value):
    if isinstance(value,np.generic):
        return value.item()
    raise TypeError(f'Unsupported report value {type(value).__name__}')


@dataclass(frozen=True)
class Thresholds:
    first_forward_speed_mps: float = 4.0
    opening_seconds: float = 2.0
    opening_moving_floor_mps: float = .5
    derivative_window_seconds: float = .5
    max_windowed_acceleration_mps2: float = 4.0
    max_windowed_braking_mps2: float = 6.0
    p95_abs_windowed_jerk_mps3: float = 6.0
    max_abs_windowed_jerk_mps3: float = 12.0
    max_throttle_rate_per_s: float = 9.0
    max_brake_rate_per_s: float = 9.0
    max_steer_rate_per_s: float = 2.4
    minimum_all_body_clearance_m: float = .75
    minimum_ego_body_clearance_m: float = 1.0
    neighbor_distance_m: float = 80.0
    projected_minimum_area_px2: float = 96.0
    opening_mean_projected_neighbors: float = 2.0
    opening_frames_with_neighbor_fraction: float = .75
    opening_frames_with_active_neighbor_fraction: float = .75
    active_neighbor_speed_mps: float = 1.0
    mask_minimum_instance_pixels: int = 20
    road_center_extra_tolerance_m: float = .35
    road_vertical_tolerance_m: float = 1.5
    speed_record_tolerance_mps: float = .05
    step_tolerance_s: float = .002


def load_thresholds(path: Path | None):
    raw = json.loads(path.read_text(encoding='utf-8')) if path else {}
    obj = Thresholds(**raw.get('thresholds', raw))
    if obj.first_forward_speed_mps < 4 or obj.opening_seconds < 2:
        raise ValueError('First speed >=4m/s and opening >=2s are mandatory v2 requirements')
    values = asdict(obj)
    if any(not math.isfinite(float(v)) or v <= 0 for v in values.values()):
        raise ValueError('All quality thresholds must be positive and finite')
    for key in ('opening_frames_with_neighbor_fraction', 'opening_frames_with_active_neighbor_fraction'):
        if values[key] > 1:
            raise ValueError(f'{key} must be <=1')
    return obj


def k_from_sensor(sensor):
    width, height, fov = int(sensor['width']), int(sensor['height']), float(sensor['fov'])
    if width <= 0 or height <= 0 or not 0 < fov < 180:
        raise ValueError('Invalid native camera dimensions/FOV')
    focal = width/(2*math.tan(math.radians(fov)/2))
    return np.array([[focal,0,width/2],[0,focal,height/2],[0,0,1.]], float)


def project_box(box, sensor, near_m=.1):
    """Clip 3D OBB edges at the optical near plane; project actual camera pose."""
    world_from_camera = np.asarray(sensor['transform']['world_from_local'], float)
    if world_from_camera.shape != (4,4) or not np.isfinite(world_from_camera).all():
        raise ValueError('Invalid native camera extrinsic')
    camera_from_world = np.linalg.inv(world_from_camera)
    def local(points):
        return (camera_from_world @ np.column_stack((points,np.ones(len(points)))).T).T[:,:3]
    vertices = local(box['vertices'])
    points = [p for p in vertices if p[0] >= near_m]
    for edge in box['edges']:
        a,b = local(edge)
        if (a[0] < near_m) != (b[0] < near_m):
            t = (near_m-a[0])/(b[0]-a[0])
            points.append(a+t*(b-a))
    if not points:
        return None
    p = np.asarray(points)
    optical = np.column_stack((p[:,1],-p[:,2],p[:,0]))
    uvh = (k_from_sensor(sensor) @ optical.T).T
    uv = uvh[:,:2]/uvh[:,2,None]
    width,height = int(sensor['width']),int(sensor['height'])
    x0,y0 = uv.min(axis=0);x1,y1 = uv.max(axis=0)
    clipped = [max(0.,float(x0)),max(0.,float(y0)),min(float(width),float(x1)),min(float(height),float(y1))]
    if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
        return None
    return {'clipped_bbox_xyxy':clipped,
        'projected_rectangle_area_px2':(clipped[2]-clipped[0])*(clipped[3]-clipped[1]),
        'minimum_positive_camera_depth_m':float(p[:,0].min()),
        'visibility':'geometric_candidate_occlusion_unknown'}


def local_polynomial_derivatives(times, values, window_s):
    """Explicit quadratic least-squares fit; centered windows, one-sided endpoints.

    Returns d(value)/dt and d²(value)/dt². No downsampling, spikes deletion or
    failure-dependent filtering. At least five real consecutive samples per fit.
    """
    times, values = np.asarray(times,float),np.asarray(values,float)
    if len(times) < 5 or not np.all(np.diff(times)>0):
        raise ValueError('Need at least five strictly ordered timestamped samples')
    first, second = [], []
    for i, t in enumerate(times):
        indexes = np.flatnonzero(abs(times-t) <= window_s/2+1e-7)
        if len(indexes)<5:
            indexes = np.arange(max(0,min(i-2,len(times)-5)),max(0,min(i-2,len(times)-5))+5)
        x = times[indexes]-t
        design = np.column_stack((np.ones(len(x)),x,x*x))
        coefficients = np.linalg.lstsq(design,values[indexes],rcond=None)[0]
        first.append(float(coefficients[1]));second.append(float(2*coefficients[2]))
    return np.asarray(first),np.asarray(second)


def summarize(values, times):
    values = np.asarray(values,float)
    index = int(np.argmax(abs(values)))
    return {'minimum':float(values.min()),'maximum':float(values.max()),
        'p95_absolute':float(np.quantile(abs(values),.95)),
        'maximum_absolute':float(abs(values[index])), 'maximum_absolute_time_s':float(times[index])}


def vehicle_mask_diagnostic(run, frame, threshold):
    """Count rendered native vehicle-instance masks; no actor identity inference."""
    sensor = frame['sensors'].get('instance')
    if sensor is None:
        return {'available':False,'reason':'no_native_instance_camera'}
    path = run/'instance'/f"{frame['frame']:08d}.png"
    raw = run/'raw_bgra'/f"instance_{frame['frame']:08d}.bin"
    if not path.is_file():
        return {'available':False,'reason':'native_instance_png_missing'}
    pixels = np.asarray(Image.open(path).convert('RGB'))
    h,w = int(sensor['height']),int(sensor['width'])
    provenance='saved_native_BGRA'
    data=raw.read_bytes() if raw.is_file() else None
    if data is None and pixels.shape==(h,w,3):
        # PNG discarded only alpha. Recover a constant alpha iff the ENTIRE
        # reconstructed byte stream matches the actual callback raw SHA. This
        # is never accepted by assumption, and no reconstructed file is saved.
        for alpha in (255,0):
            candidate=np.dstack((pixels[:,:,[2,1,0]],np.full((h,w),alpha,np.uint8))).tobytes()
            if hashlib.sha256(candidate).hexdigest()==sensor.get('raw_sha256'):
                data=candidate
                provenance=f'PNG_RGB_plus_constant_alpha_{alpha}_exact_native_raw_SHA_match'
                break
    valid = (data is not None and len(data)==h*w*4 and hashlib.sha256(data).hexdigest()==sensor.get('raw_sha256') and pixels.shape==(h,w,3))
    if valid:
        bgra = np.frombuffer(data,np.uint8).reshape(h,w,4)
        valid = bool(np.array_equal(pixels,bgra[:,:,[2,1,0]]))
    if not valid:
        return {'available':True,'native_pixel_provenance_valid':False,'png_sha256':file_sha(path)}
    code = pixels[:,:,1].astype(np.uint16)*256+pixels[:,:,2].astype(np.uint16)
    selected = np.isin(pixels[:,:,0],[14,15,16,18,19])
    identities,counts = np.unique(code[selected],return_counts=True)
    masks = [{'native_instance_code_gb':int(i),'pixels':int(n)}
             for i,n in zip(identities,counts) if i != 0 and n >= threshold]
    return {'available':True,'native_pixel_provenance_valid':True,
        'png_sha256':file_sha(path),'raw_bgra_sha256':hashlib.sha256(data).hexdigest(),
        'raw_bgra_saved':raw.is_file(),'pixel_provenance_method':provenance,
        'actual_rendered_vehicle_instance_masks':len(masks),'masks':masks,
        'individual_actor_mapping_verified':False,'ego_may_be_included':True,
        'camera':'native instance camera; this is not a chase-camera occlusion check'}


def native_route_diagnostic(native_map, route, map_name, map_sha, map_crlf_to_lf_sha=None):
    errors=[];points=0
    if route.get('map') != map_name:
        errors.append({'reason':'route_map_name_mismatch','route_map':route.get('map'),'capture_map':map_name})
    expected_source_sha=route.get('source_xodr_sha256')
    if expected_source_sha and expected_source_sha not in (map_sha,map_crlf_to_lf_sha):
        errors.append({'reason':'route_native_map_hash_mismatch'})
    for stage in route.get('stages',[]):
        previous=None
        for reference in stage.get('xodr_path',[]):
            wp = native_map.get_waypoint_xodr(int(reference[0]),int(reference[1]),float(reference[2]))
            points+=1
            if wp is None or wp.lane_type != carla.LaneType.Driving:
                errors.append({'stage':stage.get('name'),'reference':reference,'reason':'not_native_driving_waypoint'})
                continue
            point = np.array([wp.transform.location.x,wp.transform.location.y,wp.transform.location.z])
            if previous is not None and np.linalg.norm(point-previous)>8.05:
                errors.append({'stage':stage.get('name'),'reference':reference,'reason':'native_reference_spacing_exceeds_8m'})
            previous=point
        previous=None
        for reference in stage.get('path',[]):
            points+=1
            wp=native_map.get_waypoint(carla.Location(*reference))
            if wp is None or wp.lane_type != carla.LaneType.Driving:
                errors.append({'stage':stage.get('name'),'reference':reference,'reason':'world_reference_not_on_native_driving_map'})
            elif wp.transform.location.distance(carla.Location(*reference))>max(2.,wp.lane_width/2+.35):
                errors.append({'stage':stage.get('name'),'reference':reference,'reason':'world_reference_far_from_its_native_road'})
            if previous is not None and math.dist(previous,reference)>8.05:
                errors.append({'stage':stage.get('name'),'reference':reference,'reason':'world_reference_spacing_exceeds_8m'})
            previous=reference
    return {'passed':not errors and bool(route),'reference_points_checked':points,'errors':errors[:20],
        'map_source_serialization':{'actual_raw_sha256':map_sha,'actual_CRLF_to_LF_only_sha256':map_crlf_to_lf_sha,
            'expected_source_sha256':expected_source_sha,
            'comparison':'Exact bytes or CRLF-to-LF only; every other byte remains unchanged.',
            'expected_matches_raw':bool(expected_source_sha and expected_source_sha==map_sha),
            'expected_matches_CRLF_to_LF_only':bool(expected_source_sha and expected_source_sha==map_crlf_to_lf_sha)},
        'scope':'Existing native map/lane and reference density. Exact topology/action completion uses the independent scenario action auditor.'}


def audit(run: Path, execution: Path, out: Path, route_path=None, thresholds_path=None,
          density_camera='chase', verify_instance=False):
    run=run.resolve();out=out.resolve();thresholds=load_thresholds(thresholds_path)
    manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    frames=[json.loads(line) for line in (run/'frames.jsonl').read_text(encoding='utf-8').splitlines() if line.strip()]
    route=json.loads(route_path.read_text(encoding='utf-8')) if route_path else manifest.get('route_specification',{})
    collisions=json.loads((run/'collisions.json').read_text(encoding='utf-8'))
    process=json.loads(execution.read_text(encoding='utf-8'))
    if len(frames)<5:
        raise ValueError('Fewer than five actual native frames; motion derivatives cannot be audited')
    native_map=carla.Map(manifest['map'].split('/')[-1],(run/'map.xodr').read_text(encoding='utf-8'))
    hashes={name:file_sha(run/name) for name in ('manifest.json','frames.jsonl','map.xodr','collisions.json')}
    dt=float(manifest.get('fixed_delta_seconds',1/manifest['fps']))
    times=np.array([float(f['timestamp']) for f in frames]);relative=times-times[0]
    checks={};details={}
    def check(name, value, detail=None):
        checks[name]=bool(value)
        if detail is not None:details[name]=detail
    check('native_capture_complete',manifest.get('status')=='complete' and manifest.get('native_cleanup_complete') is True)
    check('source_native_frames_count_matches_manifest',len(frames)==manifest.get('frames_captured'))
    consecutive=all(b['frame']==a['frame']+1 for a,b in zip(frames,frames[1:]))
    check('consecutive_world_frames_and_timestamps',consecutive and bool(np.all(abs(np.diff(times)-dt)<=thresholds.step_tolerance_s)))
    check('actual_process_exit_zero_and_source_hashes_bound',process.get('returncode')==0 and process.get('native_process_returncode',0)==0
        and process.get('manifest_sha256')==hashes['manifest.json'] and process.get('frames_jsonl_sha256')==hashes['frames.jsonl']
        and Path(process.get('capture_run','')).resolve()==run)
    check('native_collision_events_zero',isinstance(collisions,list) and not collisions and manifest.get('collision_count')==0)
    map_crlf_to_lf_sha=hashlib.sha256((run/'map.xodr').read_bytes().replace(b'\r\n',b'\n')).hexdigest()
    route_diagnostic=native_route_diagnostic(native_map,route,manifest['map'].split('/')[-1],hashes['map.xodr'],map_crlf_to_lf_sha)
    check('route_native_road_references_valid',route_diagnostic['passed'],route_diagnostic)
    speeds=[];forward_speeds=[];velocities=[];forwards=[];controls=[];positions=[]
    series=[];road_errors=[];invalid_geometry=[];sensor_errors=[];teleport_errors=[];speed_errors=[];control_errors=[]
    minimum_all=None;minimum_ego=None;pair_count=exact_pair_count=0;overlaps=[];previous_actors={}
    masks=[]
    for index,frame in enumerate(frames):
        ego_list=[v for v in frame['vehicles'] if v.get('ego')]
        if len(ego_list)!=1:raise ValueError(f"Expected one ego at actual frame {frame['frame']}")
        ego=ego_list[0];p=np.array(ego['transform']['location_m'],float)
        velocity=np.array(ego['velocity_mps'],float);pose=np.asarray(ego['transform']['world_from_local'],float)
        forward=pose[:3,0]/np.linalg.norm(pose[:3,0]);speed=float(np.linalg.norm(velocity))
        if not np.isfinite(np.r_[p,velocity,pose.ravel()]).all():raise ValueError('Non-finite native ego state')
        speeds.append(speed);forward_speeds.append(float(velocity@forward));velocities.append(velocity);forwards.append(forward);positions.append(p)
        controls.append([float(ego['control'][k]) for k in ('throttle','brake','steer')])
        if abs(speed-float(ego['speed_mps']))>thresholds.speed_record_tolerance_mps:
            speed_errors.append({'frame':frame['frame'],'saved_speed_mps':ego['speed_mps'],'velocity_norm_mps':speed})
        if index:
            displacement=float(np.linalg.norm(p-positions[-2]));observed_dt=times[index]-times[index-1]
            bound=.20+.5*(speed+speeds[-2])*max(0,observed_dt)*1.2
            if displacement>bound:teleport_errors.append({'frame':frame['frame'],'displacement_m':displacement,'velocity_integration_bound_m':bound})
        wp=native_map.get_waypoint(carla.Location(*p));saved=ego.get('waypoint',{})
        if wp is None or wp.lane_type!=carla.LaneType.Driving:
            road_errors.append({'frame':frame['frame'],'reason':'ego_not_on_native_driving_map'})
        else:
            f=wp.transform.get_forward_vector();side=np.array([-f.y,f.x,0.]);side/=np.linalg.norm(side)
            anchor=np.array([wp.transform.location.x,wp.transform.location.y,wp.transform.location.z])
            lateral=abs(float((p-anchor)@side))
            if lateral>wp.lane_width/2+thresholds.road_center_extra_tolerance_m:
                road_errors.append({'frame':frame['frame'],'reason':'ego_outside_native_lane_extent','lateral_m':lateral,'lane_width_m':wp.lane_width})
            if abs(p[2]-anchor[2])>thresholds.road_vertical_tolerance_m:
                road_errors.append({'frame':frame['frame'],'reason':'ego_height_off_native_road',
                    'actual_height_delta_m':float(p[2]-anchor[2])})
            if saved.get('road_id')!=wp.road_id or saved.get('lane_id')!=wp.lane_id:
                other=native_map.get_waypoint_xodr(int(saved.get('road_id',0)),int(saved.get('lane_id',0)),float(saved.get('s_m',0)))
                same_join=(other is not None and wp.is_junction and other.is_junction and wp.junction_id==other.junction_id
                    and other.transform.location.distance(wp.transform.location)<=.15
                    and angle_difference(other.transform.rotation.yaw,wp.transform.rotation.yaw)<=3)
                if not same_join:road_errors.append({'frame':frame['frame'],'reason':'saved_native_waypoint_mismatch','saved':saved,'recomputed':[wp.road_id,wp.lane_id,wp.s]})
        boxes={v['id']:obb(v) for v in frame['vehicles']}
        for vehicle in frame['vehicles']:
            for key,lower,upper in (('throttle',0.,1.),('brake',0.,1.),('steer',-1.,1.)):
                value=vehicle.get('control',{}).get(key)
                if value is None or not math.isfinite(float(value)) or not lower <= float(value) <= upper:
                    control_errors.append({'frame':frame['frame'],'sample':index,'actor_id':vehicle['id'],
                        'role':vehicle.get('role_name'),'channel':key,'actual_value_repr':repr(value),
                        'required_finite_interval':[lower,upper]})
            box=boxes[vehicle['id']]
            if not np.isfinite(box['vertices']).all() or np.min(box['extent'])<=0 or not np.allclose(box['axes'].T@box['axes'],np.eye(3),atol=1e-4):
                invalid_geometry.append({'frame':frame['frame'],'actor':vehicle['id']})
        for a,b in itertools.combinations(frame['vehicles'],2):
            pair_count+=1;aa,bb=boxes[a['id']],boxes[b['id']]
            # Sphere lower bound cannot miss an overlap or any improvement to
            # either minimum. All pairs are screened, nearby pairs use exact OBB.
            lower=float(np.linalg.norm(aa['center']-bb['center'])-np.linalg.norm(aa['extent'])-np.linalg.norm(bb['extent']))
            includes_ego=bool(a.get('ego') or b.get('ego'))
            needed=max(thresholds.minimum_all_body_clearance_m,
                minimum_all['signed_clearance_m'] if minimum_all else float('inf'),
                (minimum_ego['signed_clearance_m'] if minimum_ego else float('inf')) if includes_ego else 0.,
                thresholds.minimum_ego_body_clearance_m if includes_ego else 0.)
            if lower>needed:continue
            exact_pair_count+=1;clearance=obb_signed_clearance(aa,bb)
            row={'signed_clearance_m':clearance,'sample':index,'frame':frame['frame'],'time_s':float(relative[index]),
                 'actor_ids':[a['id'],b['id']],'roles':[a.get('role_name'),b.get('role_name')]}
            if minimum_all is None or clearance<minimum_all['signed_clearance_m']:minimum_all=row
            if includes_ego and (minimum_ego is None or clearance<minimum_ego['signed_clearance_m']):minimum_ego=row
            if clearance<-1e-6:overlaps.append(row)
        camera_rows={}
        measured_motion={}
        for vehicle in frame['vehicles']:
            native_speed=float(np.linalg.norm(vehicle['velocity_mps']))
            prior=previous_actors.get(vehicle['id']);location=np.asarray(vehicle['transform']['location_m'],float)
            pose_speed=(float(np.linalg.norm(location-prior[0]))/(times[index]-prior[1]) if prior is not None and times[index]>prior[1] else None)
            measured_motion[vehicle['id']]={'native_velocity_speed_mps':native_speed,'actual_pose_interval_speed_mps':pose_speed}
            previous_actors[vehicle['id']]=(location,times[index])
        for name in ('rgb','chase'):
            sensor=frame['sensors'].get(name)
            if sensor is None:continue
            if sensor.get('frame')!=frame['frame'] or abs(float(sensor.get('timestamp',0))-times[index])>1e-5:
                sensor_errors.append({'frame':frame['frame'],'camera':name,'reason':'camera_frame_or_timestamp_differs'})
            candidates=[]
            for vehicle in frame['vehicles']:
                if vehicle.get('ego'):continue
                distance=float(np.linalg.norm(boxes[vehicle['id']]['center']-boxes[ego['id']]['center']))
                if distance>thresholds.neighbor_distance_m:continue
                projection=project_box(boxes[vehicle['id']],sensor)
                if projection is None or projection['projected_rectangle_area_px2']<thresholds.projected_minimum_area_px2:continue
                motion=measured_motion[vehicle['id']]
                active=(motion['native_velocity_speed_mps']>=thresholds.active_neighbor_speed_mps or
                        (motion['actual_pose_interval_speed_mps'] is not None and motion['actual_pose_interval_speed_mps']>=thresholds.active_neighbor_speed_mps))
                candidates.append({'actor_id':vehicle['id'],'role':vehicle.get('role_name'),'distance_m':distance,
                    **projection,**motion,'active_from_actual_state':active})
            camera_rows[name]={'projected_neighbor_candidates':len(candidates),
                'active_projected_candidates':sum(c['active_from_actual_state'] for c in candidates),'candidates':candidates}
        if verify_instance:
            masks.append({'frame':frame['frame'],**vehicle_mask_diagnostic(run,frame,thresholds.mask_minimum_instance_pixels)})
        series.append({'sample':index,'frame':frame['frame'],'timestamp_s':times[index],'time_s':relative[index],
            'actual_speed_mps':speed,'actual_forward_speed_mps':forward_speeds[-1],
            'control':{key:(value if math.isfinite(value) else repr(value)) for key,value in zip(('throttle','brake','steer'),controls[-1])},
            'camera_geometry':camera_rows})
    speeds=np.asarray(speeds);forward_speeds=np.asarray(forward_speeds);controls=np.asarray(controls)
    raw_acceleration=np.gradient(speeds,times,edge_order=2);raw_jerk=np.gradient(raw_acceleration,times,edge_order=2)
    acceleration,jerk=local_polynomial_derivatives(times,speeds,thresholds.derivative_window_seconds)
    world_acceleration=np.gradient(np.asarray(velocities),times,axis=0,edge_order=2)
    projected_world_acceleration=np.sum(world_acceleration*np.asarray(forwards),axis=1)
    control_delta=np.diff(controls,axis=0);control_rates=control_delta/np.diff(times)[:,None]
    opening=relative<thresholds.opening_seconds-1e-7
    opening_rows=[series[i] for i in np.flatnonzero(opening)]
    projected=[r['camera_geometry'].get(density_camera,{}).get('projected_neighbor_candidates',0) for r in opening_rows]
    active=[r['camera_geometry'].get(density_camera,{}).get('active_projected_candidates',0) for r in opening_rows]
    check('first_frame_actual_forward_speed_at_least_4_mps',forward_speeds[0]>=thresholds.first_forward_speed_mps,
        {'measured_mps':float(forward_speeds[0]),'required_mps':thresholds.first_forward_speed_mps})
    check('opening_two_seconds_actual_moving',relative[-1]+dt>=thresholds.opening_seconds and np.min(forward_speeds[opening])>=thresholds.opening_moving_floor_mps,
        {'duration_required_s':thresholds.opening_seconds,'observed_min_forward_speed_mps':float(np.min(forward_speeds[opening])),
         'moving_floor_mps':thresholds.opening_moving_floor_mps,'samples':int(opening.sum())})
    check('camera_calibration_samples_match_actual_world_frames',not sensor_errors,sensor_errors[:10])
    check('density_camera_available_every_opening_sample',all(density_camera in r['camera_geometry'] for r in opening_rows))
    check('opening_geometric_neighbor_density_target',bool(projected) and np.mean(projected)>=thresholds.opening_mean_projected_neighbors
        and np.mean(np.asarray(projected)>0)>=thresholds.opening_frames_with_neighbor_fraction,
        {'camera':density_camera,'mean_candidates':float(np.mean(projected)),'frames_with_candidate_fraction':float(np.mean(np.asarray(projected)>0)),
         'occlusion_tested':False,'meaning':'Only geometric scene-density target; this does not accept actual pixel visibility.'})
    check('opening_projected_neighbor_actual_activity_target',bool(active) and np.mean(np.asarray(active)>0)>=thresholds.opening_frames_with_active_neighbor_fraction,
        {'frames_with_active_candidate_fraction':float(np.mean(np.asarray(active)>0)),'active_speed_floor_mps':thresholds.active_neighbor_speed_mps,
         'kinematic_mirrors':'Actual pose displacement can prove motion even when native velocity is zero.'})
    check('windowed_longitudinal_acceleration_target',acceleration.max()<=thresholds.max_windowed_acceleration_mps2 and acceleration.min()>=-thresholds.max_windowed_braking_mps2,
        summarize(acceleration,relative))
    check('windowed_longitudinal_jerk_target',np.quantile(abs(jerk),.95)<=thresholds.p95_abs_windowed_jerk_mps3 and abs(jerk).max()<=thresholds.max_abs_windowed_jerk_mps3,
        summarize(jerk,relative))
    limits=(thresholds.max_throttle_rate_per_s,thresholds.max_brake_rate_per_s,thresholds.max_steer_rate_per_s)
    check('all_native_vehicle_controls_finite_and_in_range',not control_errors,
        {'invalid_actual_samples':len(control_errors),'examples':control_errors[:20]})
    control_diagnostics={}
    for i,key in enumerate(('throttle','brake','steer')):
        finite=np.isfinite(control_rates[:,i]);indexes=np.flatnonzero(finite)
        worst=int(indexes[np.argmax(abs(control_rates[indexes,i]))]) if len(indexes) else None
        control_diagnostics[key]={'maximum_absolute_delta':float(abs(control_delta[indexes,i]).max()) if len(indexes) else None,
            'maximum_absolute_rate_per_s':float(abs(control_rates[worst,i])) if worst is not None else None,
            'required_maximum_rate_per_s':limits[i],'maximum_rate_frame':frames[worst+1]['frame'] if worst is not None else None,
            'nonfinite_delta_samples':int((~finite).sum()),'nonfinite_values_replaced':False}
    check('actual_control_rate_targets',bool(np.isfinite(control_rates).all()) and all(abs(control_rates[:,i]).max()<=limits[i] for i in range(3)),control_diagnostics)
    check('native_speed_values_match_actual_velocity',not speed_errors,speed_errors[:10])
    check('actual_position_steps_consistent_with_velocity',not teleport_errors,teleport_errors[:10])
    check('ego_native_lane_and_road_geometry_valid',not road_errors,road_errors[:20])
    check('all_native_vehicle_obb_geometry_valid',not invalid_geometry,invalid_geometry[:10])
    check('sampled_vehicle_obb_no_overlap',not overlaps,overlaps[:20])
    check('sampled_all_vehicle_body_clearance_target',minimum_all is not None and minimum_all['signed_clearance_m']>=thresholds.minimum_all_body_clearance_m,minimum_all)
    check('sampled_ego_body_clearance_target',minimum_ego is not None and minimum_ego['signed_clearance_m']>=thresholds.minimum_ego_body_clearance_m,minimum_ego)
    if verify_instance:
        check('native_instance_pixel_provenance_valid',all(m.get('available') and m.get('native_pixel_provenance_valid') for m in masks))
    for i,row in enumerate(series):
        row.update(raw_longitudinal_acceleration_mps2=float(raw_acceleration[i]),raw_longitudinal_jerk_mps3=float(raw_jerk[i]),
            windowed_longitudinal_acceleration_mps2=float(acceleration[i]),windowed_longitudinal_jerk_mps3=float(jerk[i]),
            raw_world_acceleration_projected_on_forward_mps2=float(projected_world_acceleration[i]))
        if verify_instance:row['native_instance_pixels']=masks[i]
    out.parent.mkdir(parents=True,exist_ok=True)
    series_path=out.with_name(out.stem+'-series.jsonl')
    series_path.write_text(''.join(json.dumps(r,ensure_ascii=False,allow_nan=False,default=numpy_scalar)+'\n' for r in series),encoding='utf-8')
    report={'schema':'native_capture_motion_quality_v2','motion_quality_validated':all(checks.values()),
        'checks':checks,'failed_checks':[name for name,passed in checks.items() if not passed],
        'details':details,'run':str(run),'scenario':manifest.get('scenario'),'frames':len(frames),'fps':1/dt,
        'thresholds':asdict(thresholds),'thresholds_source':str(thresholds_path.resolve()) if thresholds_path else 'declared auditor defaults',
        'thresholds_source_sha256':file_sha(thresholds_path) if thresholds_path else None,
        'raw_actual_speed_mps':summarize(speeds,relative),'raw_longitudinal_acceleration_mps2':summarize(raw_acceleration,relative),
        'raw_longitudinal_jerk_mps3':summarize(raw_jerk,relative),
        'raw_world_acceleration_projected_on_forward_mps2':summarize(projected_world_acceleration,relative),
        'windowed_longitudinal_acceleration_mps2':summarize(acceleration,relative),'windowed_longitudinal_jerk_mps3':summarize(jerk,relative),
        'derivative_method':{'raw':'numpy gradient over exact recorded native timestamps, edge_order=2',
            'quality_gate':'quadratic least-squares of actual speed over the declared local time window, min5 consecutive native samples; endpoints one-sided',
            'window_s':thresholds.derivative_window_seconds,'adaptive_filter_or_spike_removal':False},
        'camera_density_gate':density_camera,'actual_pixel_occlusion_or_neighbor_identity_verified':False,
        'native_instance_pixel_diagnostics_enabled':verify_instance,
        'all_vehicle_minimum_3d_obb':minimum_all,'ego_minimum_3d_obb':minimum_ego,
        'all_pairs_screened':pair_count,'exact_3d_obb_comparisons':exact_pair_count,
        'geometry_method':'Existing independent full 3D OBB including native pitch/roll/yaw/local offsets; conservative sphere lower-bound only skips pairs that cannot improve either minimum or violate either threshold.',
        'geometry_source_sha256':file_sha(CARLA_DIR/'audit_native_actions.py'),
        'input_sha256':hashes,'execution_file':str(execution.resolve()),'execution_sha256':file_sha(execution),
        'route_source':str(route_path.resolve()) if route_path else 'actual capture manifest route_specification',
        'route_source_sha256':file_sha(route_path) if route_path else hashes['manifest.json'],
        'series_file':str(series_path),'series_sha256':file_sha(series_path),'auditor_sha256':file_sha(Path(__file__)),
        'limitations':['Thresholds are explicit project quality preferences; not a driving safety standard.',
            'Projected rectangle intersections do not prove pixels are visible; buildings/vehicles may occlude candidates. Native masks are separate rendered-pixel evidence and never mapped to actor IDs by assumption.',
            'Sampled OBB checks do not prove continuous-time collision avoidance.',
            'Reference/map validity does not accept a scene maneuver. Independent native action audit remains required.',
            'Saved-source hashes bind records to process evidence; they cannot authenticate arbitrary forged source data.'],
        'raw_capture_modified':False,'server_connected_or_ticked':False}
    out.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False,default=numpy_scalar)+'\n',encoding='utf-8')
    print(json.dumps({'report':str(out),'motion_quality_validated':report['motion_quality_validated'],
        'failed_checks':report['failed_checks'],'first_forward_speed_mps':float(forward_speeds[0]),
        'minimum_ego_obb_clearance_m':minimum_ego['signed_clearance_m'] if minimum_ego else None},indent=2))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--execution',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--route-json',type=Path)
    parser.add_argument('--thresholds',type=Path)
    parser.add_argument('--density-camera',choices=('rgb','chase'),default='chase')
    parser.add_argument('--verify-instance-pixels',action='store_true')
    args=parser.parse_args()
    result=audit(args.run,args.execution,args.out,args.route_json,args.thresholds,args.density_camera,args.verify_instance_pixels)
    raise SystemExit(0 if result['motion_quality_validated'] else 2)
