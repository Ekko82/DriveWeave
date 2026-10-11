"""Audit actual native CARLA motion and OBB separation without a server.

Reads capture snapshots, exported native map and collision/controller logs.
Commands/events alone cannot satisfy physical lane, ramp or overtake checks.
Does not rewrite capture manifests or plans. Owned by raw_highway.
"""
from pathlib import Path
import argparse
import hashlib
import itertools
import json
import math
import numpy as np
import carla


SIGNS=np.array(list(itertools.product((-1.,1.),repeat=3)))
EDGE_INDEX=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(SIGNS[i]!=SIGNS[j])==1]


def file_sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def angle_difference(a,b):
    return abs((a-b+180)%360-180)


def obb(vehicle):
    b=vehicle['bounding_box'];r,p,y=b.get('rotation_rpy_deg',[0,0,0])
    local=carla.Transform(carla.Location(*b['center_local_m']),carla.Rotation(pitch=p,yaw=y,roll=r))
    pose=np.array(vehicle['transform']['world_from_local'],dtype=float)@np.array(local.get_matrix(),dtype=float)
    axes=pose[:3,:3];axes=axes/np.linalg.norm(axes,axis=0)
    extent=np.array(b['extent_m'],dtype=float)
    center=pose[:3,3]
    vertices=center+(SIGNS*extent)@axes.T
    edges=np.array([[vertices[i],vertices[j]] for i,j in EDGE_INDEX])
    return {'center':center,'axes':axes,'extent':extent,'vertices':vertices,'edges':edges}


def obb_signed_clearance(a,b):
    """Exact convex-box feature distance if separated; negative SAT overlap.

    Uses all 15 SAT axes, all vertex-to-box distances and all 144 edge pairs.
    Includes native box offsets, pitch, roll, yaw and elevations.
    """
    cross=np.cross(a['axes'].T[:,None,:],b['axes'].T[None,:,:]).reshape(-1,3)
    axes=np.vstack((a['axes'].T,b['axes'].T,cross))
    norms=np.linalg.norm(axes,axis=1);axes=axes[norms>1e-8]/norms[norms>1e-8,None]
    aa=a['vertices']@axes.T;bb=b['vertices']@axes.T
    signed=np.maximum(aa.min(axis=0)-bb.max(axis=0),bb.min(axis=0)-aa.max(axis=0))
    if np.max(signed)<=1e-7:
        overlap=np.minimum(aa.max(axis=0),bb.max(axis=0))-np.maximum(aa.min(axis=0),bb.min(axis=0))
        return -float(np.min(overlap))
    candidates=[]
    for vertices,box in ((a['vertices'],b),(b['vertices'],a)):
        q=(vertices-box['center'])@box['axes']
        candidates.append(float(np.min(np.linalg.norm(q-np.clip(q,-box['extent'],box['extent']),axis=1))))
    p=a['edges'][:,0,None,:];q=b['edges'][None,:,0,:]
    u=(a['edges'][:,1]-a['edges'][:,0])[:,None,:]
    v=(b['edges'][:,1]-b['edges'][:,0])[None,:,:]
    w=p-q
    aa=np.sum(u*u,axis=-1);bb=np.sum(u*v,axis=-1);cc=np.sum(v*v,axis=-1)
    dd=np.sum(u*w,axis=-1);ee=np.sum(v*w,axis=-1);denom=aa*cc-bb*bb
    for t,s in ((np.zeros_like(bb),np.clip(ee/cc,0,1)),(np.ones_like(bb),np.clip((ee+bb)/cc,0,1)),
                (np.clip(-dd/aa,0,1),np.zeros_like(bb)),(np.clip((bb-dd)/aa,0,1),np.ones_like(bb))):
        candidates.append(float(np.min(np.linalg.norm(w+t[...,None]*u-s[...,None]*v,axis=-1))))
    safe=np.where(abs(denom)>1e-9,denom,1)
    t=(bb*ee-cc*dd)/safe;s=(aa*ee-bb*dd)/safe
    valid=(abs(denom)>1e-9)&(t>=0)&(t<=1)&(s>=0)&(s<=1)
    distances=np.linalg.norm(w+t[...,None]*u-s[...,None]*v,axis=-1)
    if valid.any():candidates.append(float(np.min(distances[valid])))
    return min(candidates)


def unit_heading(wp):
    f=wp.transform.get_forward_vector();h=np.array([f.x,f.y,0.],float)
    return h/np.linalg.norm(h)


def measure_lane_gaps(frame,ego,native_map,road,lane):
    """Independently project real vehicle boxes into the actual target lane."""
    ego_wp=native_map.get_waypoint(carla.Location(*ego['transform']['location_m']))
    target=native_map.get_waypoint_xodr(road,lane,ego_wp.s)
    if not target:return {'error':'target_native_waypoint_missing','safe':False}
    anchor=np.array([target.transform.location.x,target.transform.location.y,target.transform.location.z])
    forward=unit_heading(target);side=np.array([-forward[1],forward[0],0.])
    ego_box=obb(ego);ego_along=(ego_box['vertices']-anchor)@forward
    front,rear,risks=None,None,[]
    speed=float(np.dot(ego['velocity_mps'],forward))
    for other in frame['vehicles']:
        if other['id']==ego['id']:continue
        box=obb(other)
        if box['vertices'][:,2].max()<ego_box['vertices'][:,2].min() or ego_box['vertices'][:,2].max()<box['vertices'][:,2].min():continue
        lateral=(box['vertices']-anchor)@side
        if lateral.min()>target.lane_width/2 or lateral.max()<-target.lane_width/2:continue
        along=(box['vertices']-anchor)@forward
        other_speed=max(0.,float(np.dot(other['velocity_mps'],forward)))
        if float(np.dot(box['center']-ego_box['center'],forward))>=0:
            gap=float(along.min()-ego_along.max());required=max(9.,max(0,speed)*1.3);relation='front'
        else:
            gap=float(ego_along.min()-along.max());required=max(12.,other_speed*1.4+max(0,other_speed-max(0,speed))*3);relation='rear'
        item={'actor_id':other['id'],'role_name':other.get('role_name'),'relation':relation,'actual_projected_bumper_gap_m':gap,'required_m':required,'other_observed_speed_mps':other_speed}
        if relation=='front' and (front is None or gap<front['actual_projected_bumper_gap_m']):front=item
        if relation=='rear' and (rear is None or gap<rear['actual_projected_bumper_gap_m']):rear=item
        if gap+0.05<required:risks.append(item)
    return {'target_road':road,'target_lane':lane,'front':front,'rear':rear,'risks':risks,'safe':not risks}


def sequence_subsequence(observed,expected):
    index=0
    for item in observed:
        if index<len(expected) and item==expected[index]:index+=1
    return index==len(expected)


def stable_lanes(states,dt,minimum_s=.4):
    runs=[];current=None
    for state in states:
        key=(state['native_road'],state['native_lane']) if state['lateral_error_m']<=.75 and state['heading_error_deg']<=8 else None
        if current and current['key']==key:
            current['end_frame']=state['frame'];current['end_time_s']=state['time_s'];current['samples']+=1
        else:
            if current and current['key'] is not None and current['samples']*dt>=minimum_s:runs.append(current)
            current={'key':key,'start_frame':state['frame'],'end_frame':state['frame'],'start_time_s':state['time_s'],'end_time_s':state['time_s'],'samples':1}
    if current and current['key'] is not None and current['samples']*dt>=minimum_s:runs.append(current)
    return runs


def xyz_wp(wp):
    p=wp.transform.location;return[p.x,p.y,p.z]


def actual_road_sequence_proof(states,frames,plan,native_map,required):
    """Resolve native get_waypoint ambiguity only on proven overlapping joins.

    CARLA gives both ramp and through-lane connector IDs to a shared merge
    centreline. Keep the raw observed IDs, and accept an extra ID only when its
    actual native centreline and ego pose coincide with the reference inside
    the SAME actual native junction. Other extra roads remain failures.
    """
    first_goal=next((i for i,s in enumerate(states) if s['native_road']==required[-1]),len(states)-1)
    relevant=states[:first_goal+1]
    reference=[]
    for stage in plan.get('stages',[]):
        if stage.get('xodr_path'):
            reference.extend(native_map.get_waypoint_xodr(*p) for p in stage['xodr_path'])
    reference=[w for w in reference if w]
    segments=[(a,b) for a,b in zip(reference,reference[1:]) if a.transform.location.distance(b.transform.location)<8]
    aliases=[];rejected=[]
    by_frame={f['frame']:f for f in frames}
    for state in relevant:
        if state['native_road'] in required:continue
        frame=by_frame[state['frame']];ego=next(v for v in frame['vehicles'] if v['ego'])
        point=np.array(ego['transform']['location_m'])
        actual=native_map.get_waypoint(carla.Location(*point))
        actual_center=np.array(xyz_wp(actual));best=None
        for a,b in segments:
            if not actual.is_junction or not any(w.is_junction and w.junction_id==actual.junction_id for w in(a,b)):continue
            # At least one endpoint must be an expected native connector.
            if not any(w.road_id in required and w.is_junction and w.junction_id==actual.junction_id for w in(a,b)):continue
            left=np.array(xyz_wp(a));right=np.array(xyz_wp(b));v=right-left;length=float(np.dot(v,v))
            if length<1e-10:continue
            u=float(np.clip(np.dot(actual_center-left,v)/length,0,1));closest=left+u*v
            delta=float(np.linalg.norm(actual_center-closest))
            physical_delta=float(np.linalg.norm(point[:2]-closest[:2]))
            yaw=math.degrees(math.atan2(v[1],v[0]));yaw_delta=angle_difference(actual.transform.rotation.yaw,yaw)
            candidate={'frame':state['frame'],'actual_road':actual.road_id,'actual_lane':actual.lane_id,'junction_id':actual.junction_id,
                'expected_native_segment':[[a.road_id,a.lane_id,a.s],[b.road_id,b.lane_id,b.s]],
                'native_centerline_delta_m':delta,'actual_ego_to_expected_centerline_m':physical_delta,'native_heading_delta_deg':yaw_delta}
            if best is None or delta<best['native_centerline_delta_m']:best=candidate
        if best and best['native_centerline_delta_m']<=.15 and best['actual_ego_to_expected_centerline_m']<=.75 and best['native_heading_delta_deg']<=3:
            aliases.append(best)
        else:rejected.append(best or {'frame':state['frame'],'actual_road':actual.road_id,'reason':'not_an_overlapping_expected_native_junction'})
    accepted_frames={a['frame'] for a in aliases}
    canonical=[r for r,_ in itertools.groupby(s['native_road'] for s in relevant if s['frame'] not in accepted_frames)]
    passed=canonical==required and not rejected
    return {'passed':passed,'raw_actual_road_order':[r for r,_ in itertools.groupby(s['native_road'] for s in states)],
        'expected_road_order':required,'critical_sequence_after_verified_shared_junction_ids':canonical,
        'shared_native_merge_samples':aliases,'unaccepted_extra_road_samples':rejected[:5],
        'criterion':'Extra native road IDs require same real junction, native centreline <=0.15m, actual ego <=0.75m and native headings <=3deg; other extra roads fail.'}


def audit(run,plan,out):
    manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    plan=plan or manifest.get('route_specification',{})
    frames=[json.loads(line) for line in (run/'frames.jsonl').read_text(encoding='utf-8').splitlines() if line.strip()]
    collisions=json.loads((run/'collisions.json').read_text(encoding='utf-8'))
    events=json.loads((run/'controller-events.json').read_text(encoding='utf-8')) if (run/'controller-events.json').exists() else []
    m=carla.Map(manifest['map'].split('/')[-1],(run/'map.xodr').read_text(encoding='utf-8'))
    dt=float(manifest.get('fixed_delta_seconds',1/manifest['fps']))
    checks={};details={};failures=[]
    def check(name,passed,detail=None):
        checks[name]=bool(passed)
        if detail is not None:details[name]=detail
        if not passed:failures.append(name)
    check('capture_complete',manifest.get('status')=='complete')
    check('captured_count_matches_log',len(frames)==manifest.get('frames_captured') and len(frames)>2,{'logged':len(frames),'manifest':manifest.get('frames_captured')})
    check('native_tesla_and_sensor_source',manifest.get('ego_blueprint')=='vehicle.tesla.model3' and manifest.get('synthetic_sensor_fallback') is False)
    recorded_sha=file_sha(run/'map.xodr')
    check('native_map_matches_plan',plan.get('map')==manifest['map'].split('/')[-1] and (not plan.get('source_xodr_sha256') or plan['source_xodr_sha256']==recorded_sha),
        {'actual_map':manifest['map'],'actual_exported_map_sha256':recorded_sha,'plan_source_xodr_sha256':plan.get('source_xodr_sha256')})
    check('recorded_collision_sensor_zero',len(collisions)==0 and manifest.get('collision_count')==0,{'sensor_events':len(collisions),'manifest_count':manifest.get('collision_count')})
    if not frames:raise RuntimeError('No captured frames to audit')
    states=[];frame_by_id={};ego_by_frame={};waypoint_mismatches=[];motion_errors=[]
    closest_all={'signed_clearance_m':float('inf')};closest_ego={'signed_clearance_m':float('inf')};overlaps=[];roles={};previous=None
    for frame in frames:
        ego=[v for v in frame['vehicles'] if v.get('ego')]
        if len(ego)!=1:raise RuntimeError(f"Expected one actual ego at frame {frame['frame']}")
        ego=ego[0];frame_by_id[frame['frame']]=frame;ego_by_frame[frame['frame']]=ego
        p=ego['transform']['location_m'];wp=m.get_waypoint(carla.Location(*p))
        heading=unit_heading(wp);offset=np.array(p)-np.array(xyz_wp(wp))
        lateral=abs(float(np.dot(offset,[-heading[1],heading[0],0])))
        state={'frame':frame['frame'],'time_s':frame['time_s'],'native_road':wp.road_id,'native_lane':wp.lane_id,'native_s_m':wp.s,
            'lateral_error_m':lateral,'heading_error_deg':angle_difference(ego['transform']['rotation_deg'][1],wp.transform.rotation.yaw),'speed_mps':ego['speed_mps']}
        states.append(state)
        saved=ego.get('waypoint')
        if not saved or saved['road_id']!=wp.road_id or saved['lane_id']!=wp.lane_id:waypoint_mismatches.append({'frame':frame['frame'],'saved':saved,'recomputed':[wp.road_id,wp.lane_id,wp.s]})
        if previous:
            old_frame,old_ego=previous
            observed_dt=frame['timestamp']-old_frame['timestamp']
            displacement=float(np.linalg.norm(np.array(p)-np.array(old_ego['transform']['location_m'])))
            velocity_bound=(ego['speed_mps']+old_ego['speed_mps'])*max(0,observed_dt)/2+.35
            if frame['frame']!=old_frame['frame']+1 or abs(observed_dt-dt)>.002 or displacement>max(.6,velocity_bound):
                motion_errors.append({'frame':frame['frame'],'world_frame_gap':frame['frame']-old_frame['frame'],'timestamp_gap_s':observed_dt,'actual_displacement_m':displacement,'velocity_bound_m':velocity_bound})
        previous=(frame,ego)
        boxes={v['id']:obb(v) for v in frame['vehicles']}
        for vehicle in frame['vehicles']:
            role=vehicle.get('role_name') or str(vehicle['id'])
            roles.setdefault(role,[]).append((frame['frame'],np.array(vehicle['transform']['location_m']),float(vehicle['speed_mps'])))
        for a,b in itertools.combinations(frame['vehicles'],2):
            clearance=obb_signed_clearance(boxes[a['id']],boxes[b['id']])
            detail={'signed_clearance_m':clearance,'frame':frame['frame'],'time_s':frame['time_s'],'actor_ids':[a['id'],b['id']],'roles':[a.get('role_name'),b.get('role_name')]}
            if clearance<closest_all['signed_clearance_m']:closest_all=detail
            if (a.get('ego') or b.get('ego')) and clearance<closest_ego['signed_clearance_m']:closest_ego=detail
            if clearance<-.005:overlaps.append(detail)
    check('actual_waypoints_match_exported_native_map',not waypoint_mismatches,{'mismatch_count':len(waypoint_mismatches),'examples':waypoint_mismatches[:5]})
    check('continuous_native_motion_consistent_with_observed_velocity',not motion_errors,{'violations':len(motion_errors),'examples':motion_errors[:5]})
    minimum=float(plan.get('expected_evidence',{}).get('minimum_car_body_clearance_m',.05))
    check('sampled_all_vehicle_3d_obb_clearance',closest_all['signed_clearance_m']>=minimum,{'required_m':minimum,'minimum':closest_all,'overlap_samples':len(overlaps)})
    details['sampled_ego_3d_obb_minimum']=closest_ego
    dynamic={role:{'samples':len(records),'actual_displacement_from_initial_m':max(float(np.linalg.norm(p-records[0][1])) for _,p,_ in records),'observed_max_speed_mps':max(v for _,_,v in records)} for role,records in roles.items() if role!='hero'}
    expected_roles=[v['name'] for v in plan.get('traffic',[]) if 'name'in v]
    check('explicit_background_traffic_actually_moves',all(role in dynamic and dynamic[role]['actual_displacement_from_initial_m']>5 and dynamic[role]['observed_max_speed_mps']>1 for role in expected_roles) and bool(expected_roles),dynamic)
    road_order=[key for key,_ in itertools.groupby(s['native_road'] for s in states)]
    stable=stable_lanes(states,dt);stable_order=[key for key,_ in itertools.groupby(tuple(r['key']) for r in stable)]
    details['actual_native_road_order']=road_order;details['actual_centered_stable_lane_runs']=stable
    scenario=plan.get('scenario',manifest.get('scenario',''));expected=plan.get('expected_evidence',{})
    if 'ramp_entry' in scenario:
        required_roads=expected.get('native_entry_road_order',[32,63,68,1159,37])
        sequence=actual_road_sequence_proof(states,frames,plan,m,required_roads)
        check('actual_native_entry_road_sequence',sequence['passed'],sequence)
        check('actual_centered_merge_into_mainline',sequence_subsequence(stable_order,[(32,-3),(37,-7),(37,-6)]),{'actual_stable_lane_order':stable_order})
        accel=[s for s in states if s['native_road']==37 and s['native_lane']==-7 and s['native_s_m']<=48]
        speed_change=(max(s['speed_mps'] for s in accel)-min(s['speed_mps'] for s in accel)) if accel else 0
        check('actual_outer_lane_acceleration',speed_change>=.5,{'actual_speed_increase_mps':speed_change,'note':'Native outer mainline lane37/-7 is used for acceleration; it is not a newly constructed dedicated lane.'})
        lane_change_expect=[(-7,-6,'left')]
    elif 'ramp_exit' in scenario:
        required_roads=expected.get('native_exit_road_order',[37,1043,70,24,3])
        sequence=actual_road_sequence_proof(states,frames,plan,m,required_roads)
        check('actual_native_exit_road_sequence',sequence['passed'],sequence)
        check('actual_centered_cross_to_exit_lane',sequence_subsequence(stable_order,[(37,-4),(37,-3),(70,-3),(3,3)]),{'actual_stable_lane_order':stable_order})
        before=[s['speed_mps'] for s in states if s['native_road']==37 and s['native_lane']==-3 and s['native_s_m']<192]
        after=[s['speed_mps'] for s in states if s['native_road']==1043 and s['native_s_m']<12]
        decrease=(max(before)-min(after)) if before and after else 0
        check('actual_speed_reduction_before_curved_exit',decrease>=.5,{'actual_speed_reduction_mps':decrease})
        initial_wp=m.get_waypoint(carla.Location(*ego_by_frame[frames[0]['frame']]['transform']['location_m']))
        ramp_headings=[ego_by_frame[s['frame']]['transform']['rotation_deg'][1] for s in states if s['native_road'] in required_roads[1:]]
        curve=max((angle_difference(initial_wp.transform.rotation.yaw,yaw) for yaw in ramp_headings),default=0.)
        check('actual_exit_heading_change',curve>=float(expected.get('must_observe_native_exit_ramp_heading_change_deg_at_least',35)),{'native_heading_change_deg':curve})
        lane_change_expect=[(-4,-3,'left')]
    elif scenario=='highway_lane_change_overtake_return':
        road=int(expected.get('road_id',40));wanted=[(road,l) for l in expected.get('ego_lane_order',[-3,-2,-3])]
        check('actual_centered_highway_lane_order',sequence_subsequence(stable_order,wanted),{'actual_stable_lane_order':stable_order,'required':wanted})
        role=expected.get('slow_lead_role','slow_lead');passing=[]
        for frame,state in zip(frames,states):
            ego=ego_by_frame[frame['frame']];lead=next((v for v in frame['vehicles'] if v.get('role_name')==role),None)
            if not lead:continue
            h=unit_heading(m.get_waypoint(carla.Location(*ego['transform']['location_m'])))
            a=obb(ego)['vertices']@h;b=obb(lead)['vertices']@h
            passing.append({'frame':frame['frame'],'time_s':frame['time_s'],'ego_lane':state['native_lane'],
                'lead_rear_ahead_of_ego_front_m':float(b.min()-a.max()),'ego_rear_ahead_of_lead_front_m':float(a.min()-b.max())})
        check('lead_actual_initially_ahead',bool(passing) and passing[0]['lead_rear_ahead_of_ego_front_m']>8,passing[0] if passing else None)
        passed=[p for p in passing if p['ego_lane']==-2 and p['ego_rear_ahead_of_lead_front_m']>2]
        check('ego_actual_body_passes_truck_in_adjacent_lane',bool(passed),passed[0] if passed else None)
        returned=[p for p in passing if p['ego_lane']==-3 and passed and p['frame']>passed[0]['frame']]
        check('truck_body_clearance_on_actual_return',bool(returned) and min(p['ego_rear_ahead_of_lead_front_m'] for p in returned)>8,
            {'minimum_return_truck_bumper_gap_m':min(p['ego_rear_ahead_of_lead_front_m'] for p in returned) if returned else None})
        lane_change_expect=[(-3,-2,'left'),(-2,-3,'right')]
    else:raise RuntimeError(f'Unsupported action scenario: {scenario}; use its dedicated audit')
    gap_evidence=[];gap_windows=[]
    change_events=[e for e in events if e.get('event')=='lane_change_started']
    for source,target,direction in lane_change_expect:
        event=next((e for e in change_events if e.get('from_lane')==source and e.get('target_lane')==target and e.get('direction')==direction),None)
        if event and event['frame'] in frame_by_id:
            evidence=measure_lane_gaps(frame_by_id[event['frame']],ego_by_frame[event['frame']],m,int(event['from_road']),target)
            evidence.update({'frame':event['frame'],'from_lane':source,'direction':direction})
            end=next((e for e in events if e.get('event')=='stage_completed' and e.get('stage_index')==event.get('stage_index') and e['frame']>event['frame']),None)
            samples=[]
            if end:
                for frame in frames:
                    if event['frame']<=frame['frame']<=end['frame']:
                        item=measure_lane_gaps(frame,ego_by_frame[frame['frame']],m,int(event['from_road']),target)
                        item['frame']=frame['frame'];samples.append(item)
            front=[s['front']['actual_projected_bumper_gap_m'] for s in samples if s.get('front')]
            rear=[s['rear']['actual_projected_bumper_gap_m'] for s in samples if s.get('rear')]
            bad=[s for s in samples if not s['safe']]
            gap_windows.append({'from_lane':source,'target_lane':target,'start_frame':event['frame'],'completion_frame':end['frame'] if end else None,
                'actual_samples':len(samples),'minimum_front_bumper_gap_m':min(front) if front else None,
                'minimum_rear_bumper_gap_m':min(rear) if rear else None,'risk_samples':len(bad),'risk_examples':bad[:3],
                'safe':bool(samples) and not bad})
        else:evidence={'safe':False,'missing_actual_event_frame':True,'from_lane':source,'target_lane':target}
        gap_evidence.append(evidence)
    check('actual_recomputed_target_lane_gaps_at_change_start',all(e['safe'] for e in gap_evidence),gap_evidence)
    check('actual_recomputed_dynamic_gaps_through_change',len(gap_windows)==len(lane_change_expect) and all(w['safe'] for w in gap_windows),gap_windows)
    last_follow=next((s for s in reversed(plan.get('stages',[])) if s['kind']=='follow_path'),None)
    if last_follow and last_follow.get('xodr_path'):
        road,lane,s=last_follow['xodr_path'][-1];goal=m.get_waypoint_xodr(int(road),int(lane),float(s))
        candidates=[]
        for frame,state in zip(frames,states):
            if state['native_road']!=road or state['native_lane']!=lane:continue
            ego=ego_by_frame[frame['frame']];p=np.array(ego['transform']['location_m'])
            gap=float(np.linalg.norm(p[:2]-np.array(xyz_wp(goal))[:2]));heading_error=angle_difference(ego['transform']['rotation_deg'][1],goal.transform.rotation.yaw)
            candidates.append({'frame':frame['frame'],'time_s':frame['time_s'],'position_error_m':gap,'heading_error_deg':heading_error,'actual_road':state['native_road'],'actual_lane':state['native_lane'],'actual_s_m':state['native_s_m'],'actual_location_m':p.tolist(),'actual_rotation_deg':ego['transform']['rotation_deg']})
        best=min(candidates,key=lambda p:p['position_error_m']) if candidates else None
        check('actual_terminal_native_position_and_heading',best is not None and best['position_error_m']<=2.0 and best['heading_error_deg']<=8,best)
    else:check('actual_terminal_native_position_and_heading',False,'No native terminal reference')
    completed=[e for e in events if e.get('event')=='stage_completed']
    required=expected.get('required_actual_completed_stages',[s['name'] for s in plan.get('stages',[])])
    check('actual_completion_events_present',all(any(e.get('stage')==name and e.get('frame')in frame_by_id for e in completed) for name in required),
        {'actual_completed':[e.get('stage') for e in completed],'required':required})
    report={'run':str(run.resolve()),'scenario':scenario,'map':manifest['map'],'scenario_actions_validated':all(checks.values()),
        'checks':checks,'failed_checks':failures,'evidence':details,
        'input_sha256':{name:file_sha(run/name) for name in ['manifest.json','frames.jsonl','map.xodr','collisions.json']},
        'method':'Independent actual pose/velocity and recomputed native waypoint checks; exact sampled 3D OBB clearance; sensor collision log corroboration. Events cannot substitute for physical actions.',
        'limitations':['OBB minimum is measured at recorded native frames, not a continuous-time clearance guarantee. Native collision sensor covers ego contacts; all recorded vehicle pairs are additionally checked for box overlap.',
            'Bounding boxes conservatively enclose native bodies; this is geometry validation rather than mesh-level collision reconstruction.',
            'Pose/velocity continuity can detect large jumps but does not independently attest every runtime code path. No server connection or simulation tick is performed by this audit.'],
        'capture_manifest_modified':False,'plan_modified':False}
    if closest_all['signed_clearance_m']==float('inf'):report['evidence']['sampled_all_vehicle_3d_obb_clearance']['minimum']=None
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({'audit':str(out.resolve()),'scenario_actions_validated':report['scenario_actions_validated'],'failed_checks':failures,'sampled_minimum_3d_obb_clearance_m':closest_all['signed_clearance_m']},indent=2))
    return report['scenario_actions_validated']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--plan',type=Path)
    parser.add_argument('--out',type=Path)
    args=parser.parse_args()
    plan=json.loads(args.plan.read_text(encoding='utf-8')) if args.plan else None
    good=audit(args.run,plan,args.out or args.run/'action-audit.json')
    raise SystemExit(0 if good else 2)


if __name__=='__main__':main()
