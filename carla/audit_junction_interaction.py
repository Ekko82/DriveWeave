"""Independently audit actual v2 junction motion; no Client/tick/raw mutation.

Requires real capture/external-exit evidence. Native waypoints, actual actor
poses, measured velocities, native 3D boxes and light-state observations are
checked; plan names, stage events and TM paths cannot prove a maneuver.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import argparse
import hashlib
import itertools
import json
import math
import carla
import numpy as np
from audit_native_actions import obb, obb_signed_clearance, file_sha, angle_difference


def inside(point, polygon):
    x, y = point[:2]; hit = False
    for i, a in enumerate(polygon):
        b = polygon[i-1]
        if (a[1] > y) != (b[1] > y) and x < (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:
            hit = not hit
    return hit


def convex_hull(points):
    pts = sorted(set((float(x), float(y)) for x,y in points))
    def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for seq, out in [(pts, lower), (pts[::-1], upper)]:
        for p in seq:
            while len(out)>1 and cross(out[-2],out[-1],p)<=0: out.pop()
            out.append(p)
    return lower[:-1]+upper[:-1]


def overlap(a, b):
    """SAT overlap of convex polygons, including boundary contact."""
    a,b=np.asarray(a,float),np.asarray(b,float)
    for points in (a,b):
        for p,q in zip(points,np.roll(points,-1,axis=0)):
            axis=np.array([p[1]-q[1],q[0]-p[0]])
            aa,bb=a@axis,b@axis
            if aa.max()<bb.min()-1e-8 or bb.max()<aa.min()-1e-8: return False
    return True


def runs(values, predicate):
    out=[]; current=[]
    for row in values:
        if predicate(row): current.append(row)
        elif current: out.append(current); current=[]
    if current: out.append(current)
    return out


def reference(native_map, stages, allowed_roads=None):
    points=[]
    for stage in stages:
        for p in stage.get('xodr_path',[]):
            if allowed_roads is not None and int(p[0]) not in allowed_roads: continue
            wp=native_map.get_waypoint_xodr(int(p[0]),int(p[1]),float(p[2]))
            if wp is None or wp.lane_type!=carla.LaneType.Driving:
                raise RuntimeError('Reference is not a native Driving waypoint')
            q=[wp.transform.location.x,wp.transform.location.y,wp.transform.location.z]
            if not points or np.linalg.norm(np.array(q)-points[-1])>1e-5: points.append(q)
    return np.asarray(points,float)


def project(point, yaw, points):
    a,b=points[:-1],points[1:];v=b-a;length=(v*v).sum(axis=1)
    valid=(length>1e-10)&(length<64)
    u=np.clip(((np.asarray(point)-a)*v).sum(axis=1)/np.maximum(length,1e-10),0,1)
    closest=a+u[:,None]*v
    d=np.linalg.norm((closest-np.asarray(point))[:,:2],axis=1)
    heading=np.degrees(np.arctan2(v[:,1],v[:,0]));error=np.abs((heading-yaw+180)%360-180)
    score=np.where(valid,d+.015*error,np.inf)
    i=int(score.argmin())
    return {'lateral_distance_m':float(d[i]),'reference_heading_error_deg':float(error[i]),
            'segment_index':i,'segment_fraction':float(u[i]),'native_segment_xyz_m':[a[i].tolist(),b[i].tolist()]}


def first_body_pass(trace):
    episodes=runs(trace,lambda q:q['body_in_gate'])
    if not episodes:return None
    occupied=episodes[0]
    after=next((q for q in trace if q['frame']>occupied[-1]['frame'] and not q['body_in_gate']),None)
    return {'first_body_gate_frame':occupied[0]['frame'],'first_body_gate_time_s':occupied[0]['time_s'],
            'last_body_gate_frame':occupied[-1]['frame'],'last_body_gate_time_s':occupied[-1]['time_s'],
            'fully_clear_frame':after['frame'] if after else None,
            'fully_clear_time_s':after['time_s'] if after else None,
            'occupied_samples':len(occupied)}


def recorded_straight_core(trace, incoming=7, connector=1140, outgoing=6):
    """Select a truly observed traversal, allowing native physical preroll.

    A recording may begin on the known straight connector. It still must show
    that actual connector and its outgoing road, and later passes the unchanged
    position/tangent/yaw proof. Neither an arbitrary turn connector nor a clip
    starting after the junction can use this fallback.
    """
    start=next((i for i,q in enumerate(trace) if q['native_road']==incoming),None)
    origin='actual_incoming_road_observed'
    if start is None and trace and trace[0]['native_road']==connector:
        start=0;origin='recording_starts_on_exact_native_straight_connector_after_physical_preroll'
    end=next((i for i,q in enumerate(trace) if start is not None and i>start and q['native_road']==outgoing),None)
    core=trace[start:end+1] if start is not None and end is not None else []
    if not any(q['native_road']==connector for q in core):core=[]
    return core, {'observation_origin':origin if core else 'required_actual_connector_and_outgoing_not_both_observed',
        'incoming_prefix_recorded':bool(core) and core[0]['native_road']==incoming,
        'required_actual_connector':connector,'required_actual_outgoing':outgoing}


def straight_motion_proof(core, points):
    """Use actual core poses/yaw, not a requested route's maneuver label."""
    middle=[q for q in core if q['junction']]
    fits=[dict(frame=q['frame'],**project(q['position_m'],q['yaw_deg'],points)) for q in middle]
    delta=angle_difference(core[0]['yaw_deg'],core[-1]['yaw_deg']) if core else None
    heading_range=max((angle_difference(q['yaw_deg'],core[0]['yaw_deg']) for q in core),default=180)
    good=bool(core) and len(fits)>=5 and delta<=10 and heading_range<=15 and all(
        q['lateral_distance_m']<=1.0 and q['reference_heading_error_deg']<=15 for q in fits)
    return bool(good), {
        'actual_core_native_road_order':[r for r,_ in itertools.groupby(q['native_road'] for q in core)],
        'actual_entry_frame':core[0]['frame'] if core else None,'actual_exit_frame':core[-1]['frame'] if core else None,
        'entry_to_exit_heading_change_deg':delta,'maximum_actual_core_heading_deviation_deg':heading_range,
        'middle_native_arc_samples':len(fits),
        'worst_samples':sorted(fits,key=lambda q:q['lateral_distance_m'],reverse=True)[:3],
    }


def signal_observations_proof(signal, frames, planned):
    actual_config=signal.get('config',{})
    if actual_config.get('green_approaches')!=planned.get('green_approaches') or actual_config.get('red_approaches')!=planned.get('red_approaches'):
        return False
    states=signal.get('desired_states',{})
    if set(states.values())!={'Red','Green'} or sum(s=='Green' for s in states.values())<2:
        return False
    observations={int(row['native_frame']):row for row in signal.get('observations',[])}
    for frame in frames:
        row=observations.get(frame['frame'])
        if row is None or row.get('pass') is not True:
            return False
        lights=row.get('lights',[])
        if len(lights)!=len(states) or {str(q['actor_id']) for q in lights}!=set(states):
            return False
        if any(q.get('desired_state')!=states[str(q['actor_id'])] or str(q.get('native_state')).rsplit('.',1)[-1]!=states[str(q['actor_id'])] for q in lights):
            return False
    return bool(frames)


def audit(run, plan, execution_path, out, signal_path=None):
    manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    execution=json.loads(execution_path.read_text(encoding='utf-8'))
    frames_path=run/'frames.jsonl'
    frames=[json.loads(line) for line in frames_path.read_text(encoding='utf-8').splitlines() if line.strip()] if frames_path.is_file() else []
    collision_path=run/'collisions.json'
    collisions=json.loads(collision_path.read_text(encoding='utf-8')) if collision_path.is_file() else None
    checks,details={},{}
    def check(name, result, evidence=None):
        checks[name]=bool(result)
        if evidence is not None:details[name]=evidence
    check('native_capture_complete_and_cleanup',manifest.get('status')=='complete' and manifest.get('native_cleanup_complete') is True)
    check('actual_external_process_exit_zero',execution.get('returncode')==0 and execution.get('native_process_returncode')==0)
    hashes={name:file_sha(run/name) if (run/name).is_file() else None for name in ['manifest.json','frames.jsonl','map.xodr','collisions.json']}
    check('required_native_input_files_present',all(hashes.values()),{'missing':[k for k,v in hashes.items() if v is None]})
    check('external_execution_hash_matches_raw',all(hashes.values()) and execution.get('manifest_sha256')==hashes['manifest.json'] and execution.get('frames_jsonl_sha256')==hashes['frames.jsonl'])
    map_path=run/'map.xodr'
    # Windows write_text translates the native LF source to CRLF. Keep strict
    # byte hashes for raw/execution provenance; accept ONLY this documented
    # serialization difference when comparing to installed-map plan evidence.
    lf_map_sha=hashlib.sha256(map_path.read_bytes().replace(b'\r\n',b'\n')).hexdigest() if map_path.is_file() else None
    expected_map_sha=plan.get('source_xodr_sha256')
    check('native_map_matches_plan',manifest['map'].rsplit('/',1)[-1]==plan['map'] and expected_map_sha in (hashes['map.xodr'],lf_map_sha),
          {'actual_raw_map_sha256':hashes['map.xodr'],'actual_CRLF_to_LF_only_sha256':lf_map_sha,
           'expected_installed_map_sha256':expected_map_sha,'comparison':'Exact bytes or CRLF-to-LF serialization only; geometry and all other bytes unchanged.'})
    check('native_ego_collision_sensor_zero',collisions==[] and manifest.get('collision_count')==0)
    check('native_tesla_source',manifest.get('ego_blueprint')=='vehicle.tesla.model3' and manifest.get('synthetic_sensor_fallback') is False)
    check('actual_capture_route_matches_audit_plan',manifest.get('route_specification')==plan)
    check('actual_frame_count_and_monotonic',len(frames)==manifest.get('frames_captured') and len(frames)>2 and all(b['frame']==a['frame']+1 and b['time_s']>a['time_s'] for a,b in zip(frames,frames[1:])))
    if len(frames)<3:
        check('physical_junction_actions_can_be_observed',False,{'actual_recorded_frames':len(frames)})
        report={'run':str(run.resolve()),'scenario_actions_validated':False,'scenario':plan['scenario'],
                'checks':checks,'failed_checks':[name for name,ok in checks.items() if not ok],
                'evidence':details,'input_sha256':hashes,'execution_sha256':file_sha(execution_path),
                'capture_manifest_modified':False,'method':'No physical maneuver can be accepted from fewer than three recorded native frames.'}
        out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps({'audit':str(out),'scenario_actions_validated':False,'failed_checks':report['failed_checks']}))
        return False
    native_map=carla.Map(plan['map'],(run/'map.xodr').read_text(encoding='utf-8'))
    gate=next(s['yield_gate']['polygon_xy'] for s in plan['stages'] if s.get('yield_gate'))
    required_roles=[t['name'] for t in plan['traffic']]
    traces={'hero':[]};traces.update({role:[] for role in required_roles})
    clearance={'signed_clearance_m':float('inf')};background_clearance={'signed_clearance_m':float('inf')}
    overlaps=[];background_overlaps=[];missing=[]
    for frame in frames:
        ego=next(v for v in frame['vehicles'] if v['ego'])
        boxes={v['id']:obb(v) for v in frame['vehicles']}
        box=boxes[ego['id']]
        for vehicle in frame['vehicles']:
            role='hero' if vehicle['ego'] else vehicle.get('role_name')
            if role in traces:
                p=vehicle['transform']['location_m'];wp=native_map.get_waypoint(carla.Location(*p))
                if wp is None:raise RuntimeError('Actual actor cannot be located in exported native map')
                vbox=boxes[vehicle['id']]
                traces[role].append({'frame':frame['frame'],'time_s':float(frame['time_s']),
                    'actor_id':vehicle['id'],'position_m':p,'yaw_deg':float(vehicle['transform']['rotation_deg'][1]),
                    'speed_mps':float(np.linalg.norm(vehicle['velocity_mps'])),
                    'native_road':int(wp.road_id),'native_lane':int(wp.lane_id),'native_s_m':float(wp.s),
                    'body_in_gate':overlap(convex_hull(vbox['vertices'][:,:2]),gate),
                    'center_in_gate':inside(p,gate),'junction':bool(wp.is_junction)})
            if vehicle['id']==ego['id']:continue
            other=boxes[vehicle['id']]
            sphere_lower_bound=float(np.linalg.norm(box['center']-other['center'])-
                np.linalg.norm(box['extent'])-np.linalg.norm(other['extent']))
            if sphere_lower_bound>=clearance['signed_clearance_m']:continue
            distance=obb_signed_clearance(box,other)
            row={'signed_clearance_m':distance,'native_frame':frame['frame'],'time_s':frame['time_s'],
                 'other_actor_id':vehicle['id'],'other_role':vehicle.get('role_name')}
            if distance<clearance['signed_clearance_m']:clearance=row
            if distance<-.005:overlaps.append(row)
        backgrounds=[v for v in frame['vehicles'] if not v['ego']]
        for a,b in itertools.combinations(backgrounds,2):
            aa,bb=boxes[a['id']],boxes[b['id']]
            lower=float(np.linalg.norm(aa['center']-bb['center'])-np.linalg.norm(aa['extent'])-np.linalg.norm(bb['extent']))
            if lower>=background_clearance['signed_clearance_m']:continue
            distance=obb_signed_clearance(aa,bb)
            row={'signed_clearance_m':distance,'native_frame':frame['frame'],'time_s':frame['time_s'],
                 'actor_ids':[a['id'],b['id']],'roles':[a.get('role_name'),b.get('role_name')]}
            if distance<background_clearance['signed_clearance_m']:background_clearance=row
            if distance<-.005:background_overlaps.append(row)
        present={v.get('role_name') for v in frame['vehicles']}
        absent=set(required_roles)-present
        if absent:missing.append({'frame':frame['frame'],'missing_roles':sorted(absent)})
    check('all_dedicated_actors_present_every_frame',not missing,{'missing_samples':len(missing),'examples':missing[:5]})
    minimum=float(plan.get('expected_evidence',{}).get('must_observe_minimum_body_clearance_m',1.5))
    check('actual_ego_3d_body_clearance',clearance['signed_clearance_m']>=minimum and not overlaps,{'required_m':minimum,'minimum':clearance,'overlap_samples':len(overlaps)})
    check('actual_background_only_3d_body_clearance',math.isfinite(background_clearance['signed_clearance_m']) and background_clearance['signed_clearance_m']>=.75 and not background_overlaps,
          {'required_m':.75,'scope':'all recorded vehicle pairs excluding ego','minimum':background_clearance,'overlap_samples':len(background_overlaps)})
    hero=traces['hero'];first=frames[0].get('rule_controller',{})
    check('rolling_recording_first_frame',hero[0]['speed_mps']>=4 and first.get('stage_index')==0,{'speed_mps':hero[0]['speed_mps'],'controller':first})
    passes={role:first_body_pass(traces[role]) for role in required_roles}
    check('both_conflict_actors_physically_pass_gate',all(p and p['fully_clear_time_s'] is not None for p in passes.values()),passes)
    complete_passes=[p['fully_clear_time_s'] for p in passes.values() if p and p['fully_clear_time_s'] is not None]
    last_clear=max(complete_passes) if len(complete_passes)==len(required_roles) else None
    stopped=runs(hero,lambda row:row['speed_mps']<.5 and (last_clear is None or row['time_s']<=last_clear))
    dt=1/float(manifest['fps'])
    stopped_proof=[{'first_frame':r[0]['frame'],'last_frame':r[-1]['frame'],'start_time_s':r[0]['time_s'],
                    'end_time_s':r[-1]['time_s'],'duration_s':r[-1]['time_s']-r[0]['time_s']+dt,
                    'max_speed_mps':max(q['speed_mps'] for q in r)} for r in stopped]
    check('actual_stopped_yield_before_last_conflict_clears',any(r['duration_s']>=.5 for r in stopped_proof),stopped_proof)
    is_left='left_turn' in plan['scenario']
    if is_left:
        incoming=6;connector=1182;outgoing=47
        entry=next((q for q in hero if angle_difference(q['yaw_deg'],hero[0]['yaw_deg'])>=15),None)
        core_ref=reference(native_map,plan['stages'],{connector})
        turn_samples=[q for q in hero if q['native_road']==connector or (q['junction'] and -60<q['position_m'][0]<-35 and -100<q['position_m'][1]<-75)]
        fit=[dict(frame=q['frame'],**project(q['position_m'],q['yaw_deg'],core_ref)) for q in turn_samples]
        check('ego_actual_native_left_arc',len(fit)>=5 and all(q['lateral_distance_m']<=1.0 and q['reference_heading_error_deg']<=15 for q in fit),{'samples':len(fit),'worst_samples':sorted(fit,key=lambda q:q['lateral_distance_m'],reverse=True)[:5]})
        for specification in plan['traffic']:
            role=specification['name'];trace=traces[role]
            core,observed_suffix=recorded_straight_core(trace)
            ref=reference(native_map,specification['stages'],{1140})
            good,evidence=straight_motion_proof(core,ref)
            evidence.update(observed_suffix)
            check(role+'_actually_straight_not_turning',good,evidence)
        if signal_path is None:signal_path=run/'signal-policy.json'
        if signal_path.is_file():
            signal=json.loads(signal_path.read_text(encoding='utf-8'))
            observations={int(row['native_frame']):row for row in signal.get('observations',[])}
            states=signal.get('desired_states',{})
            check('native_signal_evidence_bound_to_raw_manifest',manifest.get('signal_policy_sha256')==file_sha(signal_path))
            check('native_opposing_green_cross_red_every_recorded_frame',signal_observations_proof(signal,frames,plan['signal_policy']),
                  {'source':signal_path.name,'sha256':file_sha(signal_path),'observed_frames':len(observations),'desired_states':states,'global_freeze_called':signal.get('global_freeze_called')})
            check('native_signal_cleanup_requests_completed',signal.get('restoration',{}).get('requests_completed') is True)
        else:check('native_opposing_green_cross_red_every_recorded_frame',False,{'missing_signal_evidence':str(signal_path)})
    else:
        outgoing=8
        entry=next((q for q in hero if q['native_road']==38),None)
        observed=[r for r,_ in itertools.groupby(q['native_road'] for q in hero)]
        needed=[38,12,13,14,9,10,8];index=0
        for road in observed:
            if index<len(needed) and road==needed[index]:index+=1
        check('ego_actual_ring_traversal_and_exit',index==len(needed),{'actual_native_roads':observed,'critical_native_roads':needed,'short_connectors_not_required':True})
        for specification in plan['traffic']:
            role=specification['name']
            trace=traces[role]
            displacement=max((math.dist(q['position_m'],trace[0]['position_m']) for q in trace),default=0)
            ref=np.asarray([p for s in specification['stages'] for p in s.get('path',[])],float)
            if len(ref)<2:
                ref=reference(native_map,specification['stages'])
            native_residual=[]
            for p in ref:
                w=native_map.get_waypoint(carla.Location(*p))
                native_residual.append(math.dist(p,[w.transform.location.x,w.transform.location.y,w.transform.location.z]))
            fits=[dict(frame=q['frame'],**project(q['position_m'],q['yaw_deg'],ref)) for q in trace]
            # Town03 native ring's center is approximately (-.5,+.4). This is
            # used only to measure actual winding, never to generate actor poses.
            theta=np.unwrap(np.arctan2([q['position_m'][1]-.4 for q in trace],
                                       [q['position_m'][0]+.5 for q in trace]))
            angular_sweep=abs(float((theta[-1]-theta[0])*180/math.pi)) if len(theta) else 0
            check(role+'_actually_circulates',displacement>20 and angular_sweep>=120 and any(p['speed_mps']>1 for p in trace) and passes[role] is not None and max(native_residual)<=.12 and all(q['lateral_distance_m']<=1 and q['reference_heading_error_deg']<=15 for q in fits),
                  {'max_actual_displacement_m':displacement,'actual_angular_sweep_deg':angular_sweep,
                   'native_reference_max_centerline_residual_m':max(native_residual),
                   'worst_reference_tracking_samples':sorted(fits,key=lambda q:q['lateral_distance_m'],reverse=True)[:3],
                   'first_body_gate_pass':passes[role]})
        ego_ref=reference(native_map,plan['stages'])
        fits=[dict(frame=q['frame'],**project(q['position_m'],q['yaw_deg'],ego_ref)) for q in hero]
        check('ego_actual_native_ring_reference_tracking',all(q['lateral_distance_m']<=1 and q['reference_heading_error_deg']<=15 for q in fits),
              {'samples':len(fits),'worst_samples':sorted(fits,key=lambda q:q['lateral_distance_m'],reverse=True)[:5]})
    check('ego_waits_until_both_opponent_bodies_clear',entry is not None and last_clear is not None and entry['time_s']>=last_clear,{'ego_turn_or_ring_entry':entry,'last_opponent_body_clear_time_s':last_clear})
    exit_trace=[q for q in hero if q['native_road']==outgoing]
    exit_distance=sum(math.dist(a['position_m'],b['position_m']) for a,b in zip(exit_trace,exit_trace[1:]))
    if is_left:
        delta=((exit_trace[0]['yaw_deg']-hero[0]['yaw_deg']+180)%360-180) if exit_trace else None
        check('ego_actual_left_entry_exit_heading_and_lanes',hero[0]['native_road']==6 and hero[0]['native_lane']==-1 and bool(exit_trace) and all(q['native_lane']==-1 for q in exit_trace) and -110<=delta<=-70,
              {'initial_native_road_lane':[hero[0]['native_road'],hero[0]['native_lane']],'exit_native_road_lane':[47,exit_trace[0]['native_lane']] if exit_trace else None,'signed_actual_heading_change_deg':delta})
    check('actual_exit_motion_at_least_15m',exit_distance>=15,{'exit_native_road':outgoing,'actual_motion_m':exit_distance,'samples':len(exit_trace)})
    check('actual_full_native_controller_plan_completed',manifest.get('controller_plan_completed') is True and manifest.get('recording_end_reason')=='all native controller stages physically completed')
    last_stage=plan['stages'][-1];goal=native_map.get_waypoint_xodr(*last_stage['xodr_path'][-1]);p=goal.transform.location
    error=math.dist(hero[-1]['position_m'][:2],[p.x,p.y]);heading=angle_difference(hero[-1]['yaw_deg'],goal.transform.rotation.yaw)
    check('actual_terminal_pose_with_moving_tail',error<=2.0 and heading<=8 and hero[-1]['speed_mps']>.5,{'terminal_position_error_m':error,'terminal_heading_error_deg':heading,'actual_final_speed_mps':hero[-1]['speed_mps']})
    # A missing second background is a failed diversity/clearance check, not Infinity in JSON.
    if not math.isfinite(background_clearance['signed_clearance_m']):background_clearance['signed_clearance_m']=None
    if not math.isfinite(clearance['signed_clearance_m']):clearance['signed_clearance_m']=None
    report={'run':str(run.resolve()),'scenario_actions_validated':all(checks.values()),'scenario':plan['scenario'],'map':plan['map'],
            'checks':checks,'failed_checks':[name for name,ok in checks.items() if not ok], 'evidence':details,
            'input_sha256':hashes,'execution_sha256':file_sha(execution_path),
            'plan_canonical_sha256':hashlib.sha256(json.dumps(plan,sort_keys=True,separators=(',',':')).encode('utf-8')).hexdigest(),
            'capture_manifest_modified':False,
            'method':'Offline native map + actual physics pose/velocity, true body gate passage, native reference tangent and exact sampled 3D OBB clearance; no server connection.',
            'limitations':['3D box clearance is sampled at sensor frames. Native ego collision sensor separately corroborates contacts.',
                           'Stage completion labels alone cannot pass the physical route, yielding, heading and terminal-pose checks.',
                           'Map projection may report overlapping connector IDs; straight/left checks use independently evaluated native reference geometry and actual measured yaw.']}
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({'audit':str(out),'scenario_actions_validated':report['scenario_actions_validated'],'failed_checks':report['failed_checks']}))
    return report['scenario_actions_validated']


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',required=True,type=Path);p.add_argument('--plan',required=True,type=Path)
    p.add_argument('--execution',required=True,type=Path);p.add_argument('--out',required=True,type=Path)
    p.add_argument('--signal-evidence',type=Path)
    args=p.parse_args()
    good=audit(args.run,json.loads(args.plan.read_text(encoding='utf-8')),args.execution,args.out,args.signal_evidence)
    raise SystemExit(0 if good else 2)


if __name__=='__main__':main()
