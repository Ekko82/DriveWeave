"""Independent complete v3 highway action proof from saved native physics.

No Client/tick. The full same-source motion audit supplies all-pair3D OBB,
continuity, camera-opening and comfort gates. This auditor independently adds
native lane occupation, unsafe-gap waiting, body passage, every change-frame
gap, persistent moving traffic and the actual terminal pose.
"""
from pathlib import Path
import argparse
import itertools
import json
import math
import sys
import numpy as np
import carla

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from audit_native_actions import (file_sha,obb,measure_lane_gaps,unit_heading,
    angle_difference,stable_lanes,sequence_subsequence,xyz_wp)


def physical_actions(rows,plan,events,native_map):
    checks={};details={}
    def check(name,value,evidence=None):
        checks[name]=bool(value)
        if evidence is not None:details[name]=evidence
    egos={r['frame']:next(v for v in r['vehicles'] if v['ego']) for r in rows}
    byframe={r['frame']:r for r in rows}
    states=[];mismatches=[];coverage=[]
    required={s['name'] for s in plan['traffic']}
    roles={name:[] for name in required}
    for row in rows:
        ego=egos[row['frame']];pos=np.array(ego['transform']['location_m'])
        wp=native_map.get_waypoint(carla.Location(*pos));heading=unit_heading(wp)
        side=np.array([-heading[1],heading[0],0.])
        lateral=abs(float((pos-np.array(xyz_wp(wp)))@side))
        states.append({'frame':row['frame'],'time_s':row['time_s'],
            'native_road':wp.road_id,'native_lane':wp.lane_id,'native_s_m':wp.s,
            'lateral_error_m':lateral,'heading_error_deg':angle_difference(
                ego['transform']['rotation_deg'][1],wp.transform.rotation.yaw)})
        saved=ego.get('waypoint',{})
        if (saved.get('road_id'),saved.get('lane_id'))!=(wp.road_id,wp.lane_id):
            mismatches.append(row['frame'])
        yaw=math.radians(ego['transform']['rotation_deg'][1]);f=np.array([math.cos(yaw),math.sin(yaw),0.])
        side=np.array([-f[1],f[0],0.]);near=[]
        for v in row['vehicles']:
            if v['ego']:continue
            name=v['role_name']
            if name in roles:roles[name].append(v)
            delta=np.array(v['transform']['location_m'])-pos
            along=float(delta@f);lat=float(delta@side)
            if (np.linalg.norm(delta[:2])<=60 and abs(delta[2])<3 and abs(lat)<=7 and
                    angle_difference(v['transform']['rotation_deg'][1],ego['transform']['rotation_deg'][1])<=45):
                near.append({'role':name,'along_m':along,'lateral_m':lat,'speed_mps':v['speed_mps']})
        coverage.append({'frame':row['frame'],'time_s':row['time_s'],
            'moving_rear60':[n for n in near if -60<=n['along_m']<0 and n['speed_mps']>=1.],
            'moving_rear40':[n for n in near if -40<=n['along_m']<0 and n['speed_mps']>=1.],
            'moving_front60':[n for n in near if 0<n['along_m']<=60 and n['speed_mps']>=1.]})
    check('actual_ego_waypoints_recomputed_from_saved_map',not mismatches,mismatches[:10])
    stable=stable_lanes(states,.05)
    stable_order=[key for key,_ in itertools.groupby(tuple(r['key']) for r in stable)]
    check('actual_centered_native_lane_minus2_to_minus1',
        sequence_subsequence(stable_order,[(45,-2),(45,-1)]),stable)
    movement={}
    for name,trace in roles.items():
        displacement=max((math.dist(v['transform']['location_m'],trace[0]['transform']['location_m']) for v in trace),default=0.)
        movement[name]={'frames':len(trace),'displacement_from_start_m':displacement,
            'actual_max_speed_mps':max((v['speed_mps'] for v in trace),default=0.)}
    check('all_explicit_native_traffic_present_and_actually_moves',bool(required) and all(
        m['frames']==len(rows) and m['displacement_from_start_m']>5 and m['actual_max_speed_mps']>1 for m in movement.values()),movement)
    rear_fraction=sum(bool(r['moving_rear60']) for r in coverage)/len(rows)
    check('moving_rear60_presence_at_least_80_percent',rear_fraction>=.8,
        {'fraction':rear_fraction,'minimum_fraction':.8,'definition':'Actual root positions: planar radius<=60m, along[-60,0), lateral<=7m, heading<=45deg, height<3m, actual speed>=1m/s; no pixel visibility claim.',
         'rear40_presence_fraction':sum(bool(r['moving_rear40']) for r in coverage)/len(rows),
         'front60_presence_fraction':sum(bool(r['moving_front60']) for r in coverage)/len(rows),
         'opening_2s_rear60_fraction':sum(bool(r['moving_rear60']) for r in coverage[:40])/min(40,len(rows))})
    change=next((e for e in events if e.get('event')=='lane_change_started' and
        (e.get('from_road'),e.get('from_lane'),e.get('target_lane'),e.get('direction'))==(45,-2,-1,'left')),None)
    completion=next((e for e in events if change and e.get('event')=='stage_completed' and
        e.get('stage_index')==change.get('stage_index') and e['frame']>change['frame']),None)
    observations=[];unsafe_wait=[];passed=[]
    if change and completion:
        for row in rows:
            ego=egos[row['frame']]
            if row['frame']<=completion['frame']:
                measured=measure_lane_gaps(row,ego,native_map,45,-1)
                measured.update(frame=row['frame'],sample=row['sample'],time_s=row['time_s'])
                if change['frame']<=row['frame']:observations.append(measured)
                elif row['rule_controller'].get('gap_waiting') and any(
                        risk['role_name']=='target_rear' for risk in measured.get('risks',[])):
                    unsafe_wait.append(measured)
                if row['frame']<change['frame']:
                    target=next((v for v in row['vehicles'] if v['role_name']=='target_rear'),None)
                    wp=native_map.get_waypoint(carla.Location(*ego['transform']['location_m']))
                    anchor=native_map.get_waypoint_xodr(45,-1,wp.s)
                    if target and anchor and target['speed_mps']>1:
                        direction=unit_heading(anchor)
                        gap=float((obb(target)['vertices']@direction).min()-(obb(ego)['vertices']@direction).max())
                        if gap>0:passed.append({'frame':row['frame'],'time_s':row['time_s'],'body_pass_margin_m':gap})
    check('actual_unsafe_target_gap_waiting_at_least_1_second',len(unsafe_wait)>=20,
        {'actual_unsafe_samples':len(unsafe_wait),'seconds_at_20hz':len(unsafe_wait)*.05,'examples':unsafe_wait[:2]})
    check('actual_target_rear_body_passes_before_lane_change',bool(passed),passed[:1])
    check('actual_start_and_complete_change_frames_present',bool(change and completion and
        change['frame'] in byframe and completion['frame'] in byframe),{'start':change,'completion':completion})
    check('every_actual_lane_change_frame_dynamic_gaps_safe',bool(observations) and all(o['safe'] for o in observations),
        {'actual_samples':len(observations),'risk_samples':sum(not o['safe'] for o in observations),
         'risk_examples':[o for o in observations if not o['safe']][:3],
         'actual_start':observations[0] if observations else None,
         'minimum_front_body_gap_m':min((o['front']['actual_projected_bumper_gap_m'] for o in observations if o.get('front')),default=None),
         'minimum_rear_body_gap_m':min((o['rear']['actual_projected_bumper_gap_m'] for o in observations if o.get('rear')),default=None),
         'criterion':'Unchanged native-action front max(9,v_ego*1.3); rear max(12,v_rear*1.4+positive closing*3), existing numerical tolerance0.05m.'})
    goal_ref=plan['stages'][-1]['xodr_path'][-1];goal=native_map.get_waypoint_xodr(*goal_ref)
    last=egos[rows[-1]['frame']];last_state=states[-1]
    error=math.dist(last['transform']['location_m'],xyz_wp(goal))
    heading_error=angle_difference(last['transform']['rotation_deg'][1],goal.transform.rotation.yaw)
    check('actual_last_pose_reaches_terminal_native_waypoint',
        (last_state['native_road'],last_state['native_lane'])==tuple(goal_ref[:2]) and error<=2. and heading_error<=8,
        {'last_frame':rows[-1]['frame'],'native_goal':goal_ref,'actual_last_native_state':last_state,'position_error_m':error,'heading_error_deg':heading_error})
    required_stages=[s['name'] for s in plan['stages']]
    check('all_completion_events_inside_actual_capture',all(any(e.get('event')=='stage_completed' and
        e.get('stage')==name and e['frame'] in byframe for e in events) for name in required_stages),required_stages)
    return checks,details,coverage


def audit(run,execution,motion_report,out):
    run=run.resolve();out=out.resolve()
    if out==run or run in out.parents:raise ValueError('Output must not modify raw run')
    manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    plan=manifest['route_specification']
    if not plan['scenario'].startswith('v3_highway_'):raise ValueError('Dedicated v3 highway audit only')
    rows=[json.loads(s) for s in (run/'frames.jsonl').read_text(encoding='utf-8').splitlines() if s.strip()]
    if len(rows)<40:raise ValueError('At least2s of native frames required')
    events=json.loads((run/'controller-events.json').read_text(encoding='utf-8'))
    m=carla.Map(plan['map'],(run/'map.xodr').read_text(encoding='utf-8'))
    checks,details,coverage=physical_actions(rows,plan,events,m)
    hashes={n:file_sha(run/n) for n in ('manifest.json','frames.jsonl','map.xodr','collisions.json','controller-events.json')}
    proc=json.loads(execution.read_text(encoding='utf-8'));motion=json.loads(motion_report.read_text(encoding='utf-8'))
    checks['complete_exact_native_capture']=manifest.get('status')=='complete' and manifest.get('frames_captured')==len(rows) and manifest.get('fps')==20 and manifest.get('synthetic_sensor_fallback') is False
    checks['actual_parent_exit0_and_source_hash_binding']=proc.get('returncode')==0 and Path(proc.get('capture_run','')).resolve()==run and proc.get('manifest_sha256')==hashes['manifest.json'] and proc.get('frames_jsonl_sha256')==hashes['frames.jsonl']
    checks['exact_same_source_complete_motion_audit_passed']=motion.get('motion_quality_validated') is True and bool(motion.get('checks')) and all(motion['checks'].values()) and all(motion.get('input_sha256',{}).get(n)==hashes[n] for n in ('manifest.json','frames.jsonl','map.xodr','collisions.json')) and motion.get('execution_sha256')==file_sha(execution)
    t=motion.get('thresholds',{})
    checks['motion_thresholds_not_relaxed']=t.get('minimum_ego_body_clearance_m',0)>=1. and t.get('minimum_all_body_clearance_m',0)>=.75 and t.get('max_abs_windowed_jerk_mps3',math.inf)<=12. and t.get('p95_abs_windowed_jerk_mps3',math.inf)<=6. and t.get('max_windowed_acceleration_mps2',math.inf)<=4. and t.get('max_windowed_braking_mps2',math.inf)<=6.
    collisions=json.loads((run/'collisions.json').read_text(encoding='utf-8'))
    checks['native_collision_events_zero']=not collisions and manifest.get('collision_count')==0
    checks['recorded_map_matches_plan']=plan.get('source_xodr_sha256')==hashes['map.xodr']
    details['complete_motion_obb_evidence']={'motion_report':str(motion_report.resolve()),'motion_sha256':file_sha(motion_report),
        'all_vehicle_minimum_3d_obb':motion.get('all_vehicle_minimum_3d_obb'),'ego_minimum_3d_obb':motion.get('ego_minimum_3d_obb')}
    out.parent.mkdir(parents=True,exist_ok=True)
    series=out.with_name(out.stem+'-neighbor-series.jsonl')
    series.write_text(''.join(json.dumps(r)+'\n' for r in coverage),encoding='utf-8')
    report={'schema':'complete_native_v3_highway_actions','scenario_actions_validated':all(checks.values()),
        'checks':checks,'failed_checks':[n for n,v in checks.items() if not v],'details':details,
        'run':str(run),'frames':len(rows),'input_sha256':hashes,'execution_sha256':file_sha(execution),
        'motion_report_sha256':file_sha(motion_report),'auditor_sha256':file_sha(Path(__file__)),
        'neighbor_series_sha256':file_sha(series),'raw_sources_modified':False,'server_connected_or_ticked':False,
        'limitations':['Actual pixels/occlusion require sensor and video review; native local positions do not prove visibility.',
            'Full same-source motion report supplies independently computed all-pair OBB and physical quality; no physics gate is inferred from controller command labels.',
            'Sampled geometry is not a continuous collision-avoidance guarantee. No trained policy is asserted.']}
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'report':str(out),'scenario_actions_validated':report['scenario_actions_validated'],'failed_checks':report['failed_checks']}))
    return report


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True);ap.add_argument('--execution',type=Path,required=True)
    ap.add_argument('--motion-report',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();r=audit(a.run,a.execution,a.motion_report,a.out)
    raise SystemExit(0 if r['scenario_actions_validated'] else 2)
