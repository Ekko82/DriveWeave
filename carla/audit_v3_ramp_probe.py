"""Read-only physical ramp-probe audit; no Client, tick or camera claims.

Native poses and boxes supply the action/gap evidence. The unchanged formal
motion-auditor derivative and clearance thresholds supply a diagnostic physical
gate after an explicitly identified candidate hot-start observation. A probe
cannot pass the formal camera/sensor/media motion gate.
"""
from pathlib import Path
import argparse
from dataclasses import asdict
import itertools
import json
import math
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE/'verification'))
import carla
import numpy as np
from audit_native_actions import (obb, obb_signed_clearance, measure_lane_gaps,
    angle_difference, unit_heading, xyz_wp, stable_lanes, sequence_subsequence,
    actual_road_sequence_proof, file_sha)
from audit_motion_quality import Thresholds, local_polynomial_derivatives, summarize


def geometric_neighbors(frame, ego, radius, *, same_native_lane=False):
    """Actual local positions, not rendered visibility or route-arc coverage."""
    pos = np.asarray(ego['transform']['location_m'], float)
    yaw = math.radians(ego['transform']['rotation_deg'][1])
    forward = np.array([math.cos(yaw), math.sin(yaw), 0.])
    side = np.array([-forward[1], forward[0], 0.])
    found = []
    for vehicle in frame['vehicles']:
        if vehicle['id'] == ego['id']:
            continue
        offset = np.asarray(vehicle['transform']['location_m'])-pos
        angle = angle_difference(vehicle['transform']['rotation_deg'][1], ego['transform']['rotation_deg'][1])
        native_same = (vehicle['waypoint']['road_id'], vehicle['waypoint']['lane_id']) == (ego['waypoint']['road_id'], ego['waypoint']['lane_id'])
        if (np.linalg.norm(offset[:2]) <= radius and abs(offset[2]) < 2.5
                and abs(float(offset@side)) <= (2. if same_native_lane else 7.)
                and angle <= 30 and (not same_native_lane or native_same)):
            found.append({'role':vehicle.get('role_name'), 'actor_id':vehicle['id'],
                'along_m':float(offset@forward), 'lateral_m':float(offset@side),
                'actual_speed_mps':float(vehicle['speed_mps']), 'same_native_lane':native_same})
    return found


def candidate_hot_start(rows, plan):
    """Exact declared observation; preserves all earlier rest-start samples."""
    lower = float(plan.get('preroll_min_seconds', 3.))
    upper = float(plan.get('preroll_max_seconds', 18.))
    for index, row in enumerate(rows):
        if (lower <= row['time_s'] <= upper
                and row['rule_controller'].get('stage_index') == 0
                and row.get('traffic_readiness', {}).get('ready') is True):
            return index
    return None


def audit(run, out, execution=None):
    run, out = run.resolve(), out.resolve()
    if out == run or run in out.parents:
        raise ValueError('Report must be outside immutable probe directory')
    plan = json.loads((run/'plan.json').read_text(encoding='utf-8'))
    summary = json.loads((run/'summary.json').read_text(encoding='utf-8'))
    rows = [json.loads(line) for line in (run/'frames.jsonl').read_text(encoding='utf-8').splitlines() if line]
    collisions = json.loads((run/'collisions.json').read_text(encoding='utf-8'))
    if len(rows) < 5:
        raise ValueError('At least five actual ordered snapshots required')
    scene = 'entry' if 'ramp_entry' in plan['scenario'] else 'exit' if 'ramp_exit' in plan['scenario'] else None
    if scene is None:
        raise ValueError('Only native entry/exit physical probes are supported')
    native_map = carla.Map(plan['map'], (run/'map.xodr').read_text(encoding='utf-8'))
    threshold = Thresholds()
    checks, details = {}, {}
    def check(name, passed, evidence=None):
        checks[name] = bool(passed)
        if evidence is not None:
            details[name] = evidence
    source_names = ['plan.json','summary.json','physics-complete.json','frames.jsonl','map.xodr','collisions.json']
    hashes = {name:file_sha(run/name) for name in source_names}
    executed = {name:file_sha(run/('executed-'+name)) for name in summary['source_sha256']}
    check('executed_sources_match_recorded_sha', executed == summary['source_sha256'], executed)
    check('diagnostic_source_complete', summary.get('diagnostic_only') is True and summary.get('accepted_capture') is False
          and summary.get('error') is None and summary.get('cleanup_complete') is True
          and summary.get('frames') == len(rows))
    check('recorded_ego_collision_sensor_zero', len(collisions) == summary.get('collisions') == 0)
    times = np.array([r['timestamp'] for r in rows], float)
    dt = float(np.median(np.diff(times)))
    check('consecutive_physics_snapshots', all(b['frame'] == a['frame']+1 and b['sample'] == a['sample']+1 for a,b in zip(rows,rows[1:]))
          and np.all(abs(np.diff(times)-.05) <= threshold.step_tolerance_s))
    ego_by_frame, frame_by_id, states, roles, series = {}, {}, [], {}, []
    minimum_all, minimum_ego, overlaps, map_errors, continuity_errors = None, None, [], [], []
    previous = None
    for row in rows:
        candidates = [v for v in row['vehicles'] if v.get('ego')]
        if len(candidates) != 1:
            raise ValueError('Exactly one actual native ego is required')
        ego = candidates[0]; p = np.array(ego['transform']['location_m'],float)
        ego_by_frame[row['frame']] = ego; frame_by_id[row['frame']] = row
        wp = native_map.get_waypoint(carla.Location(*p))
        heading = unit_heading(wp); off = p-np.array(xyz_wp(wp))
        state = {'frame':row['frame'], 'time_s':row['time_s'], 'native_road':wp.road_id,
            'native_lane':wp.lane_id, 'native_s_m':wp.s,
            'lateral_error_m':abs(float(off@np.array([-heading[1],heading[0],0.]))),
            'heading_error_deg':angle_difference(ego['transform']['rotation_deg'][1],wp.transform.rotation.yaw),
            'speed_mps':ego['speed_mps']}
        states.append(state)
        if (wp.road_id,wp.lane_id) != (ego['waypoint']['road_id'],ego['waypoint']['lane_id']):
            map_errors.append({'sample':row['sample'],'saved':ego['waypoint'],'recomputed':[wp.road_id,wp.lane_id,wp.s]})
        if previous:
            old, vehicle = previous
            bound = (ego['speed_mps']+vehicle['speed_mps'])*(row['timestamp']-old['timestamp'])/2+.35
            if np.linalg.norm(p-np.asarray(vehicle['transform']['location_m'])) > max(.6,bound):
                continuity_errors.append(row['sample'])
        previous = row, ego
        boxes = {v['id']:obb(v) for v in row['vehicles']}
        for a,b in itertools.combinations(row['vehicles'],2):
            x,y = boxes[a['id']],boxes[b['id']]
            lower = float(np.linalg.norm(x['center']-y['center'])-np.linalg.norm(x['extent'])-np.linalg.norm(y['extent']))
            includes_ego = bool(a.get('ego') or b.get('ego'))
            best = minimum_all['signed_clearance_m'] if minimum_all else math.inf
            best_ego = minimum_ego['signed_clearance_m'] if minimum_ego else math.inf
            if lower > max(threshold.minimum_all_body_clearance_m,best) and (not includes_ego or lower>max(threshold.minimum_ego_body_clearance_m,best_ego)):
                continue
            clearance = obb_signed_clearance(x,y)
            item = {'signed_clearance_m':clearance,'sample':row['sample'],'frame':row['frame'],
                'time_s':row['time_s'],'roles':[a.get('role_name'),b.get('role_name')]}
            if minimum_all is None or clearance < minimum_all['signed_clearance_m']:
                minimum_all = item
            if includes_ego and (minimum_ego is None or clearance < minimum_ego['signed_clearance_m']):
                minimum_ego = item
            if clearance < -.005:
                overlaps.append(item)
        for vehicle in row['vehicles']:
            roles.setdefault(vehicle['role_name'],[]).append({'sample':row['sample'],'frame':row['frame'],
                'position_m':vehicle['transform']['location_m'], 'waypoint':vehicle['waypoint'],
                'actual_speed_mps':vehicle['speed_mps']})
        neighbor60 = geometric_neighbors(row,ego,60.)
        rear40 = geometric_neighbors(row,ego,40.,same_native_lane=True)
        series.append({'sample':row['sample'],'frame':row['frame'],'time_s':row['time_s'],
            'ego_actual_speed_mps':ego['speed_mps'],'front_and_rear_60m_actual_neighbors':neighbor60,
            'rear40_same_native_lane_roles':[v['role'] for v in rear40 if v['along_m']<0 and v['actual_speed_mps']>=1.],
            'rear60_roles':[v['role'] for v in neighbor60 if v['along_m']<0 and v['actual_speed_mps']>=1.]})
    check('actual_native_waypoints_match_export', not map_errors, map_errors[:5])
    check('pose_motion_consistent_with_actual_velocity', not continuity_errors, continuity_errors[:5])
    check('all_sampled_3d_obb_no_overlap', not overlaps, overlaps[:10])
    check('all_vehicle_body_clearance_target',minimum_all['signed_clearance_m']>=threshold.minimum_all_body_clearance_m,minimum_all)
    check('ego_body_clearance_target',minimum_ego['signed_clearance_m']>=threshold.minimum_ego_body_clearance_m,minimum_ego)
    ego_id = ego_by_frame[rows[0]['frame']]['id']
    events = summary['controller_events'].get(str(ego_id),[])
    required_roads = [32,63,68,1159,37] if scene=='entry' else [37,1043,70,24,3]
    road_proof = actual_road_sequence_proof(states,rows,plan,native_map,required_roads)
    check('actual_native_ramp_road_sequence',road_proof['passed'],road_proof)
    stable = stable_lanes(states,dt)
    expected = [(37,-7),(37,-6)] if scene=='entry' else [(37,-4),(37,-3),(70,-3),(3,3)]
    check('actual_centered_native_lane_sequence',sequence_subsequence([tuple(x['key']) for x in stable],expected),stable)
    completed = {event['stage_index'] for event in events if event.get('event')=='stage_completed' and event['frame'] in frame_by_id}
    check('all_actual_stages_completed',completed==set(range(len(plan['stages']))) and summary.get('end_reason')=='native_ego_plan_completed',events)
    road,lane,s = plan['stages'][-1]['xodr_path'][-1]
    goal = native_map.get_waypoint_xodr(road,lane,s)
    final = ego_by_frame[rows[-1]['frame']]
    end_error = float(np.linalg.norm(np.array(final['transform']['location_m'])[:2]-np.array(xyz_wp(goal))[:2]))
    end_heading = angle_difference(final['transform']['rotation_deg'][1],goal.transform.rotation.yaw)
    check('actual_terminal_native_pose', (states[-1]['native_road'],states[-1]['native_lane'])==(road,lane)
          and end_error<=2. and end_heading<=8.,{'position_error_m':end_error,'heading_error_deg':end_heading})
    wait_rows = [r for r in rows if r['rule_controller'].get('gap_waiting')]
    target_lane = -6 if scene=='entry' else -3
    real_refusals = [dict(sample=r['sample'],frame=r['frame'],**measure_lane_gaps(r,ego_by_frame[r['frame']],native_map,37,target_lane)) for r in wait_rows]
    check('actual_target_gap_refusal_demonstrated',bool(real_refusals) and any(r['risks'] for r in real_refusals),
          {'wait_samples':len(wait_rows),'duration_s':len(wait_rows)*dt,'first':real_refusals[0] if real_refusals else None})
    changes = [event for event in events if event.get('event')=='lane_change_started']
    change_evidence = []
    for change in changes:
        finish = next((e for e in events if e.get('event')=='stage_completed' and e['stage_index']==change['stage_index'] and e['frame']>change['frame']),None)
        evidence = []
        if finish:
            for row in rows:
                if change['frame']<=row['frame']<=finish['frame']:
                    measured = measure_lane_gaps(row,ego_by_frame[row['frame']],native_map,change['from_road'],change['target_lane'])
                    evidence.append({'sample':row['sample'],'frame':row['frame'],**measured})
        origin = frame_by_id.get(change['frame']); actual_wp = native_map.get_waypoint(carla.Location(*ego_by_frame[change['frame']]['transform']['location_m'])) if origin else None
        permission = str(actual_wp.lane_change).split('.')[-1] if actual_wp else ''
        change_evidence.append({'event':change,'actual_native_marking_legal':actual_wp is not None and not actual_wp.is_junction and permission in ['Both',change['direction'].capitalize()],
            'completion_frame':finish['frame'] if finish else None,'sample_count':len(evidence),
            'all_gap_samples_safe':bool(evidence) and all(x['safe'] for x in evidence),
            'first':evidence[0] if evidence else None,
            'risk_samples':[x for x in evidence if not x['safe']]})
    check('actual_safe_gap_release_and_whole_change',len(change_evidence)==1 and all(e['actual_native_marking_legal'] and e['all_gap_samples_safe'] for e in change_evidence),change_evidence)
    hot = candidate_hot_start(rows,plan)
    check('actual_hot_start_candidate_with_ready_traffic',hot is not None,{'sample':hot,'time_s':rows[hot]['time_s'] if hot is not None else None,
        'chosen_by':'first actual readiness observation within declared preroll bounds while stage0; this is not a captured camera start'})
    motion = None
    if hot is not None and len(rows)-hot>=5:
        speed = np.array([ego_by_frame[r['frame']]['speed_mps'] for r in rows[hot:]])
        t = times[hot:];acc,jerk=local_polynomial_derivatives(t,speed,threshold.derivative_window_seconds)
        raw_acc=np.gradient(speed,t,edge_order=2);raw_jerk=np.gradient(raw_acc,t,edge_order=2)
        motion = {'sample_start':hot,'sample_end_exclusive':len(rows),'earlier_samples_retained':hot,
            'windowed_acceleration_mps2':summarize(acc,t-t[0]),'windowed_jerk_mps3':summarize(jerk,t-t[0]),
            'raw_acceleration_mps2':summarize(raw_acc,t-t[0]),'raw_jerk_mps3':summarize(raw_jerk,t-t[0]),
            'formal_camera_sensor_media_gate_passed':False,'adaptive_spike_removal':False}
        check('diagnostic_hot_start_acceleration_target',acc.max()<=threshold.max_windowed_acceleration_mps2 and acc.min()>=-threshold.max_windowed_braking_mps2,motion['windowed_acceleration_mps2'])
        check('diagnostic_hot_start_jerk_target',np.quantile(abs(jerk),.95)<=threshold.p95_abs_windowed_jerk_mps3 and abs(jerk).max()<=threshold.max_abs_windowed_jerk_mps3,motion['windowed_jerk_mps3'])
    dynamics = {}
    for role, observed in roles.items():
        if role=='hero':continue
        start=np.array(observed[0]['position_m']);distance=max(float(np.linalg.norm(np.array(v['position_m'])-start)) for v in observed)
        dynamics[role]={'samples':len(observed),'maximum_actual_displacement_from_start_m':distance,
            'maximum_actual_speed_mps':max(v['actual_speed_mps'] for v in observed),
            'moving_fraction_over_whole_probe':sum(v['actual_speed_mps']>=1 for v in observed)/len(observed)}
    expected_roles=[r['name'] for r in plan['traffic']]
    check('explicit_traffic_actually_moving',all(role in dynamics and dynamics[role]['maximum_actual_displacement_from_start_m']>5 and dynamics[role]['maximum_actual_speed_mps']>1 for role in expected_roles),dynamics)
    start=hot if hot is not None else 0; observations=series[start:]
    coverage={'range_sample_start':start,'range_sample_end_exclusive':len(rows),
        'actual_rear60_two_moving_fraction':sum(len(r['rear60_roles'])>=2 for r in observations)/len(observations),
        'actual_rear40_same_native_lane_any_moving_fraction':sum(bool(r['rear40_same_native_lane_roles']) for r in observations)/len(observations),
        'explicit_rear_roles':{role:{'rear60_moving_fraction':sum(role in r['rear60_roles'] for r in observations)/len(observations),
            'rear40_same_native_lane_moving_fraction':sum(role in r['rear40_same_native_lane_roles'] for r in observations)/len(observations)} for role in [scene+'_rear_1',scene+'_rear_2']},
        'camera_visibility_verified':False,'curved_route_following_fraction_claimed':False,
        'method':'Actual pose tangent60m/corridor7m/same-heading30deg; rear40 additionally requires same native road/lane and2m lateral band. Curved-route arc following requires a separate metric.'}
    receipt=None
    if execution:
        receipt=json.loads(execution.read_text(encoding='utf-8'))
        receipt_valid=(receipt.get('returncode')==0 and Path(receipt.get('capture_run',receipt.get('probe_run',''))).resolve()==run
            and receipt.get('frames_jsonl_sha256')==hashes['frames.jsonl'])
        check('actual_external_process_exit_receipt',receipt_valid,{'source_sha256':file_sha(execution)})
    report={'schema_version':1,'run':str(run),'scene':scene,'diagnostic_only':True,
        'physical_probe_checks_passed':all(checks.values()),'formal_motion_quality_passed':False,'accepted_capture':False,
        'process_exit_independently_verified':receipt is not None and checks.get('actual_external_process_exit_receipt',False),
        'checks':checks,'failed_checks':[name for name,p in checks.items() if not p], 'evidence':details,
        'hot_start_motion':motion,'actual_geometric_traffic_coverage':coverage,
        'thresholds_unchanged_from_formal_motion_auditor':asdict(threshold),'input_sha256':hashes,
        'auditor_dependencies_sha256':{p.name:file_sha(p) for p in [HERE/'audit_native_actions.py',HERE/'verification/audit_motion_quality.py']},
        'map_sha256_crlf_to_lf':__import__('hashlib').sha256((run/'map.xodr').read_bytes().replace(b'\r\n',b'\n')).hexdigest(),
        'limitations':['Probe has no cameras: rendered density, image quality, sensor alignment and formal hot-start capture are unverified.',
            'An absent external process receipt is reported as unknown, not invented exit0. Source summary cleanup is checked separately.',
            'Recorded frame OBB and gap evidence does not establish continuous-time separation. No raw data or thresholds were modified.']}
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    out.with_suffix('.series.jsonl').write_text(''.join(json.dumps(row,allow_nan=False)+'\n' for row in series),encoding='utf-8')
    print(json.dumps({'report':str(out),'physical_probe_checks_passed':report['physical_probe_checks_passed'],
        'formal_motion_quality_passed':False,'failed_checks':report['failed_checks'],
        'minimum_all_clearance_m':minimum_all['signed_clearance_m'],'minimum_ego_clearance_m':minimum_ego['signed_clearance_m'],
        'rear60_two_fraction':coverage['actual_rear60_two_moving_fraction']}),flush=True)
    return report


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--execution',type=Path)
    args=parser.parse_args()
    result=audit(args.run,args.out,args.execution)
    raise SystemExit(0 if result['physical_probe_checks_passed'] else 1)
