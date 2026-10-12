"""Independent v3 role-aware junction audit; no server or source mutation.

The unchanged v2 motion auditor remains mandatory. Opponents, followers and
second-entry traffic have separate physical proofs, while OBB coverage includes
every recorded vehicle pair. Event labels alone never prove a maneuver.
"""
from pathlib import Path
import argparse
import hashlib
import itertools
import json
import math
import sys

ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT), str(ROOT / 'verification')]
import carla
import numpy as np
from audit_native_actions import obb, obb_signed_clearance, file_sha, angle_difference
from audit_junction_interaction import (reference, project, convex_hull, overlap,
    first_body_pass, recorded_straight_core, straight_motion_proof, runs,
    signal_observations_proof)
from audit_motion_quality import audit as audit_motion


def role_groups(plan):
    evidence = plan['expected_evidence']
    groups = {'opposing': list(evidence.get('opposing_roles', [])),
              'circulating': list(evidence.get('circulating_roles', [])),
              'followers': list(evidence.get('following_queue_roles', [])),
              'second_entry': [evidence['second_entry_role']] if evidence.get('second_entry_role') else []}
    flat = [role for values in groups.values() for role in values]
    names = [spec['name'] for spec in plan['traffic']]
    if len(set(flat)) != len(flat) or len(set(names)) != len(names) or set(flat) != set(names):
        raise ValueError('Every explicit actor needs one unique physical audit role')
    if bool(groups['opposing']) == bool(groups['circulating']) or len(groups['followers']) != 2:
        raise ValueError('Expected one junction conflict group and two dedicated followers')
    return groups


def gate_interaction(actor, opponents):
    """Compare actual projected body occupation, retaining all repeated episodes.

    A center outside the polygon never counts as complete body clearance. A
    later stream is reported only if its body actually enters after ego leaves.
    """
    by_frame = {q['frame']: q for q in actor}
    occupied = [q for q in actor if q['body_in_gate']]
    conflicts = []
    observed_before, observed_after = [], []
    if not occupied:
        return {'body_pass_observed': False, 'no_simultaneous_body_occupancy': False,
                'gap_between_streams_observed': False, 'simultaneous_samples': []}
    first, last = occupied[0], occupied[-1]
    for role, trace in opponents.items():
        if any(q['body_in_gate'] and q['frame'] < first['frame'] for q in trace):
            observed_before.append(role)
        if any(q['body_in_gate'] and q['frame'] > last['frame'] for q in trace):
            observed_after.append(role)
        for q in trace:
            if q['body_in_gate'] and by_frame.get(q['frame'], {}).get('body_in_gate'):
                conflicts.append({'frame': q['frame'], 'time_s': q['time_s'], 'role': role})
    return {'body_pass_observed': True, 'ego_first_body_gate_time_s': first['time_s'],
            'ego_last_body_gate_time_s': last['time_s'],
            'no_simultaneous_body_occupancy': not conflicts,
            'simultaneous_samples': conflicts[:20], 'simultaneous_sample_count': len(conflicts),
            'opponent_roles_observed_before': observed_before,
            'opponent_roles_observed_after': observed_after,
            'gap_between_streams_observed': bool(observed_before and observed_after and not conflicts),
            'observed_pattern': 'between_observed_streams' if observed_before and observed_after and not conflicts
                else 'all_observed_conflict_traffic_cleared_before_entry' if observed_before and not conflicts
                else 'not_proven'}


def trace_tracking(trace, native_reference):
    fits = [dict(frame=q['frame'], time_s=q['time_s'], **project(q['position_m'], q['yaw_deg'], native_reference)) for q in trace]
    return {'passed': bool(fits) and all(q['lateral_distance_m'] <= 1. and q['reference_heading_error_deg'] <= 15. for q in fits),
            'maximum_lateral_m': max((q['lateral_distance_m'] for q in fits), default=None),
            'maximum_heading_error_deg': max((q['reference_heading_error_deg'] for q in fits), default=None),
            'worst_lateral_samples': sorted(fits, key=lambda q: q['lateral_distance_m'], reverse=True)[:3]}


def stopped_episodes(trace):
    return [{'first_frame': episode[0]['frame'], 'last_frame': episode[-1]['frame'],
             'start_time_s': episode[0]['time_s'], 'end_time_s': episode[-1]['time_s'],
             'duration_s': episode[-1]['time_s'] - episode[0]['time_s'] + .05,
             'last_position_m': episode[-1]['position_m']}
            for episode in runs(trace, lambda q: q['speed_mps'] <= .1)]


def native_reference_checked(native_map, stages):
    # No fallback to nearest lane for a claimed native reference.
    path = []
    for stage in stages:
        if not stage.get('xodr_path'):
            raise ValueError('V3 audit requires explicit native xodr reference for every actor stage')
        prior = None
        for point in stage['xodr_path']:
            wp = native_map.get_waypoint_xodr(int(point[0]), int(point[1]), float(point[2]))
            if wp is None or wp.lane_type != carla.LaneType.Driving:
                raise ValueError(f'Non-native driving reference {point}')
            if prior is not None:
                distance = prior.transform.location.distance(wp.transform.location)
                if distance > 8.05:
                    raise ValueError('Disconnected native reference spacing')
                if (wp.road_id, wp.lane_id) != (prior.road_id, prior.lane_id):
                    candidates = [v for step in (max(.001, distance), max(.001, distance-.05), distance+.1)
                                  for v in prior.next(step)]
                    if not any((v.road_id, v.lane_id) == (wp.road_id, wp.lane_id) for v in candidates):
                        raise ValueError('Reference road boundary is not a native successor')
            prior = wp
        path.extend(stage['xodr_path'])
    # Stage seams are evaluated as part of the combined true centerline.
    for a, b in zip(stages, stages[1:]):
        aa = native_map.get_waypoint_xodr(*a['xodr_path'][-1]).transform.location
        bb = native_map.get_waypoint_xodr(*b['xodr_path'][0]).transform.location
        if aa.distance(bb) > 1.:
            raise ValueError('Discontinuous native stage seam')
    return reference(native_map, stages)


def audit(run, execution_path, out, plan_path=None, motion_path=None):
    run, execution_path, out = Path(run).resolve(), Path(execution_path).resolve(), Path(out).resolve()
    manifest = json.loads((run / 'manifest.json').read_text(encoding='utf-8'))
    execution = json.loads(execution_path.read_text(encoding='utf-8'))
    plan = json.loads(Path(plan_path).read_text(encoding='utf-8')) if plan_path else manifest['route_specification']
    rows = [json.loads(line) for line in (run / 'frames.jsonl').read_text(encoding='utf-8').splitlines() if line.strip()]
    hashes = {name: file_sha(run / name) for name in ['manifest.json', 'frames.jsonl', 'map.xodr', 'collisions.json', 'controller-events.json']}
    checks, details = {}, {}
    def check(name, passed, evidence=None):
        checks[name] = bool(passed)
        if evidence is not None:
            details[name] = evidence
    check('complete_native_capture_and_cleanup', manifest.get('status') == 'complete' and manifest.get('native_cleanup_complete') is True)
    check('actual_process_exit_and_source_hashes', execution.get('returncode') == 0 and execution.get('native_process_returncode') == 0
        and execution.get('manifest_sha256') == hashes['manifest.json']
        and execution.get('frames_jsonl_sha256') == hashes['frames.jsonl']
        and Path(execution.get('capture_run', '')).resolve() == run)
    check('exact_plan_bound_to_source_manifest', manifest.get('route_specification') == plan)
    check('native_tesla_and_no_sensor_substitution', manifest.get('ego_blueprint') == 'vehicle.tesla.model3'
        and manifest.get('synthetic_sensor_fallback') is False)
    check('actual_frames_count_and_sequence', len(rows) == manifest.get('frames_captured') and len(rows) >= 5
        and all(b['frame'] == a['frame'] + 1 and abs(b['timestamp'] - a['timestamp'] - .05) <= .002 for a, b in zip(rows, rows[1:])))
    collisions = json.loads((run / 'collisions.json').read_text())
    check('native_collision_sensor_zero', collisions == [] and manifest.get('collision_count') == 0)
    groups = role_groups(plan)
    check('explicit_actors_classified_once', True, groups)
    if motion_path is None:
        motion_path = out.with_name(out.stem + '-motion.json')
        motion = audit_motion(run, execution_path, motion_path, Path(plan_path) if plan_path else None,
                             ROOT / 'verification/motion-quality-thresholds-v2.json')
    else:
        motion = json.loads(Path(motion_path).read_text(encoding='utf-8'))
    check('unchanged_motion_and_physical_geometry_auditor_passes', motion.get('motion_quality_validated') is True,
          {'report': str(Path(motion_path).resolve()), 'failed_checks': motion.get('failed_checks')})
    check('motion_report_bound_to_this_exact_capture', all(motion.get('input_sha256', {}).get(k) == hashes[k]
        for k in ['manifest.json', 'frames.jsonl', 'map.xodr', 'collisions.json'])
        and motion.get('execution_sha256') == file_sha(execution_path)
        and motion.get('auditor_sha256') == file_sha(ROOT / 'verification/audit_motion_quality.py')
        and motion.get('thresholds_source_sha256') == file_sha(ROOT / 'verification/motion-quality-thresholds-v2.json'))
    native_map = carla.Map(plan['map'], (run / 'map.xodr').read_text(encoding='utf-8'))
    lf_sha = hashlib.sha256((run / 'map.xodr').read_bytes().replace(b'\r\n', b'\n')).hexdigest()
    expected_map = plan.get('source_xodr_sha256')
    check('exact_native_map_source', expected_map in (hashes['map.xodr'], lf_sha),
          {'plan_sha256': expected_map, 'raw_sha256': hashes['map.xodr'], 'only_crlf_to_lf_sha256': lf_sha})
    refs = {'hero': native_reference_checked(native_map, plan['stages'])}
    for spec in plan['traffic']:
        refs[spec['name']] = native_reference_checked(native_map, spec['stages'])
    check('all_explicit_native_paths_and_topology_valid', True,
          {'actors': list(refs), 'reference_points': {role: len(value) for role, value in refs.items()}})
    traces = {role: [] for role in refs}
    actor_ids, missing, diagnostics = {}, [], {}
    gate = next(s['yield_gate']['polygon_xy'] for s in plan['stages'] if 'yield_gate' in s)
    minimum_ego, minimum_other = None, None
    pair_count, exact_count, overlap_count = 0, 0, 0
    for row in rows:
        boxes = {v['id']: obb(v) for v in row['vehicles']}
        present = [v.get('role_name') for v in row['vehicles']]
        for role in refs:
            if present.count(role) != 1:
                missing.append({'frame': row['frame'], 'role': role, 'count': present.count(role)})
        for v in row['vehicles']:
            role = v.get('role_name')
            if role not in traces:
                continue
            actor_ids.setdefault(role, set()).add(v['id'])
            p = v['transform']['location_m']
            wp = native_map.get_waypoint(carla.Location(*p))
            traces[role].append({'frame': row['frame'], 'time_s': row['time_s'], 'timestamp': row['timestamp'],
                'position_m': p, 'yaw_deg': v['transform']['rotation_deg'][1], 'speed_mps': float(np.linalg.norm(v['velocity_mps'])),
                'native_road': wp.road_id, 'native_lane': wp.lane_id, 'junction': wp.is_junction,
                'body_in_gate': overlap(convex_hull(boxes[v['id']]['vertices'][:, :2]), gate),
                'body_xy': convex_hull(boxes[v['id']]['vertices'][:, :2])})
            diagnostics.setdefault(role, []).append(row.get('traffic_control_diagnostics', {}).get('npc', {}).get(str(v['id']), {}))
        for a, b in itertools.combinations(row['vehicles'], 2):
            pair_count += 1
            aa, bb = boxes[a['id']], boxes[b['id']]
            is_ego = a['ego'] or b['ego']
            current = minimum_ego if is_ego else minimum_other
            lower = float(np.linalg.norm(aa['center'] - bb['center']) - np.linalg.norm(aa['extent']) - np.linalg.norm(bb['extent']))
            if current and lower >= current['signed_clearance_m']:
                continue
            value = obb_signed_clearance(aa, bb)
            exact_count += 1
            overlap_count += value < -1e-6
            if current is None or value < current['signed_clearance_m']:
                q = {'signed_clearance_m': value, 'frame': row['frame'], 'time_s': row['time_s'], 'roles': [a.get('role_name'), b.get('role_name')]}
                if is_ego:
                    minimum_ego = q
                else:
                    minimum_other = q
    check('all_explicit_actors_present_with_stable_ids', not missing and all(len(ids) == 1 for ids in actor_ids.values()),
          {'missing': missing[:20], 'ids': {role: sorted(ids) for role, ids in actor_ids.items()}})
    ego_limit = max(1.5, float(plan['expected_evidence'].get('must_observe_minimum_body_clearance_m', 1.5)))
    check('all_actor_pairs_have_no_sampled_obb_overlap', overlap_count == 0,
          {'all_pairs_screened': pair_count, 'exact_comparisons': exact_count, 'overlap_samples_found': overlap_count})
    check('original_junction_ego_clearance_threshold', minimum_ego and minimum_ego['signed_clearance_m'] >= ego_limit,
          {'required_m': ego_limit, 'minimum': minimum_ego})
    check('original_all_background_clearance_threshold', minimum_other and minimum_other['signed_clearance_m'] >= .75,
          {'required_m': .75, 'minimum': minimum_other})
    hero = traces['hero']
    check('ego_actual_native_reference_tracking', **dict(passed=(fit := trace_tracking(hero, refs['hero']))['passed'], evidence=fit))
    conflict_roles = groups['opposing'] or groups['circulating']
    for role in conflict_roles:
        spec = next(s for s in plan['traffic'] if s['name'] == role)
        if groups['opposing']:
            core, prefix = recorded_straight_core(traces[role])
            good, proof = straight_motion_proof(core, reference(native_map, spec['stages'], {1140}))
            check(role + '_actual_straight_native_crossing', good, {**proof, **prefix})
        else:
            theta = np.unwrap(np.arctan2([q['position_m'][1] - .4 for q in traces[role]],
                                         [q['position_m'][0] + .5 for q in traces[role]]))
            sweep = abs(float(np.degrees(theta[-1] - theta[0])))
            fit = trace_tracking(traces[role], refs[role])
            check(role + '_actual_native_circulation', sweep >= 120. and fit['passed']
                and max(q['speed_mps'] for q in traces[role]) > 1., {'actual_sweep_deg': sweep, **fit})
    interaction = gate_interaction(hero, {role: traces[role] for role in conflict_roles})
    check('ego_yields_without_simultaneous_conflict_body_occupation', interaction['body_pass_observed']
        and interaction['no_simultaneous_body_occupancy'] and bool(interaction.get('opponent_roles_observed_before')), interaction)
    stops = stopped_episodes(hero)
    qualifying = [q for q in stops if q['duration_s'] >= .5 and q['end_time_s'] <= interaction.get('ego_first_body_gate_time_s', -1)]
    check('actual_stopped_yield_at_least_half_second', bool(qualifying), stops)
    events = json.loads((run / 'controller-events.json').read_text(encoding='utf-8'))
    release = [e for e in events if e.get('event') == 'yield_gate_released']
    pose_by_frame = {q['frame']: q for q in hero}
    release_proofs = []
    for e in release:
        # Controller event refers to the measured snapshot before the next tick.
        q = pose_by_frame.get(e['frame'])
        if q is not None:
            release_proofs.append({'frame': e['frame'], 'event_timestamp': e['timestamp'],
                'actual_timestamp': q['timestamp'], 'actual_speed_mps': q['speed_mps'],
                'earlier_actual_stop_confirmed': any(s['duration_s'] >= .5 and s['first_frame'] <= q['frame']
                    and q['frame'] <= s['last_frame'] + 1 for s in stops)})
    check('yield_event_matches_actual_stop_snapshot', bool(release_proofs) and all(
        abs(q['event_timestamp'] - q['actual_timestamp']) <= .002 and q['actual_speed_mps'] <= .11
        and q['earlier_actual_stop_confirmed'] for q in release_proofs), release_proofs)
    if groups['opposing']:
        exiting = [q for q in hero if q['native_road'] == 47]
        turn = [q for q in hero if q['native_road'] == 1182 or (q['junction'] and -60 < q['position_m'][0] < -35 and -100 < q['position_m'][1] < -75)]
        fit = trace_tracking(turn, reference(native_map, plan['stages'], {1182}))
        heading = ((exiting[0]['yaw_deg'] - hero[0]['yaw_deg'] + 180) % 360 - 180) if exiting else None
        check('actual_left_turn_arc_and_departure_lane', len(turn) >= 5 and fit['passed'] and exiting
              and all(q['native_lane'] == -1 for q in exiting) and -110 <= heading <= -70, {'heading_change_deg': heading, **fit})
        signal_path = run / 'signal-policy.json'
        signal = json.loads(signal_path.read_text(encoding='utf-8')) if signal_path.exists() else {}
        check('opposing_green_cross_red_signals_and_source_hash', signal_path.exists()
            and manifest.get('signal_policy_sha256') == file_sha(signal_path)
            and signal_observations_proof(signal, rows, plan['signal_policy'])
            and signal.get('restoration', {}).get('requests_completed') is True)
    else:
        exiting = [q for q in hero if q['native_road'] == 8]
        order = [r for r, _ in itertools.groupby(q['native_road'] for q in hero)]
        required = [38, 12, 13, 14, 9, 10, 8]
        i = 0
        for road in order:
            if i < len(required) and road == required[i]:
                i += 1
        check('actual_native_ring_and_exit_road_order', i == len(required), {'observed': order, 'required': required})
    exit_distance = sum(math.dist(a['position_m'], b['position_m']) for a, b in zip(exiting, exiting[1:]))
    check('actual_exit_motion_at_least_15m', exit_distance >= 15., {'distance_m': exit_distance})
    endpoint = native_map.get_waypoint_xodr(*plan['stages'][-1]['xodr_path'][-1])
    finish_error = math.dist(hero[-1]['position_m'][:2], [endpoint.transform.location.x, endpoint.transform.location.y])
    finish_heading = angle_difference(hero[-1]['yaw_deg'], endpoint.transform.rotation.yaw)
    check('physical_terminal_pose_and_moving_plan_completion', finish_error <= 2. and finish_heading <= 8.
        and hero[-1]['speed_mps'] > .5 and manifest.get('controller_plan_completed') is True
        and manifest.get('recording_end_reason') == 'all native controller stages physically completed',
        {'endpoint_error_m': finish_error, 'heading_error_deg': finish_heading, 'speed_mps': hero[-1]['speed_mps']})
    follower_observations = {}
    for role in groups['followers']:
        trace = traces[role]
        fit = trace_tracking(trace, refs[role])
        rear = []
        for h, q in zip(hero, trace):
            dx, dy = np.asarray(q['position_m'][:2]) - np.asarray(h['position_m'][:2])
            angle = math.radians(h['yaw_deg'])
            along = dx * math.cos(angle) + dy * math.sin(angle)
            lateral = -dx * math.sin(angle) + dy * math.cos(angle)
            rear.append(-80 <= along <= -5 and abs(lateral) <= 3.5 and angle_difference(q['yaw_deg'], h['yaw_deg']) <= 45)
        queue_stops = [s for s in stopped_episodes(trace) if s['duration_s'] >= .5]
        restarts = []
        for stop in queue_stops:
            after = [q for q in trace if q['frame'] > stop['last_frame'] and q['speed_mps'] > .5]
            advance = max((math.dist(stop['last_position_m'], q['position_m']) for q in after), default=0)
            restarts.append({'stop_last_frame': stop['last_frame'], 'actual_subsequent_distance_m': advance,
                             'resumed_at_least_2m': advance >= 2.})
        observation = {'tracking': fit, 'rear_heading_corridor_fraction': sum(rear) / len(rear),
            'rear_heading_corridor_seconds': sum(rear) * .05, 'actual_stops': queue_stops,
            'actual_restarts': restarts, 'queue_resume_observed': any(q['resumed_at_least_2m'] for q in restarts),
            'measurement_limit': 'Body pose geometry, not pixels; curved-route rear coverage is reported separately from route tracking.'}
        follower_observations[role] = observation
        check(role + '_actual_following_queue_and_resume', fit['passed'] and sum(rear) >= 40
            and bool(queue_stops) and observation['queue_resume_observed'], observation)
    for role in groups['second_entry']:
        trace = traces[role]
        spec = next(s for s in plan['traffic'] if s['name'] == role)
        own_gate = next(s['yield_gate'] for s in spec['stages'] if 'yield_gate' in s)
        own = [dict(q, body_in_gate=overlap(q['body_xy'], own_gate['polygon_xy'])) for q in trace]
        others = {other: [dict(q, body_in_gate=overlap(q['body_xy'], own_gate['polygon_xy'])) for q in traces[other]]
                  for other in own_gate['actor_roles']}
        # Limit to its first entry, before a repeated circulating pass reaches
        # the same entry gate on the next lap.
        episode = runs(own, lambda q: q['body_in_gate'])
        finish_frame = episode[0][-1]['frame'] if episode else -1
        first_entry = [q for q in own if q['frame'] <= finish_frame]
        own_interaction = gate_interaction(first_entry, {r: [q for q in t if q['frame'] <= finish_frame] for r, t in others.items()})
        order = [r for r, _ in itertools.groupby(q['native_road'] for q in trace)]
        crossed = any(q['native_road'] == 30 for q in trace) and any(q['native_road'] == 1772 for q in trace)
        entered = next((q for q in trace if q['native_road'] == 10), None)
        fit = trace_tracking(trace, refs[role])
        check(role + '_actual_second_native_entry', crossed and entered is not None and fit['passed']
              and own_interaction['body_pass_observed'] and own_interaction['no_simultaneous_body_occupancy'],
              {'native_road_order': order, 'first_road10_time_s': entered['time_s'] if entered else None,
               'gate_interaction': own_interaction, 'tracking': fit})
    # This fact is deliberately not an unconditional PASS: a correct crossing
    # after all six cars clear does not demonstrate entering between streams.
    demonstration_facts = {'ego_gap_between_streams_observed': interaction['gap_between_streams_observed'],
        'ego_observed_yield_pattern': interaction.get('observed_pattern'),
        'opponent_body_passes': {role: first_body_pass(traces[role]) for role in conflict_roles},
        'follower_resume_observed': {role: q['queue_resume_observed'] for role, q in follower_observations.items()}}
    if plan['expected_evidence'].get('must_observe_gap_between_streams') is True:
        check('explicit_between_streams_demonstration_requirement', interaction['gap_between_streams_observed'])
    report = {'schema': 'native_v3_junction_role_aware_audit', 'scenario_actions_validated': all(checks.values()),
        'run': str(run), 'scenario': plan['scenario'], 'checks': checks,
        'failed_checks': [k for k, v in checks.items() if not v], 'evidence': details,
        'demonstration_facts': demonstration_facts, 'input_sha256': hashes,
        'execution_sha256': file_sha(execution_path), 'motion_report_sha256': file_sha(motion_path),
        'auditor_sha256': file_sha(Path(__file__)),
        'source_capture_modified': False, 'server_connected_or_ticked': False,
        'limitations': ['Sampled actual 3D boxes do not prove continuous-time collision avoidance.',
            'Road projection may report overlapping junction connector IDs; native reference residual and yaw are independently checked.',
            'Visibility, segmentation and LiDAR processing need their separate native sensor and visual audits.',
            'Source hashes bind this assessment to files; they do not independently authenticate arbitrary forged source data.']}
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({'report': str(out), 'scenario_actions_validated': report['scenario_actions_validated'],
                      'failed_checks': report['failed_checks'], 'demonstration_facts': demonstration_facts}))
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--execution', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--plan', type=Path)
    p.add_argument('--motion-report', type=Path)
    args = p.parse_args()
    result = audit(args.run, args.execution, args.out, args.plan, args.motion_report)
    return 0 if result['scenario_actions_validated'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
