"""Read-only audit of the user-visible native CARLA action, distinct from a full plan.

This deliberately does not rewrite captures or the stricter yield/whole-plan audit.
Run with the CARLA client Python; carla.Map below parses saved OpenDRIVE offline.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ego(row):
    return next(v for v in row['vehicles'] if v['ego'])


def loc(actor):
    return actor['transform']['location_m']


def wrapped_yaw_change(first, last):
    return (last['transform']['rotation_deg'][1]-first['transform']['rotation_deg'][1]+180)%360-180


def inside(point, polygon):
    result = False
    for i, a in enumerate(polygon):
        b = polygon[i-1]
        if ((a[1] > point[1]) != (b[1] > point[1]) and
                point[0] < (b[0]-a[0])*(point[1]-a[1])/(b[1]-a[1])+a[0]):
            result = not result
    return result


def pose_evidence(row, actor):
    return {'sample': row['sample'], 'frame': row['frame'], 'display_time_s': row['time_s'],
            'native_timestamp_s': row['timestamp'], 'location_m': loc(actor),
            'yaw_deg': actor['transform']['rotation_deg'][1], 'speed_mps': actor['speed_mps'],
            'nearest_native_waypoint': actor['waypoint']}


def transitions(rows, role):
    result, previous = [], None
    for row in rows:
        actor = next(v for v in row['vehicles'] if v['role_name'] == role)
        wp = actor['waypoint']
        current = (wp['road_id'], wp['lane_id'])
        if current != previous:
            result.append({'sample': row['sample'], 'frame': row['frame'],
                           'time_s': row['time_s'], 'road_lane': list(current), 's_m': wp['s_m']})
            previous = current
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--whole-audit', type=Path, required=True)
    parser.add_argument('--cut-out', type=Path, required=True)
    parser.add_argument('--physics-audit', type=Path,
                        help='Independent, already generated native 3D OBB report; read only.')
    args = parser.parse_args()
    run = args.run.resolve()
    manifest_path, frames_path = run/'manifest.json', run/'frames.jsonl'
    manifest = json.loads(manifest_path.read_text())
    rows = [json.loads(s) for s in frames_path.read_text().splitlines()]
    actors = [ego(r) for r in rows]
    events = json.loads((run/'controller-events.json').read_text())
    collisions = json.loads((run/'collisions.json').read_text())
    execution_path = run.parent.parent/'executions'/(run.name+'.json')
    execution = json.loads(execution_path.read_text())
    whole = json.loads(args.whole_audit.read_text())
    if Path(whole['run']).resolve() != run:
        raise ValueError('Whole-plan audit references a different capture.')
    checks = {
        'actual_native_process_exit_zero': execution.get('returncode') == 0,
        'execution_manifest_hash_matches': execution.get('manifest_sha256') == sha(manifest_path),
        'execution_frames_hash_matches': execution.get('frames_jsonl_sha256') == sha(frames_path),
        'capture_manifest_complete': manifest.get('status') == 'complete',
        'native_cleanup_complete': manifest.get('native_cleanup_complete') is True,
        'native_collision_zero': len(collisions) == 0,
        'actual_builtin_tesla_model3': all(e['blueprint'] == 'vehicle.tesla.model3' for e in actors),
        'native_frames_contiguous': all(b['frame']-a['frame'] == 1 for a, b in zip(rows, rows[1:])),
        'native_sampling_matches_source_fps': all(abs(b['timestamp']-a['timestamp']-1/manifest['fps']) < 1e-5 for a, b in zip(rows, rows[1:])),
    }
    specification = manifest['route_specification']
    stages = specification['stages']
    completed = [e for e in events if e['event'] == 'stage_completed']
    completed_indices = sorted({e['stage_index'] for e in completed})
    final_stage = stages[-1]
    if final_stage.get('path'):
        endpoint = final_stage['path'][-1]
    else:
        import carla
        road_map = carla.Map(manifest['map'], (run/'map.xodr').read_text())
        endpoint_wp = road_map.get_waypoint_xodr(*final_stage['xodr_path'][-1])
        p = endpoint_wp.transform.location
        endpoint = [p.x, p.y, p.z]
    nearest_final_distance = min(math.dist(loc(e)[:2], endpoint[:2]) for e in actors)
    all_stages_completed = completed_indices == list(range(len(stages)))
    endpoint_reached = nearest_final_distance < 2.5
    report = {
        'audit_type': 'independent_user_visible_core_action',
        'source': 'Actual native CARLA frame poses, controls, events and independently recorded process exit; no server connection, replay or new simulation.',
        'run': str(run), 'scenario': manifest['scenario'], 'checks': checks,
        'process_evidence': {'execution': str(execution_path), 'execution_sha256': sha(execution_path),
                             'returncode': execution['returncode'], 'collision_event_count': len(collisions),
                             'manifest_sha256': sha(manifest_path), 'frames_jsonl_sha256': sha(frames_path),
                             'controller_events_sha256': sha(run/'controller-events.json')},
        'samples': len(rows), 'source_fps': manifest['fps'], 'source_duration_s': len(rows)/manifest['fps'],
        'full_internal_plan_completed': all_stages_completed and endpoint_reached,
        'full_internal_plan_evidence': {'completed_stage_indices': completed_indices,
                                        'planned_stage_count': len(stages),
                                        'final_reference_endpoint_m': endpoint,
                                        'minimum_actual_endpoint_distance_m': nearest_final_distance,
                                        'endpoint_within_2_5m': endpoint_reached},
        'preserved_strict_audit': {'path': str(args.whole_audit.resolve()), 'sha256': sha(args.whole_audit),
                                   'action_audit_passed': whole['action_audit_passed']},
        'nearest_native_waypoint_trace': transitions(rows, actors[0]['role_name']),
        'native_waypoint_note': 'The native nearest-driving-lane query can select nearby overlapping connector aliases inside junctions. All recorded query IDs are retained; geometric action is verified with actual pose, heading and exit motion.',
        'independent_full_3d_obb_check': 'Separate auditor report required; this report does not substitute zero collision callbacks for a body-overlap check.',
        'trained_model_claimed': False, 'processed_sensor_results_validated': False,
        'encoded_video_validated': False,
    }
    scenario = manifest['scenario']
    expected = specification.get('expected_evidence', {})
    gate_stage = next(s for s in stages if s.get('yield_gate'))
    gate = gate_stage['yield_gate']
    polygon = gate['polygon_xy']
    releases = [e for e in events if e['event'] == 'yield_gate_released']
    stopped_conflict = [(r, e) for r, e in zip(rows, actors)
                        if r.get('rule_controller', {}).get('waiting') == 'native_conflict_traffic'
                        and e['speed_mps'] < .5]
    first_entry = next(((r, e) for r, e in zip(rows, actors) if inside(loc(e), polygon)), None)
    roles = gate['actor_roles']
    opponent_facts = {}
    for role in roles:
        actor_series = [(r, next(v for v in r['vehicles'] if v['role_name'] == role)) for r in rows]
        occupancies = [(r, a) for r, a in actor_series if inside(loc(a), polygon)]
        before = [(r, a) for r, a in occupancies if first_entry and r['frame'] < first_entry[0]['frame']]
        delta = wrapped_yaw_change(actor_series[0][1], actor_series[-1][1])
        opponent_facts[role] = {
            'actor_id': actor_series[0][1]['id'], 'blueprint': actor_series[0][1]['blueprint'],
            'first_pose': pose_evidence(*actor_series[0]), 'last_pose': pose_evidence(*actor_series[-1]),
            'heading_change_deg': delta,
            'conflict_polygon_first_occupancy': pose_evidence(*occupancies[0]) if occupancies else None,
            'conflict_polygon_last_occupancy_before_ego': pose_evidence(*before[-1]) if before else None,
            'occupied_at_ego_entry': bool(first_entry and any(r['frame'] == first_entry[0]['frame'] for r, a in occupancies)),
            'road_lane_trace': transitions(rows, role),
        }
    report['gate_evidence'] = {
        'polygon_xy_m': polygon, 'actual_stopped_conflict_samples': len(stopped_conflict),
        'first_stopped_conflict_pose': pose_evidence(*stopped_conflict[0]) if stopped_conflict else None,
        'last_stopped_conflict_pose': pose_evidence(*stopped_conflict[-1]) if stopped_conflict else None,
        'actual_ego_first_polygon_occupancy': pose_evidence(*first_entry) if first_entry else None,
        'release_events': [dict(e, display_time_s=e['timestamp']-rows[0]['timestamp']) for e in releases],
        'opponents': opponent_facts,
    }
    cut_end = len(rows)
    if 'left_turn' in scenario:
        wanted = expected['ego_road_order']
        roads = [x['road_lane'][0] for x in report['nearest_native_waypoint_trace']]
        cursor = 0
        for road in roads:
            if cursor < len(wanted) and road == wanted[cursor]:
                cursor += 1
        checks['actual_approach_curve_exit_order_allowing_query_aliases'] = cursor == len(wanted)
        delta = wrapped_yaw_change(actors[0], actors[-1])
        checks['actual_left_heading_rotation'] = delta < -55
        checks['actual_stopped_for_native_conflict'] = len(stopped_conflict) > 0
        checks['actual_unprotected_wait'] = bool(stopped_conflict and all(not r['ego_is_at_traffic_light'] for r, e in stopped_conflict))
        checks['both_actual_opponents_passed_conflict_polygon_before_ego'] = bool(first_entry and all(
            x['conflict_polygon_last_occupancy_before_ego'] and not x['occupied_at_ego_entry'] for x in opponent_facts.values()))
        checks['release_recorded_after_both_opponents_actual_passage'] = bool(releases and all(
            x['conflict_polygon_last_occupancy_before_ego'] and
            x['conflict_polygon_last_occupancy_before_ego']['native_timestamp_s'] < releases[0]['timestamp']
            for x in opponent_facts.values()))
        for x in opponent_facts.values():
            x['actual_maneuver'] = 'right_turn_to_road_27' if x['heading_change_deg'] > 55 and x['last_pose']['nearest_native_waypoint']['road_id'] == 27 else 'other_or_undetermined'
            x['planned_straight_motion_observed'] = False
        exit_samples = [(r, e) for r, e in zip(rows, actors) if e['waypoint']['road_id'] == wanted[-1] and e['speed_mps'] > 1]
        continuous_moving_exit = bool(exit_samples and all(e['waypoint']['road_id'] == wanted[-1] and e['speed_mps'] > 1
                                     for e in actors[exit_samples[0][0]['sample']:]))
        exit_moving_s = exit_samples[-1][0]['time_s']-exit_samples[0][0]['time_s'] if exit_samples else 0
        max_exit_s = max((e['waypoint']['s_m'] for r, e in exit_samples), default=0)
        extent = max((e['bounding_box']['extent_m'][0] for r, e in exit_samples), default=float('inf'))
        checks['actual_exit_motion_continuous_at_least_2s'] = continuous_moving_exit and exit_moving_s >= 2
        checks['actual_body_beyond_exit_with_3m_longitudinal_margin'] = max_exit_s > extent+3
        report.update({
            'accepted_core_scope': 'Approach an unsignalized junction, actually stop and let the two oncoming-origin conflict vehicles pass, perform a native physical left turn, then continue moving in exit road27. Their actual maneuver is a right turn, not a straight traversal.',
            'heading_change_deg': delta,
            'exit_motion': {'first': pose_evidence(*exit_samples[0]) if exit_samples else None,
                            'last': pose_evidence(*exit_samples[-1]) if exit_samples else None,
                            'continuous_moving_exit_span_s': exit_moving_s,
                            'maximum_actual_exit_s_m': max_exit_s,
                            'bbox_longitudinal_half_extent_m': extent},
            'remaining_internal_plan': 'The extra road27/s23 reference endpoint is not reached; the stage-complete event for the entire follow-path stage is absent. The core-action pass does not imply full-plan completion.',
            'core_left_turn_completed': all(checks.values()),
        })
        core_complete_time = exit_samples[0][0]['timestamp']+2 if exit_samples else rows[-1]['timestamp']
    elif 'roundabout' in scenario:
        center = expected.get('ring_center_xy_m', [-.5, .4])
        west = next(((r, e) for r, e in zip(rows, actors) if loc(e)[0] < -16 and abs(loc(e)[1]-center[1]) < 20), None)
        north = next(((r, e) for r, e in zip(rows, actors) if loc(e)[1] > 15 and abs(loc(e)[0]-center[0]) < 20), None)
        exit_pair = expected['exit_road_lane']
        east_exit = next(((r, e) for r, e in zip(rows, actors)
                          if [e['waypoint']['road_id'], e['waypoint']['lane_id']] == exit_pair), None)
        checks['actual_native_west_then_north_then_east_exit_order'] = bool(west and north and east_exit and
            west[0]['frame'] < north[0]['frame'] < east_exit[0]['frame'])
        checks['all_internal_stages_completed'] = all_stages_completed
        checks['actual_final_reference_endpoint_reached'] = endpoint_reached
        checks['actual_east_exit_moving'] = bool(east_exit and east_exit[1]['speed_mps'] > 1)
        report.update({
            'accepted_core_scope': 'Native physical roundabout entry, circulation along west and north arcs, and east exit; this does not claim a triggered wait for circulating priority traffic.',
            'route_poses': {'west': pose_evidence(*west) if west else None,
                            'north': pose_evidence(*north) if north else None,
                            'east_exit': pose_evidence(*east_exit) if east_exit else None},
            'actual_conflict_yield_triggered': len(stopped_conflict) > 0,
            'forced_yield_wait_demonstrated': len(stopped_conflict) > 0,
            'actual_priority_vehicle_first_pass_claim_supported': whole['checks']['actual_conflict_vehicle_passed_before_ego'],
            'yield_outcome': 'No native conflict wait was triggered. Recorded release event reports zero predicted conflicts. This run proves circulation and exit, not waiting for a priority vehicle.',
            'core_roundabout_route_completed': all(checks.values()),
            'requested_route_action_completed': all(checks.values()),
        })
        core_complete_time = max(e['timestamp'] for e in completed)
        completion_index = next(i for i, r in enumerate(rows) if r['timestamp'] >= core_complete_time)
        for i in range(completion_index+2, len(rows)-2):
            if all(actors[k]['speed_mps'] < .5 for k in range(i, i+3)):
                cut_end = i
                break
    else:
        raise ValueError('Only left-turn and roundabout core audits are supported.')
    report['core_action_passed'] = all(checks.values())
    if args.physics_audit:
        physics = json.loads(args.physics_audit.read_text())
        if Path(physics['run']).resolve() != run:
            raise ValueError('Independent body audit belongs to a different capture.')
        physics_checks = physics['checks']
        checks['independent_recorded_3d_obb_no_overlap'] = physics_checks['no_sampled_vehicle_obb_overlap'] is True
        checks['independent_body_audit_matches_current_raw_and_execution'] = (
            physics_checks['execution_matches_current_raw_manifest_and_frames'] is True and
            physics['input_sha256']['manifest.json'] == sha(manifest_path) and
            physics['input_sha256']['frames.jsonl'] == sha(frames_path) and
            physics['input_sha256']['collisions.json'] == sha(run/'collisions.json') and
            physics['execution_sha256'] == sha(execution_path) and physics['frames'] == len(rows))
        report['independent_full_3d_obb_check'] = {
            'report': str(args.physics_audit.resolve()), 'report_sha256': sha(args.physics_audit),
            'recorded_frames_tested': physics['frames'],
            'sampled_body_separation_validated': physics['sampled_body_separation_validated'],
            'all_vehicle_minimum': physics['all_vehicle_minimum'],
            'ego_minimum': physics['ego_minimum'],
            'meaning': 'Native full 3D body-box comparisons at all recorded 20Hz frames. This does not establish continuous-time or real-world driving safety.',
        }
        report['core_action_passed'] = all(checks.values())
        if 'left_turn' in scenario:
            report['core_left_turn_completed'] = report['core_action_passed']
        else:
            report['core_roundabout_route_completed'] = report['core_action_passed']
            report['requested_route_action_completed'] = report['core_action_passed']
            report['observed_near_pass_note'] = 'The minimum recorded native body-box clearance is approximately 0.127m near roundabout entry. Do not describe this run as demonstrating ample safety clearance.'
    last_row, last_actor = rows[cut_end-1], actors[cut_end-1]
    cut = {'source_run': str(run), 'core_action_audit': str(args.out.resolve()),
           'source_fps': manifest['fps'], 'samples_start_inclusive': 0, 'samples_end_exclusive': cut_end,
           'duration_s': cut_end/manifest['fps'], 'last_sample': cut_end-1,
           'last_carla_frame': last_row['frame'], 'last_display_time_s': last_row['time_s'],
           'last_speed_mps': last_actor['speed_mps'],
           'last_native_road_lane': [last_actor['waypoint']['road_id'], last_actor['waypoint']['lane_id']],
           'user_visible_core_completed': report['core_action_passed'],
           'full_internal_plan_completed': report['full_internal_plan_completed'],
           'core_completion_display_time_s': core_complete_time-rows[0]['timestamp'],
           'retained_seconds_after_core_completion': last_row['timestamp']-core_complete_time+1/manifest['fps'],
           'removed_tail_frames': len(rows)-cut_end,
           'encoded_video_verified': False, 'no_fake_frames': True, 'no_motion_speed_change': True,
           'selection': 'Original contiguous samples only; preserve user-visible core action and remove consecutive parking tail if present.',
           'scope_note': report['accepted_core_scope']}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    # A cut is only proposed when source-based core checks pass.
    if report['core_action_passed']:
        args.cut_out.parent.mkdir(parents=True, exist_ok=True)
        args.cut_out.write_text(json.dumps(cut, indent=2), encoding='utf-8')
    print(json.dumps({'report': str(args.out.resolve()), 'core_passed': report['core_action_passed'],
                      'full_internal_plan_completed': report['full_internal_plan_completed'],
                      'checks': checks, 'cut': cut if report['core_action_passed'] else None}, indent=2))
    return 0 if report['core_action_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
