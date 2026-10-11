"""Independent read-only evidence audit of actual official live co-simulation.

Recomputes CARLA/SUMO front-bumper conversion from exported actual states.
Explicitly tests the official next-SUMO-step latch; does not claim zero delay.
No Client, server connection, tick, capture changes or image generation.
"""
from pathlib import Path
import argparse
import collections
import itertools
import json
import math
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_native_actions import file_sha, obb, obb_signed_clearance


def angle_error(a, b):
    return abs((a - b + 180) % 360 - 180)


def front_from_carla(actor, offset):
    x, y, _ = actor['carla_world_center_m']
    yaw = actor['carla_rotation_pitch_yaw_roll_deg'][1]
    length_half = actor['carla_bbox_extent_m'][0]
    return [x + length_half * math.cos(math.radians(yaw)) + offset[0],
            -y - length_half * math.sin(math.radians(yaw)) + offset[1]], (yaw + 90) % 360


def audit(run, execution, expected_frames, out):
    run = run.resolve()
    manifest = json.loads((run / 'manifest.json').read_text())
    frames = [json.loads(line) for line in (run / 'frames.jsonl').read_text().splitlines() if line.strip()]
    syncs = [json.loads(line) for line in (run / 'sumo-sync.jsonl').read_text().splitlines() if line.strip()]
    process = json.loads(execution.read_text())
    meta = manifest['live_co_simulation']
    dt = manifest['fixed_delta_seconds']
    hashes = {name: file_sha(run / name) for name in ['manifest.json', 'frames.jsonl', 'sumo-sync.jsonl', 'map.xodr', 'collisions.json']}
    checks, evidence = {}, {}
    def check(name, good, value=None):
        checks[name] = bool(good)
        if value is not None:
            evidence[name] = value
    check('actual_python_client_exit_zero', process.get('returncode') == 0 and process.get('native_process_returncode') == 0)
    check('execution_matches_actual_raw_capture', Path(process['capture_run']).resolve() == run and process['manifest_sha256'] == hashes['manifest.json'] and process['frames_jsonl_sha256'] == hashes['frames.jsonl'])
    check('native_capture_and_cleanup_complete', manifest.get('status') == 'complete' and manifest.get('capture_samples_complete') is True and manifest.get('native_cleanup_complete') is True)
    check('real_live_mode_and_no_replay_fallback', meta['mode'] == 'live_official_bidirectional_actor_synchronization' and meta['postcapture_replay'] is False and manifest.get('synthetic_sensor_fallback') is False)
    check('exact_requested_complete_frame_count', len(frames) == len(syncs) == expected_frames == manifest['frames_captured'])
    official = Path(meta['official_sync_source'])
    bridge = official.parent / 'sumo_integration/bridge_helper.py'
    check('official_sync_and_bridge_hashes_match', file_sha(official) == meta['official_sync_sha256'] and file_sha(bridge) == meta['official_bridge_sha256'])
    network = Path(meta['network'])
    cfg = Path(meta['sumo_cfg'])
    netroot = ET.parse(network).getroot()
    offset = [float(v) for v in netroot.find('location').get('netOffset').split(',')]
    check('actual_sumo_config_and_network_hashes_match', file_sha(network) == meta['network_sha256'] and file_sha(cfg) == meta['sumo_cfg_sha256'])
    check('network_offset_matches_actual_net_xml', offset == meta['network_offset_m'], offset)
    provenance_path = network.parent.parent / 'investigation.json'
    provenance = json.loads(provenance_path.read_text())
    town = manifest['map'].split('/')[-1]
    pmap = next(m for m in provenance['maps'] if m['town'] == town)
    pattempt = next(a for a in pmap['attempts'] if a['kind'] == 'official-carla')
    check('actual_native_map_is_official_converted_network_source',
          pmap['source_xodr_sha256'] == hashes['map.xodr'] == file_sha(Path(pmap['source_xodr'])) and
          pattempt['actual_returncode'] == 0 and pattempt['network']['sha256'] == hashes.get('network', file_sha(network)),
          {'source_xodr': pmap['source_xodr'], 'native_map_sha256': hashes['map.xodr'], 'network_sha256': file_sha(network), 'converter_actual_returncode': pattempt['actual_returncode']})
    problems = []
    times = []
    source_match_errors = []
    actor_observations = collections.defaultdict(list)
    coordinate_errors = []
    carla2sumo_errors, sumo2carla_errors = [], []
    delay_counts = collections.Counter()
    maps_counts = []
    signed_minimum = None
    overlaps = []
    index_rows = [{a['sumo_id']: a for a in sync['actors']} for sync in syncs]
    for i, (frame, sync) in enumerate(zip(frames, syncs)):
        if frame['sample'] != i or sync['source_sample'] != i or frame['frame'] != sync['source_frame'] or abs(frame['timestamp'] - sync['source_timestamp_s']) > 1e-6:
            source_match_errors.append(i)
        if i and (frame['frame'] != frames[i-1]['frame'] + 1 or abs(frame['timestamp'] - frames[i-1]['timestamp'] - dt) > 2e-6 or abs(sync['sumo_time_s'] - syncs[i-1]['sumo_time_s'] - dt) > 2e-6):
            times.append(i)
        observed_native = {a['id']: a for a in frame['vehicles']}
        s2c = sync['sumo_to_carla_ids']
        c2s = sync['carla_to_sumo_ids']
        maps_counts.append([len(s2c), len(c2s)])
        if not s2c or not c2s or set(s2c).intersection(c2s.values()) or set(s2c.values()).intersection(int(k) for k in c2s):
            problems.append({'sample':i, 'problem':'missing, overlapping or cyclic authority maps'})
        expected = {(sid, int(cid), 'sumo_to_carla') for sid, cid in s2c.items()} | {(sid, int(cid), 'carla_to_sumo') for cid, sid in c2s.items()}
        actual = {(a['sumo_id'], a['carla_id'], a['direction']) for a in sync['actors']}
        if expected != actual:
            problems.append({'sample':i, 'problem':'mapping rows differ from actual authority maps'})
        for a in sync['actors']:
            native = observed_native.get(a['carla_id'])
            if native is None or math.dist(native['transform']['location_m'], a['carla_world_center_m']) > 1e-6 or angle_error(native['transform']['rotation_deg'][1], a['carla_rotation_pitch_yaw_roll_deg'][1]) > 1e-6:
                problems.append({'sample':i, 'problem':'mapped actor differs from recorded actual world snapshot', 'actor':a['carla_id']})
            target_xy, target_angle = front_from_carla(a, offset)
            current_xy_error = math.dist(target_xy, a['sumo_front_bumper_xy_m'])
            if a['direction'] == 'carla_to_sumo':
                # TraCI moveToXY is latched on the NEXT SUMO simulation step.
                # Test its actual readback at i+1; the final command has no
                # next recorded SUMO frame, so it is explicitly excluded.
                if i + 1 < len(syncs):
                    future = index_rows[i+1].get(a['sumo_id'])
                    xy_error = math.dist(target_xy, future['sumo_front_bumper_xy_m']) if future else float('inf')
                    yaw_error = angle_error(target_angle, future['sumo_navigation_angle_deg']) if future else float('inf')
                    carla2sumo_errors.append(xy_error)
                    delay_counts['carla_to_sumo_next_step'] += 1
                    if xy_error > .03 or yaw_error > .1:
                        coordinate_errors.append({'sample':i, 'direction':a['direction'], 'actor':a['sumo_id'], 'readback_source_sample':i+1, 'xy_error_m':xy_error, 'yaw_error_deg':yaw_error})
            elif a['direction'] == 'sumo_to_carla':
                # Native snapshots can show a queued official transform one
                # frame late. Preserve and disclose that actual SDK phase.
                candidates = [(current_xy_error, angle_error(target_angle, a['sumo_navigation_angle_deg']), 0)]
                if i:
                    previous = index_rows[i-1].get(a['sumo_id'])
                    if previous:
                        candidates.append((math.dist(target_xy, previous['sumo_front_bumper_xy_m']), angle_error(target_angle, previous['sumo_navigation_angle_deg']), 1))
                xy_error, yaw_error, delay = min(candidates, key=lambda value: value[0])
                sumo2carla_errors.append(xy_error)
                delay_counts['sumo_to_carla_same_step' if delay == 0 else 'sumo_to_carla_previous_step'] += 1
                if xy_error > .03 or yaw_error > .1:
                    coordinate_errors.append({'sample':i,'direction':a['direction'],'actor':a['sumo_id'],'readback_source_sample':i-delay,'xy_error_m':xy_error,'yaw_error_deg':yaw_error})
                actor_observations[a['sumo_id']].append({'sample':i, 'sumo_xy':a['sumo_front_bumper_xy_m'], 'carla_xyz':a['carla_world_center_m'], 'sumo_speed_mps':a['sumo_speed_mps']})
        boxes = {a['id']:obb(a) for a in frame['vehicles']}
        for a,b in itertools.combinations(frame['vehicles'],2):
            clearance = obb_signed_clearance(boxes[a['id']],boxes[b['id']])
            value = {'signed_clearance_m':clearance,'sample':i,'frame':frame['frame'],'time_s':frame['time_s'],'actor_ids':[a['id'],b['id']],'roles':[a.get('role_name'),b.get('role_name')]}
            if signed_minimum is None or clearance < signed_minimum['signed_clearance_m']:
                signed_minimum = value
            if clearance < -.005:
                overlaps.append(value)
    offsets = [sync['sumo_time_s'] - frame['timestamp'] for frame,sync in zip(frames,syncs)]
    check('raw_and_live_records_share_actual_samples_frames_timestamps', not source_match_errors, {'mismatch_samples':source_match_errors[:20]})
    check('both_clocks_advance_exactly_one_fixed_step_per_world_frame', not times and max(offsets)-min(offsets) < 2e-5,
          {'step_s':dt,'violations':times[:20],'sumo_minus_carla_clock_epoch_offset_s':offsets[0], 'epoch_offset_range_s':max(offsets)-min(offsets)})
    check('actual_two_way_maps_have_distinct_authority_and_world_actors', not problems and all(a>0 and b>0 for a,b in maps_counts), {'actual_mapping_counts_per_frame':sorted(set(tuple(c) for c in maps_counts)), 'violations':problems[:20]})
    check('official_front_bumper_coordinate_conversion_matches_phase_correct_actual_readbacks', not coordinate_errors and bool(carla2sumo_errors) and bool(sumo2carla_errors),
          {'xy_tolerance_m':.03,'yaw_tolerance_deg':.1,'carla_to_sumo_next_step_max_xy_error_m':max(carla2sumo_errors), 'sumo_to_carla_same_or_one_step_late_max_xy_error_m':max(sumo2carla_errors), 'actual_delay_counts':dict(delay_counts), 'violations':coordinate_errors[:20], 'terminal_pending_carla_to_sumo_commands_not_observable':len(syncs[-1]['carla_to_sumo_ids']), 'formula':'SUMOx=CARLAx+extentX*cos(yaw)+offsetX; SUMOy=-CARLAy-extentX*sin(yaw)+offsetY; SUMOangle=CARLAyaw+90.'})
    autonomous = {}
    for sid, records in actor_observations.items():
        autonomous[sid] = {'actual_samples':len(records),
            'actual_sumo_displacement_from_initial_m':max(math.dist(v['sumo_xy'], records[0]['sumo_xy']) for v in records),
            'actual_carla_proxy_displacement_from_initial_m':max(math.dist(v['carla_xyz'],records[0]['carla_xyz']) for v in records),
            'actual_sumo_max_speed_mps':max(v['sumo_speed_mps'] for v in records)}
    check('sumo_origin_actor_actually_moves_in_both_running_simulators', bool(autonomous) and all(v['actual_samples']==expected_frames and v['actual_sumo_displacement_from_initial_m']>5 and v['actual_carla_proxy_displacement_from_initial_m']>5 and v['actual_sumo_max_speed_mps']>1 for v in autonomous.values()), autonomous)
    sumo_collisions = [s['source_sample'] for s in syncs if s.get('sumo_colliding_vehicle_ids')]
    collision_records = json.loads((run/'collisions.json').read_text())
    check('actual_native_and_sumo_collision_records_zero', not collision_records and manifest['collision_count']==0 and not sumo_collisions, {'native_records':len(collision_records),'sumo_collision_samples':sumo_collisions})
    check('all_recorded_native_vehicles_have_separated_3d_obb', not overlaps and signed_minimum is not None and signed_minimum['signed_clearance_m']>=.05, {'minimum':signed_minimum, 'overlap_samples':len(overlaps), 'overlap_examples':overlaps[:10]})
    check('live_sumo_sync_log_hash_matches_capture_summary', hashes['sumo-sync.jsonl']==meta['sumo_sync_jsonl_sha256'])
    report = {'live_joint_simulation_verified':all(checks.values()), 'run':str(run), 'frames':len(frames), 'fps':manifest['fps'], 'duration_s':len(frames)*dt,
        'checks':checks, 'failed_checks':[name for name,good in checks.items() if not good], 'evidence':evidence,
        'input_sha256':hashes,'execution_file':str(execution.resolve()),'execution_sha256':file_sha(execution),
        'network_provenance_file':str(provenance_path),'network_provenance_sha256':file_sha(provenance_path),
        'delivery_videos':[], 'scope':'Actual live official actor synchronization and sampled physical separation. Video/sensor pixel audits remain separate.',
        'limitations':['CARLA-origin transforms become observable in SUMO at the next step, a 0.05s phase delay. Final pending commands have no next recorded readback and are disclosed rather than accepted as same-step.',
            'SUMO-origin official kinematic CARLA proxies may appear in a native snapshot zero or one step later. They use set_transform with physics disabled, as implemented by the official bridge.',
            'Actor state exchange is verified; this does not claim a single coupled physics solver or zero-latency control.',
            '3D OBB clearance is sampled at recorded native world frames, not a continuous-time collision guarantee.'],
        'raw_capture_modified':False, 'server_connected_or_ticked':False}
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({'report':str(out),'live_joint_simulation_verified':report['live_joint_simulation_verified'],'failed_checks':report['failed_checks'], 'phase_delay_counts':dict(delay_counts),'autonomous':autonomous,'minimum_body_clearance':signed_minimum},indent=2))
    return report['live_joint_simulation_verified']


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--execution',type=Path,required=True)
    p.add_argument('--expected-frames',type=int,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    raise SystemExit(0 if audit(a.run,a.execution,a.expected_frames,a.out) else 2)
