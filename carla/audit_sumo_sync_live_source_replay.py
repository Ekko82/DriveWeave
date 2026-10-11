"""Independent read-only verification of saved CARLA -> SUMO native GUI replay.

Recomputes source correspondence, official BridgeHelper inputs, an independent
scalar conversion, TraCI readback errors and every actual PNG/hash/dimension.
It neither connects to CARLA/SUMO nor edits the replay or source recording.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET
from PIL import Image


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runtime_config import RuntimeConfig, hidden_process_options
_RUNTIME = RuntimeConfig.resolve(require_sumo=True, require_bridge=True)
_RUNTIME.activate()
SUMO_HOME, CARLA_SUMO = _RUNTIME.sumo_home, _RUNTIME.carla_sumo_root
import carla
from sumo_integration.bridge_helper import BridgeHelper


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def angle_distance(a, b):
    return abs((a-b+180)%360-180)


def scalar_front(actor, offset):
    # Independently reproduce official BridgeHelper's actor-origin/extent
    # convention, including CARLA float32 Location/Rotation storage. This is
    # intentionally not a separately invented full rotated-box-center mapping.
    x, y, z = actor['transform']['location_m']
    yaw = actor['transform']['rotation_deg'][1]
    extent_x = actor['bounding_box']['extent_m'][0]
    a = math.radians(yaw)
    return [f32(x+math.cos(a)*extent_x+offset[0]),
            f32(offset[1]-y-math.sin(a)*extent_x)], f32(yaw+90)%360


def scalar_inverse(xy, nav_angle, extent_x, offset):
    a = math.radians(90-f32(nav_angle))
    return [f32(f32(xy[0])-math.cos(a)*extent_x-offset[0]),
            f32(offset[1]-f32(xy[1])+math.sin(a)*extent_x)]


def outer_source_binding(process, manifest_file, run, source_live):
    """Bind a generic v2 runner to its immutable real live source.

    Legacy runners add a source_was_live_capture boolean. V2 initially records
    the exact --run command and accepted replay manifest hash instead. Neither
    representation may substitute for the independently verified source live
    mode, SUMO sync hash, bridge, network and raw execution checks below.
    """
    command = process.get('command', [])
    try:
        command_run = Path(command[command.index('--run')+1]).resolve()
    except (ValueError, IndexError, TypeError):
        return False, 'missing_actual_source_command'
    bound = (process.get('returncode') == 0 and
             process.get('this_render_is_live') is False and
             process.get('mode') == 'post_capture_synchronized_replay' and
             process.get('manifest_sha256') == sha(manifest_file) and
             command_run == run.resolve() and
             source_live.get('mode') == 'live_official_bidirectional_actor_synchronization' and
             source_live.get('postcapture_replay') is False)
    explicit = process.get('source_was_live_capture') is True
    generic_v2 = (process.get('CARLA_connected_or_ticked') is False and
                  process.get('source_gates_automated_passed') is True and
                  process.get('source_gate_phase') in {'motion_action_preparation', 'all_source_gates_passed'})
    return bound and (explicit or generic_v2), ('legacy_explicit_source_flag' if explicit else
                        'v2_command_and_immutable_source_manifest_binding')


def audit(replay, out, process_file):
    replay = replay.resolve()
    manifest_file = replay/'manifest.json'
    manifest = json.loads(manifest_file.read_text())
    run = Path(manifest['source_run'])
    source_manifest_file, source_frames_file = run/'manifest.json', run/'frames.jsonl'
    source_manifest = json.loads(source_manifest_file.read_text())
    source_rows = [json.loads(line) for line in source_frames_file.read_text().splitlines()]
    execution_file = run.parent.parent/'executions'/(run.name+'.json')
    execution = json.loads(execution_file.read_text())
    process_file = process_file.resolve()
    processes = json.loads(process_file.read_text())
    process = next(p for p in processes if Path(p['manifest']).resolve() == manifest_file)
    sample_argument = int(process['command'][process['command'].index('--samples')+1])
    network = Path(manifest['network'])
    investigation_file = network.parent.parent/'investigation.json'
    investigation = json.loads(investigation_file.read_text())
    town = source_manifest['map'].split('/')[-1]
    map_investigation = next(m for m in investigation['maps'] if m['town'] == town)
    attempt = next(a for a in map_investigation['attempts']
                   if a['kind'] == 'official-carla' and a.get('network', {}).get('sha256') == sha(network))
    network_root = ET.parse(network).getroot()
    offset = tuple(map(float, network_root.find('location').get('netOffset').split(',')))
    BridgeHelper.offset = offset
    lanes = {l.get('id'): {p.get('key'): p.get('value') for p in l.findall('param')}
             for e in network_root.findall('edge') for l in e.findall('lane')}
    mapping_file = replay/'mapping.jsonl'
    mappings = [json.loads(line) for line in mapping_file.read_text().splitlines()]
    source_rows = source_rows[:sample_argument]
    source_xodr = run/'map.xodr'
    original_converter = Path(investigation['official_converter'])
    bridge_file = CARLA_SUMO/'sumo_integration/bridge_helper.py'
    source_live = source_manifest.get('live_co_simulation', {})
    source_sync_file = run/'sumo-sync.jsonl'
    source_bound, source_binding_method = outer_source_binding(process, manifest_file, run, source_live)
    checks = {
        'replay_status_complete': manifest['status'] == 'complete',
        'explicit_post_capture_mode_and_no_live_claim': manifest['mode'] == 'post_capture_synchronized_replay' and manifest['two_way_live_control_claimed'] is False,
        'no_carla_connection_or_tick_claimed': manifest['carla_connected_or_ticked'] is False,
        'no_interpolated_frames': manifest['interpolated_frames'] == 0,
        'native_gui_viewport_claimed': manifest['native_gui_only'] is True,
        'actual_outer_replay_process_returncode_zero': process['returncode'] == 0,
        'actual_sumo_process_returncode_zero_recorded': manifest['sumo_process_exit_code'] == 0,
        'raw_carla_process_exit_and_cleanup_complete': execution['returncode'] == 0 and source_manifest['status'] == 'complete' and source_manifest['native_cleanup_complete'] is True,
        'raw_source_manifest_hash_matches_both_records': sha(source_manifest_file) == execution['manifest_sha256'] == manifest['source_manifest_sha256'],
        'raw_source_frames_hash_matches_both_records': sha(source_frames_file) == execution['frames_jsonl_sha256'] == manifest['source_frames_jsonl_sha256'],
        'raw_execution_hash_matches': sha(execution_file) == manifest['source_execution_sha256'],
        'network_actual_hash_matches_record_and_conversion': sha(network) == manifest['network_sha256'] == attempt['network']['sha256'],
        'network_source_xodr_matches_actual_capture_and_inventory': sha(source_xodr) == map_investigation['source_xodr_sha256'] == sha(Path(map_investigation['source_xodr'])),
        'official_converter_current_hash_matches_provenance': sha(original_converter) == investigation['official_converter_sha256'],
        'official_network_conversion_returncode_zero': attempt['actual_returncode'] == 0,
        'official_bridge_current_hash_matches_record': sha(bridge_file) == manifest['official_bridge_helper_sha256'],
        'mapping_current_hash_matches_manifest': sha(mapping_file) == manifest['mapping_sha256'],
        'sumo_log_hash_matches_manifest': sha(replay/'sumo-gui.log') == manifest['sumo_log_sha256'],
        'source_fps_20_matches_replay': source_manifest['fps'] == manifest['source_fps'] == 20,
        'network_offset_matches_real_xml': list(offset) == manifest['network_offset_m'],
        'mapping_sample_count_matches_source_cut_and_manifest': len(mappings) == len(source_rows) == manifest['samples_mapped'] == sample_argument,
        'source_recorded_live_bidirectional_mode': source_live.get('mode') == 'live_official_bidirectional_actor_synchronization' and source_live.get('postcapture_replay') is False,
        'source_live_synchronization_evidence_hash_matches': source_sync_file.is_file() and sha(source_sync_file) == source_live.get('sumo_sync_jsonl_sha256'),
        'source_live_network_matches_visualization_network': source_live.get('network_sha256') == sha(network),
        'source_live_bridge_matches_visualization_bridge': source_live.get('official_bridge_sha256') == sha(bridge_file),
        'outer_replay_record_binds_exact_immutable_actual_live_source': source_bound,
        'outer_execution_manifest_hash_matches': process.get('manifest_sha256') == sha(manifest_file),
        'outer_execution_mapping_hash_matches': process.get('mapping_sha256') == sha(mapping_file),
        'outer_execution_log_hash_matches': process.get('log_sha256') == sha(Path(process['log'])),
    }
    failures, source_frame_ids, actor_count = [], [], 0
    maximum = {name: 0.0 for name in ['source_timestamp_error_s', 'display_time_error_s',
               'display_source_origin_error_s', 'sumo_clock_error_s', 'manual_vs_official_front_error_m',
               'target_vs_official_front_error_m', 'readback_vs_official_front_error_m',
               'readback_vs_official_angle_error_deg', 'inverse_actor_origin_xy_error_m',
               'reported_vs_independently_computed_inverse_error_m']}
    mismatch_native_lane_queries = 0
    png_sizes, png_hashes = set(), []
    valid_png_count = 0

    def fail(kind, sample, **data):
        failures.append({'kind': kind, 'sample': sample, **data})

    for index, record in enumerate(mappings):
        if index >= len(source_rows):
            fail('mapping_has_no_source_sample', index)
            continue
        source = source_rows[index]
        sample, frame = source['sample'], source['frame']
        source_frame_ids.append(frame)
        if record['source_sample'] != sample or record['source_frame'] != frame or sample != index:
            fail('source_sample_or_frame_id_mismatch', sample)
        time_errors = {
            'source_timestamp_error_s': abs(record['source_timestamp_s']-source['timestamp']),
            'display_time_error_s': abs(record['display_time_s']-source['time_s']),
            'display_source_origin_error_s': abs(record['display_time_s']-(source['timestamp']-source_rows[0]['timestamp'])),
            'sumo_clock_error_s': abs(record['sumo_replay_time_s']-(index+1)/source_manifest['fps'])}
        for name, value in time_errors.items():
            maximum[name] = max(maximum[name], value)
            if value > 1e-6:
                fail(name, sample, error=value)
        expected_png = replay/'frames_native_sumo'/f'{frame:08d}.png'
        if Path(record['native_sumo_png']).resolve() != expected_png:
            fail('native_png_path_or_frame_filename_mismatch', sample)
        if not expected_png.exists():
            fail('missing_png', sample)
        else:
            digest = sha(expected_png)
            if digest != record['native_sumo_png_sha256']:
                fail('png_hash_mismatch', sample)
            try:
                with Image.open(expected_png) as image:
                    image.load()
                    png_sizes.add(image.size)
                    if image.size != (1280, 720) or image.size != tuple(manifest['viewport_px']):
                        fail('native_png_dimensions_mismatch', sample, actual=list(image.size))
                    if not any(hi-lo > 20 for lo, hi in image.convert('RGB').getextrema()):
                        fail('uniform_or_blank_png', sample)
                valid_png_count += 1
                png_hashes.append(digest)
            except OSError as exc:
                fail('native_png_decode_failed', sample, error=str(exc))
        source_actors = {a['id']: a for a in source['vehicles']}
        actual_actor_ids = [a['carla_id'] for a in record['actors']]
        if len(set(actual_actor_ids)) != len(actual_actor_ids) or set(actual_actor_ids) != set(source_actors):
            fail('actor_id_set_mismatch_or_duplicate', sample)
        for mapped in record['actors']:
            native = source_actors.get(mapped['carla_id'])
            if native is None:
                fail('actor_missing_in_source', sample, id=mapped['carla_id'])
                continue
            actor_count += 1
            t = native['transform']
            rotation = t['rotation_deg']
            original_t = carla.Transform(carla.Location(*t['location_m']),
                carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2]))
            extent = carla.Vector3D(*native['bounding_box']['extent_m'])
            target = BridgeHelper.get_sumo_transform(original_t, extent)
            official_xy = [target.location.x, target.location.y]
            official_angle = target.rotation.yaw % 360
            manual_xy, manual_angle = scalar_front(native, offset)
            actual_xy, actual_angle = mapped['readback_sumo_front_bumper_xy_m'], mapped['readback_sumo_nav_angle_deg']
            inverse_xy = scalar_inverse(actual_xy, actual_angle, native['bounding_box']['extent_m'][0], offset)
            inverse_error = math.dist(inverse_xy, t['location_m'][:2])
            actor_errors = {
                'manual_vs_official_front_error_m': math.dist(manual_xy, official_xy),
                'target_vs_official_front_error_m': math.dist(mapped['target_sumo_front_bumper_xy_m'], official_xy),
                'readback_vs_official_front_error_m': math.dist(actual_xy, official_xy),
                'readback_vs_official_angle_error_deg': angle_distance(actual_angle, official_angle),
                'inverse_actor_origin_xy_error_m': inverse_error,
                'reported_vs_independently_computed_inverse_error_m': abs(mapped['inverse_carla_center_xy_error_m']-inverse_error)}
            tolerances = {'inverse_actor_origin_xy_error_m': .0001,
                          'reported_vs_independently_computed_inverse_error_m': .000001}
            for name, value in actor_errors.items():
                maximum[name] = max(maximum[name], value)
                if value > tolerances.get(name, .000001):
                    fail(name, sample, actor_id=native['id'], error=value)
            if angle_distance(manual_angle, official_angle) > 1e-6 or angle_distance(mapped['target_sumo_nav_angle_deg'], official_angle) > 1e-6:
                fail('manual_or_recorded_target_angle_mismatch', sample, actor_id=native['id'])
            same_source_fields = (mapped['role'] == native['role_name'] and mapped['ego'] == native['ego']
                and mapped['sumo_id'] == 'carla_'+str(native['id'])
                and mapped['carla_actor_center_m'] == t['location_m']
                and mapped['carla_yaw_deg'] == rotation[1]
                and mapped['bbox_extent_m'] == native['bounding_box']['extent_m']
                and mapped['recorded_native_nearest_waypoint'] == native['waypoint']
                and mapped['source_speed_mps'] == native['speed_mps'])
            if not same_source_fields:
                fail('serialized_source_actor_fields_mismatch', sample, actor_id=native['id'])
            if abs(mapped['xy_error_m']-math.dist(actual_xy, official_xy)) > 1e-6 or abs(mapped['yaw_error_deg']-angle_distance(actual_angle, official_angle)) > 1e-6:
                fail('reported_readback_error_mismatch', sample, actor_id=native['id'])
            lane = mapped['readback_sumo_lane_id']
            if not lane or lane not in lanes:
                fail('readback_lane_missing_from_actual_network', sample, actor_id=native['id'], lane=lane)
            elif mapped['readback_lane_origId'] != lanes[lane].get('origId', ''):
                fail('actual_readback_lane_original_id_mismatch', sample, actor_id=native['id'], lane=lane)
            original_pair = f"{native['waypoint']['road_id']}_{native['waypoint']['lane_id']}"
            if original_pair not in (mapped['readback_lane_origId'] or '').split():
                mismatch_native_lane_queries += 1
    png_files = set((replay/'frames_native_sumo').glob('*.png'))
    expected_files = {replay/'frames_native_sumo'/f'{f:08d}.png' for f in source_frame_ids}
    checks.update({
        'native_png_names_complete_no_extra_or_missing_frames': png_files == expected_files and len(png_files) == manifest['native_png_count'],
        'native_png_all_hashes_dimensions_and_full_decodes_valid': valid_png_count == len(source_rows) and not any(f['kind'].startswith(('png_', 'native_png_', 'uniform_', 'missing_png')) for f in failures),
        'source_frame_ids_contiguous_no_offset': all(b-a == 1 for a, b in zip(source_frame_ids, source_frame_ids[1:])),
        'actor_count_matches_manifest': actor_count == manifest['actors_mapped'],
        'all_frame_source_time_mapping_and_coordinate_checks_pass': not failures,
        'native_png_motion_or_view_changes_present': len(set(png_hashes)) > 1,
    })
    report = {'audit_type': 'independent_all_sample_native_SUMO_replay_mapping',
        'created_utc': datetime.now(timezone.utc).isoformat(), 'replay': str(replay), 'source_run': str(run),
        'mode': manifest['mode'], 'passed': all(checks.values()), 'checks': checks,
        'failed_checks': [name for name, value in checks.items() if not value],
        'failures_count': len(failures), 'failures_examples': failures[:40],
        'samples_checked': len(mappings), 'actors_checked': actor_count, 'native_pngs_fully_decoded': valid_png_count,
        'native_png_dimensions': [list(s) for s in sorted(png_sizes)],
        'first_source_frame': source_frame_ids[0], 'last_source_frame': source_frame_ids[-1],
        'maximum_recomputed_errors': maximum,
        'time_origins': {'source_timestamp_origin_s': source_rows[0]['timestamp'],
                         'source_display_origin_s': source_rows[0]['time_s'],
                         'sumo_first_recorded_time_s': mappings[0]['sumo_replay_time_s'],
                         'meaning': 'CARLA native frame/timestamp is exactly matched to each source record. Replay display starts at zero; first SUMO replay step is 0.05s. These explicit clock origins are not a one-frame source shift.'},
        'nearest_lane_original_id_differences': mismatch_native_lane_queries,
        'nearest_lane_note': 'Native CARLA queries use actor origin; SUMO mapping uses front bumper. Junction overlapping/internal connectors can select different lane IDs even at exact same physical pose. Original IDs and network membership are verified; lane IDs are not forced equal.',
        'input_sha256': {'replay_manifest': sha(manifest_file), 'mapping': sha(mapping_file),
                         'raw_manifest': sha(source_manifest_file), 'raw_frames': sha(source_frames_file),
                         'raw_execution': sha(execution_file), 'raw_xodr': sha(source_xodr),
                         'network': sha(network), 'network_investigation': sha(investigation_file),
                         'official_converter': sha(original_converter), 'official_bridge_helper': sha(bridge_file),
                         'outer_replay_process_record': sha(process_file), 'auditor': sha(Path(__file__))},
        'network_provenance': {'investigation': str(investigation_file), 'source_xodr': map_investigation['source_xodr'],
                                'official_netconvert_returncode': attempt['actual_returncode'],
                                'official_import_warning_count': attempt['warning_count']},
        'source_capture_mode': source_live.get('mode'),
        'outer_source_binding_method': source_binding_method,
        'legacy_outer_source_was_live_capture_field': process.get('source_was_live_capture'),
        'source_live_synchronization_jsonl_sha256': sha(source_sync_file),
        'source_vs_visualization': 'The immutable original capture records official live bidirectional co-simulation. This independent native-GUI visualization is generated after capture from saved CARLA actor transforms and remains post-capture replay; it does not operate live co-simulation.',
        'speed_semantics': 'Source speed_mps is copied without alteration. Physics-disabled SUMO mirror actors may have native CARLA velocity zero despite moving saved transforms. Replay pose is determined by those transforms; no substitute speed or autonomous policy is claimed.',
        'limitations': ['This visualization is immutable post-capture replay from a live source, not a new two-way real-time control run. The independent live-control evidence audit is separate.',
                       'Native GUI provenance is supported by process, command, screenshot hashes and recorded TraCI mapping. The auditor reads existing evidence, not a new GUI capture.',
                       'Official BridgeHelper maps CARLA actor-origin to front bumper with bbox extent; local bbox-center offsets are not an alternate transform.',
                       '3D height and exact SUMO velocity dynamics are not claimed to be replayed by the 2D GUI pane.',
                       'This audit does not validate the encoded MP4 pane; a separate video audit is required.'],
        'raw_or_replay_modified': False, 'CARLA_or_SUMO_connected_or_ticked': False}
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'scene': replay.name, 'passed': report['passed'], 'report': str(out.resolve()),
                      'samples': len(mappings), 'actor_count': actor_count,
                      'failures_count': len(failures), 'failed_checks': report['failed_checks'],
                      'max_errors': maximum}), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--process-file', type=Path, required=True)
    args = parser.parse_args()
    base = Path(__file__).resolve().parent
    if args.replay is None or args.out is None:
        parser.error('Use --replay with --out and --process-file.')
    return 0 if audit(args.replay, args.out, args.process_file)['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
