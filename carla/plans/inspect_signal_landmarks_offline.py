"""Inspect installed native Town05 light landmarks without Client/world tick."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from runtime_config import RuntimeConfig
from signal_policy import town05_wide_left_signal_config, _incoming_reference, _wp_record, _angle
import carla
import hashlib
import json
import math


def main():
    root = RuntimeConfig.resolve(require_carla=True).carla_root
    source = root / 'CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr'
    m = carla.Map('Town05', source.read_text(encoding='utf-8'))
    config = town05_wide_left_signal_config()
    approaches = {(q['road_id'], lane): (q, state)
                  for name, state in [('green_approaches', 'Green'), ('red_approaches', 'Red')]
                  for q in config[name] for lane in q['lane_ids']}
    cx, cy = config['junction_center_xy_m']
    records, covered, signal_states = [], set(), {}
    for l in m.get_all_landmarks():
        loc = l.transform.location
        if l.type != '1000001' or math.hypot(loc.x-cx, loc.y-cy) > 40:
            continue
        for lo, hi in l.get_lane_validities():
            for lane in range(min(lo, hi), max(lo, hi)+1):
                if not lane:
                    continue
                w = m.get_waypoint_xodr(l.road_id, lane, l.s)
                incoming, back = _incoming_reference(w, 1126, 60)
                key = (incoming.road_id, incoming.lane_id)
                if key not in approaches:
                    raise RuntimeError('Native landmark incoming lane is not in signal policy: ' + repr(key))
                approach, state = approaches[key]
                error = _angle(incoming.transform.rotation.yaw, approach['yaw_deg'])
                if error > config['max_heading_error_deg']:
                    raise RuntimeError('Native landmark direction does not match the proposed policy')
                covered.add(key)
                signal_states.setdefault(str(l.id), set()).add(state)
                records.append({'landmark_id': str(l.id), 'native_type': l.type,
                                'native_name': l.name, 'native_orientation': str(l.orientation),
                                'native_pole_xyz_m': [loc.x, loc.y, loc.z],
                                'native_affected_waypoint': _wp_record(w),
                                'resolved_external_incoming_waypoint': _wp_record(incoming),
                                'native_predecessor_trace_m': back, 'role': approach['role'],
                                'required_state': state, 'heading_error_deg': error})
    missing = sorted(set(approaches)-covered)
    if missing or any(len(states) != 1 for states in signal_states.values()):
        raise RuntimeError('Incomplete or incompatible native landmark evidence: ' + repr(missing))
    evidence = {
        'map': 'Town05', 'junction_id': 1126, 'signalized': True,
        'source_root_env': 'CARLA_ROOT', 'source_xodr': source.relative_to(root).as_posix(),
        'source_xodr_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'server_connected': False, 'world_ticked': False,
        'landmarks_are_not_runtime_get_stop_waypoints': True,
        'runtime_stop_waypoint_classification_required': True,
        'runtime_states_and_actions_verified': False,
        'desired_native_landmark_states': {k: next(iter(v)) for k, v in signal_states.items()},
        'all_expected_incoming_lanes_covered': True, 'config': config, 'native_landmarks': records,
        'global_freeze_api_scope': 'CARLA TrafficLight.freeze acts on all episode lights; disabled by default.',
        'timer_hold_runtime_validation_required': True,
    }
    out = Path(__file__).resolve().parents[1] / 'verification/v2-town05-signal-landmarks-offline.json'
    out.write_text(json.dumps(evidence, indent=2), encoding='utf-8')
    print(json.dumps({'evidence': out.relative_to(out.parents[1]).as_posix(),
                      'native_landmarks': len(records), 'signals': evidence['desired_native_landmark_states'],
                      'covered_lanes': sorted(covered), 'runtime_accepted': False}))


if __name__ == '__main__':
    main()
