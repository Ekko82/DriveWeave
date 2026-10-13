"""Source-bound supplementary roundabout geometry decision, preserving strict FAIL.

Only the reviewed frozen capture is eligible. The road12 proof uses two actual
20Hz positions and their recorded bodies. Interpolation never becomes native
samples, and finite sampled geometry never establishes continuous-time safety.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
import carla

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'carla'), str(ROOT/'carla/verification')]
from audit_native_actions import obb
from audit_junction_interaction import convex_hull, project
from audit_v3_junction import native_reference_checked

SOURCE_SHA = {
    'manifest.json': 'd679e884490ec478b038e06b08b1b9ee227d2427ee7ec8f65f9568917faafeed',
    'frames.jsonl': '0fc81f974a70017368b11c3a0a120820b92d5ce7c7624cc7bc7395615859cdff',
    'map.xodr': '6b7b80d8396990433bf9e152fc4a21b9aeb2dc2cfe46bba8df6cc8880c29af7c',
    'collisions.json': '4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945',
    'controller-events.json': 'c945e74da441bf5e1d793627d797b980430ea296548d7a5fd110a41afce69bd6',
}
STRICT_SHA = '1638d60a2b5448fb7f7a29ef1119ff7993a276ea3cf89d4ef43738bd38d98dc6'
CORRIDOR_SHA = '239a3fca72564071b2348c8460c2ef3b9e271f6802e94d194e644f8f59384be6'
DEPENDENCIES = {
    'carla/audit_native_actions.py': '8fa1a9888bd502f367a50e7bdfd8be06fedd2bb3fa94d388d7aaf5081157e719',
    'carla/audit_junction_interaction.py': 'ed0cce35fb663e275cbe0fa150b129561ae82941e8efdee83f7088c06a9c619f',
    'carla/audit_v3_junction.py': '1b0e59eaac70942f4aba9d99167e21400fc4dcc3b5d842f625e34d913b8c786f',
    'carla/audit_v3_entry_corridors.py': '3ec34170d13b0240307d136151ead2d336e24e30c178fcf58e5e112404b414b0',
}
EXPECTED_FAILURES = ['actual_native_ring_and_exit_road_order',
                     'second_entry_north_actual_second_native_entry']
CORRIDOR_FAILURES = ['only_original_wide_gate_failure',
                    'original_all_other_source_motion_path_and_clearance_checks_pass']


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def require(value, message):
    if not value:
        raise ValueError(message)


def check_bound_files(bindings):
    for path, expected in bindings.items():
        require(sha(path) == expected, 'Exact frozen source mismatch: '+str(path))


def contains(poly, point):
    p = np.asarray(poly, float); delta = np.asarray(point, float)[:2]-p
    edges = np.roll(p, -1, axis=0)-p
    cross = edges[:, 0]*delta[:, 1]-edges[:, 1]*delta[:, 0]
    return bool(len(p) >= 3 and (np.all(cross >= -1e-7) or np.all(cross <= 1e-7)))


def bracket_geometry(left, right, start_xyz, end_xyz):
    """No interpolated observation is used in the acceptance predicate."""
    a,b = np.asarray(start_xyz, float)[:2], np.asarray(end_xyz, float)[:2]
    length = float(np.linalg.norm(b-a))
    require(length > 1e-6, 'Degenerate native road extent')
    tangent = (b-a)/length; normal = np.array([-tangent[1], tangent[0]])
    pa,pb = np.asarray(left['xyz']), np.asarray(right['xyz'])
    along = [float((p[:2]-a)@tangent) for p in (pa,pb)]
    lateral = [float((p[:2]-a)@normal) for p in (pa,pb)]
    dt = right['timestamp']-left['timestamp']
    displacement = float(np.linalg.norm(pb-pa))
    heading = math.degrees(math.atan2(tangent[1], tangent[0]))
    errors = [abs((sample['yaw_deg']-heading+180)%360-180) for sample in (left,right)]
    coverage = [all(contains(s['body_xy'],p) for p in (a,b)) for s in (left,right)]
    # Same lateral/yaw limits as the retained native tracking check. The extra
    # displacement bound rejects an implausible jump even if endpoints bracket.
    checks = {
        'consecutive_recorded_frames': right['frame'] == left['frame']+1 and right['sample'] == left['sample']+1,
        'native_20hz_interval': abs(dt-.05) < 2e-6,
        'full_native_road_endpoints_bracketed': along[0] < 0 and along[1] > length,
        'actual_centers_within_original_tracking_width': max(abs(x) for x in lateral) <= 1.,
        'actual_headings_within_original_tracking_limit': max(errors) <= 15.,
        'both_actual_bodies_cover_complete_native_road': all(coverage),
        'short_road_smaller_than_one_actual_sample_step': length < displacement,
        'actual_step_consistent_with_recorded_speed': displacement <= min(1., max(left['speed_mps'],right['speed_mps'])*max(0.,dt)+.05),
    }
    return {'passed': all(checks.values()), 'checks': checks, 'frames': [left['frame'],right['frame']],
        'samples': [left['sample'],right['sample']], 'timestamps': [left['timestamp'],right['timestamp']],
        'native_centerline_extent_m': length, 'interval_s': dt, 'actual_displacement_m': displacement,
        'along_from_native_start_m': along, 'cross_track_m': lateral, 'heading_errors_deg': errors,
        'actual_body_endpoint_coverage': coverage}


def strict_failures_eligible(strict):
    return (strict.get('scenario_actions_validated') is False
        and strict.get('failed_checks') == EXPECTED_FAILURES
        and all(strict['checks'].get(k) is False for k in EXPECTED_FAILURES)
        and all(v is True for k,v in strict['checks'].items() if k not in EXPECTED_FAILURES))


def corridor_eligible(corridor, expected_roles):
    checks = corridor['checks']; pairs = corridor['pair_reviews']
    return (corridor.get('supplementary_precise_geometry_passed') is False
        and corridor.get('failed_checks') == CORRIDOR_FAILURES
        and all(checks.get(k) is False for k in CORRIDOR_FAILURES)
        and all(v is True for k,v in checks.items() if k not in CORRIDOR_FAILURES)
        and set(pairs) == set(expected_roles)
        and all(p['precise_conflict_simultaneous_samples'] == 0 and p['precise_conflict_samples'] == []
                and p['native_swept_body_conflict_patch_count'] > 0
                and p['minimum_entry_phase_obb']['clearance_m'] >= (1.5 if role == 'hero' else .75)
                for role,p in pairs.items()))


def subsequence(required, values):
    index = 0
    for value in values:
        if index < len(required) and value == required[index]: index += 1
    return index == len(required)


def wp_record(wp):
    p = wp.transform.location
    return {'road_id': wp.road_id, 'lane_id': wp.lane_id, 's': wp.s,
            'xyz': [p.x,p.y,p.z], 'yaw_deg': wp.transform.rotation.yaw,
            'lane_width_m': wp.lane_width}


def audit(run, strict_path, corridor_path):
    own_source_sha = sha(__file__)
    bindings = {run/name:digest for name,digest in SOURCE_SHA.items()}
    bindings.update({strict_path: STRICT_SHA, corridor_path: CORRIDOR_SHA})
    bindings.update({ROOT/name:digest for name,digest in DEPENDENCIES.items()})
    check_bound_files(bindings)
    strict = json.loads(strict_path.read_text()); corridor = json.loads(corridor_path.read_text())
    require(strict['input_sha256'] == SOURCE_SHA, 'Strict report native-source binding mismatch')
    require(corridor['source_sha256'] == SOURCE_SHA and corridor['strict_report_sha256'] == STRICT_SHA,
            'Corridor report source/strict binding mismatch')
    require(strict['auditor_sha256'] == DEPENDENCIES['carla/audit_v3_junction.py']
            and corridor['auditor_sha256'] == DEPENDENCIES['carla/audit_v3_entry_corridors.py'],
            'Retained original auditor implementations differ')
    require(strict_failures_eligible(strict), 'Strict failures are not exactly the two reviewed failures')
    manifest = json.loads((run/'manifest.json').read_text()); plan = manifest['route_specification']
    rows = [json.loads(line) for line in (run/'frames.jsonl').read_text().splitlines()]
    native_map = carla.Map(plan['map'], (run/'map.xodr').read_text())
    native_reference = native_reference_checked(native_map,plan['stages'])
    road = next(e for e in ET.parse(run/'map.xodr').getroot().findall('road') if e.get('id') == '12')
    road_length = float(road.attrib['length'])
    path = [p for stage in plan['stages'] for p in stage['xodr_path']]
    target_indices = [i for i,p in enumerate(path) if p[0] == 12]
    require(target_indices and target_indices == list(range(min(target_indices),max(target_indices)+1)),
            'Expected one contiguous native road12 reference interval')
    lanes = {path[i][1] for i in target_indices}; require(len(lanes)==1,'Ambiguous target lane')
    lane = next(iter(lanes))
    start,end = [native_map.get_waypoint_xodr(12,lane,s) for s in (0.,road_length)]
    require(start is not None and end is not None, 'Missing full native road12 endpoints')
    endpoints = [wp_record(start),wp_record(end)]
    before_ref,after_ref = path[min(target_indices)-1],path[max(target_indices)+1]
    second = next(s for s in plan['traffic'] if s['name']==plan['expected_evidence']['second_entry_role'])
    roles = next(s['yield_gate']['actor_roles'] for s in second['stages'] if 'yield_gate' in s)
    require(corridor_eligible(corridor,roles),'Original corridor geometry or clearance proof is insufficient')
    samples = []
    for row in rows:
        vehicle = next(v for v in row['vehicles'] if v['role_name']=='hero')
        xyz = vehicle['transform']['location_m']; yaw = vehicle['transform']['rotation_deg'][1]
        fit = project(xyz,yaw,native_reference)
        samples.append({'sample': row['sample'],'frame': row['frame'],'timestamp': row['timestamp'],
            'xyz': xyz,'yaw_deg': yaw,'speed_mps': vehicle['speed_mps'],
            'body_xy': convex_hull(obb(vehicle)['vertices'][:,:2]), 'native_reference_fit': fit,
            'nearest_native_waypoint': wp_record(native_map.get_waypoint(carla.Location(*xyz)))})
    matches = []
    for left,right in zip(samples,samples[1:]):
        proof = bracket_geometry(left,right,endpoints[0]['xyz'],endpoints[1]['xyz'])
        if proof['passed']:
            proof['actual_samples']=[left,right];matches.append(proof)
    require(len(matches)==1,'One unambiguous actual-pose bracket is required')
    matched = matches[0]; left,right = matched['actual_samples']
    native_order = strict['evidence']['actual_native_ring_and_exit_road_order']
    require(12 in native_order['required'] and 12 not in native_order['observed']
        and subsequence([x for x in native_order['required'] if x!=12],native_order['observed']),
        'Required road order has another unresolved omission')
    require(left['nearest_native_waypoint']['road_id']==before_ref[0]
        and right['nearest_native_waypoint']['road_id']==after_ref[0],
        'Actual bracket does not join the native preceding and succeeding roads')
    checks = {
        'exact_frozen_native_and_reports_bound': True,
        'strict_exact_two_failures_retained_and_all_other_checks_true': strict_failures_eligible(strict),
        'entire_original_native_path_and_successor_topology_valid': True,
        'road12_only_missing_required_road_label': True,
        'exactly_one_actual_20hz_pose_pair_brackets_full_native_road12': len(matches)==1,
        'actual_bracket_joins_native_predecessor_and_successor': True,
        'all_actual_hero_tracking_with_original_thresholds': all(s['native_reference_fit']['lateral_distance_m']<=1.
            and s['native_reference_fit']['reference_heading_error_deg']<=15. for s in samples),
        'both_actual_bodies_cover_complete_native_road12': all(matched['actual_body_endpoint_coverage']),
        'original_corridor_all_seven_pairs_have_no_sampled_conflict_and_original_clearance': corridor_eligible(corridor,roles),
        'original_corridor_failed_status_is_preserved': corridor['supplementary_precise_geometry_passed'] is False,
    }
    check_bound_files(bindings)
    require(sha(__file__)==own_source_sha,'Supplement auditor changed during execution')
    return {'schema':'source_bound_roundabout_sampled_geometry_supplement_v1',
        'qualified_sampled_geometry_acceptance': all(checks.values()),
        'status': 'qualified_sampled_geometry_accepted_with_retained_strict_failures' if all(checks.values()) else 'rejected',
        'ordinary_strict_action_passed': False,'strict_action_failed_checks_retained':strict['failed_checks'],
        'original_corridor_failed_checks_retained':corridor['failed_checks'],
        'checks':checks,'failed_checks':[k for k,v in checks.items() if not v],
        'run':str(run.resolve()),'source_sha256':SOURCE_SHA,'strict_report_sha256':STRICT_SHA,
        'corridor_report_sha256':CORRIDOR_SHA,'dependency_sha256':DEPENDENCIES,'auditor_sha256':own_source_sha,
        'native_road12_xodr_length_m':road_length,'native_road12_full_endpoints':endpoints,
        'native_preceding_reference':before_ref,'native_succeeding_reference':after_ref,
        'actual_road12_bracket':matched,'original_nearest_road_order':native_order,
        'original_corridor_pair_reviews':corridor['pair_reviews'],
        'method':['Bind exact original files and retained strict/corridor reports before and after evaluation.',
            'Resolve the entire native reference and successor topology from the frozen OpenDRIVE map offline.',
            'Require one actual consecutive 20Hz pose pair to lie before/after the full road12 centerline extent.',
            'Require both actual recorded full-body footprints to cover both full native endpoints.',
            'Keep original 1m/15deg tracking and all original physical motion/clearance/collision checks mandatory.',
            'Retain the original corridor report failure and reuse only its source-bound seven-pair sampled conflict geometry.'],
        'limitations':['The original strict road-order and broad-gate checks remain FAIL; this is a separate qualified sampled-geometry decision.',
            'No interpolated point is used as a native sample or as an acceptance condition.',
            'Finite reference corridors and 20Hz recorded bodies do not prove arbitrary continuous-time safety.',
            'Eligibility applies only to the exact frozen capture and reports, not unseen executions.',
            'Sensor processing and encoded video quality require separate audits.'],
        'source_modified':False,'server_connected_or_ticked':False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--strict-report',type=Path,required=True)
    parser.add_argument('--corridor-report',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    result=audit(args.run,args.strict_report,args.corridor_report)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    with args.out.open('x',encoding='utf8') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({'out':str(args.out),'status':result['status'],'failed_checks':result['failed_checks'],
        'bracket_frames':result['actual_road12_bracket']['frames']}))
    return 0 if result['qualified_sampled_geometry_acceptance'] else 1


if __name__=='__main__':
    raise SystemExit(main())
