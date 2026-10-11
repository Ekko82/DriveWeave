"""Read-only, narrowly scoped qualification of one invisible background boundary.

The original all-lane audit remains FAIL. This independent audit accepts only
one source-pinned background sample at a connected internal-to-normal endpoint,
within 0.1 m and with its complete footprint outside an actual native GUI view.
It never fills lane fields, changes poses, or relaxes source/sensor/physics gates.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

TYPE = 'qualified_single_background_endpoint_native_SUMO_replay_mapping'
EXCEPTION_CHECK = 'all_frame_source_time_mapping_and_coordinate_checks_pass'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def rows(path):
    return [json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines() if line.strip()]


def strict_failure_gate(strict, scope):
    expected = {'kind': 'readback_lane_missing_from_actual_network',
                'sample': scope['sample'], 'actor_id': scope['actor_id'], 'lane': ''}
    require(strict.get('audit_type') == 'independent_all_sample_native_SUMO_replay_mapping'
            and strict.get('passed') is False and strict.get('failures_count') == 1
            and strict.get('failures_examples') == [expected]
            and strict.get('failed_checks') == [EXCEPTION_CHECK],
            'Qualification permits exactly one original empty-lane failure')
    checks = strict.get('checks', {})
    require(bool(checks) and checks.get(EXCEPTION_CHECK) is False
            and all(v is True for k, v in checks.items() if k != EXCEPTION_CHECK),
            'Another strict source, coordinate, time, process or PNG check failed')
    require(strict.get('samples_checked') == scope['expected_samples']
            and strict.get('actors_checked') == scope['expected_actor_samples']
            and strict.get('native_pngs_fully_decoded') == scope['expected_samples'],
            'Only the pinned full source range may be qualified')
    require(strict.get('input_sha256') == scope['strict_input_sha256'],
            'Strict inputs differ from the explicitly approved immutable source')


def body_corners(actor):
    x, y = actor['readback_sumo_front_bumper_xy_m']
    hx, hy = actor['bbox_extent_m'][:2]
    require(hx > 0 and hy > 0, 'Invalid physical footprint')
    theta = math.radians(actor['readback_sumo_nav_angle_deg'])
    forward = (math.sin(theta), math.cos(theta))
    side = (-forward[1], forward[0])
    cx, cy = x-hx*forward[0], y-hx*forward[1]
    return [(cx+a*hx*forward[0]+b*hy*side[0], cy+a*hx*forward[1]+b*hy*side[1])
            for a, b in ((-1,-1), (-1,1), (1,1), (1,-1))]


def completely_outside(actor, boundary):
    (xmin, ymin), (xmax, ymax) = boundary
    require(xmin < xmax and ymin < ymax, 'Invalid actual GUI viewport')
    rectangle = [(xmin,ymin), (xmin,ymax), (xmax,ymax), (xmax,ymin)]
    body = body_corners(actor)
    # Separating-axis test: an outside front bumper alone is insufficient.
    for polygon in (body, rectangle):
        for i, p in enumerate(polygon):
            q = polygon[(i+1) % 4]
            axis = (-(q[1]-p[1]), q[0]-p[0])
            a = [axis[0]*x+axis[1]*y for x,y in body]
            b = [axis[0]*x+axis[1]*y for x,y in rectangle]
            if max(a) < min(b) or max(b) < min(a):
                return True
    return False


def lane_table(network):
    return {lane.get('id'): (edge, lane) for edge in network.findall('edge') for lane in edge.findall('lane')}


def shape(lane):
    return [tuple(map(float, p.split(',')[:2])) for p in lane.get('shape').split()]


def point_line_distance(point, points):
    result = math.inf
    for a,b in zip(points, points[1:]):
        dx,dy = b[0]-a[0], b[1]-a[1]
        den = dx*dx+dy*dy
        t = max(0,min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/den)) if den else 0
        result = min(result, math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy))
    return result


def qualification_facts(mappings, network, proof, scope):
    require(len(mappings) == scope['expected_samples']
            and sum(len(r['actors']) for r in mappings) == scope['expected_actor_samples'],
            'Full source/actor range differs from the scope')
    empty = [(i,a) for i,r in enumerate(mappings) for a in r['actors']
             if not a['readback_sumo_lane_id']]
    require(len(empty) == 1, 'Exactly one empty actor-lane sample is allowed')
    sample, actor = empty[0]
    require(sample == scope['sample'] and actor['carla_id'] == scope['actor_id']
            and mappings[sample]['source_frame'] == scope['source_frame'],
            'The sole empty lane is outside the approved sample/actor/frame')
    require(actor['ego'] is False and actor['role'] == scope['role']
            and actor['role'].startswith('dynamic_bg_'), 'Ego or non-background qualification refused')
    require(0 < sample < len(mappings)-1, 'Boundary sample needs both adjacent observations')
    require(all(any(a['ego'] for a in r['actors']) and
                all(a['readback_sumo_lane_id'] for a in r['actors'] if a['ego']) for r in mappings),
            'Every source ego observation must have actual network membership')
    adjacent = []
    for index in (sample-1, sample+1):
        matches = [a for a in mappings[index]['actors'] if a['carla_id'] == actor['carla_id']]
        require(len(matches) == 1 and matches[0]['role'] == actor['role'] and matches[0]['ego'] is False,
                'Background identity/adjacent observations changed')
        adjacent.append(matches[0])
    lanes = lane_table(network)
    before, after = adjacent
    require(before['readback_sumo_lane_id'] in lanes and after['readback_sumo_lane_id'] in lanes,
            'Both adjacent lanes must exist in the actual converted network')
    be,bl = lanes[before['readback_sumo_lane_id']]; ae,al = lanes[after['readback_sumo_lane_id']]
    require(be.get('function') == 'internal' and ae.get('function') != 'internal',
            'Only a single internal-to-normal boundary is allowed')
    require(any(c.get('from') == be.get('id') and c.get('to') == ae.get('id')
                and c.get('fromLane') == bl.get('index') and c.get('toLane') == al.get('index')
                for c in network.findall('connection')), 'Internal and normal lanes are not actually connected')
    bp,ap = shape(bl), shape(al)
    require(math.dist(bp[-1],ap[0]) <= 1e-6, 'Actual lane endpoints do not coincide')
    distance = math.dist(actor['readback_sumo_front_bumper_xy_m'],bp[-1])
    require(distance <= 0.1, 'Boundary discrepancy exceeds the explicit 0.1 m limit')
    for a,l in ((before,bl),(after,al)):
        params = {p.get('key'):p.get('value') for p in l.findall('param')}
        require(a['readback_lane_origId'] == params.get('origId') and
                point_line_distance(a['readback_sumo_front_bumper_xy_m'],shape(l)) <= float(l.get('width'))/2+0.1,
                'An adjacent actual lane/original ID or geometric membership is incorrect')
    require(proof.get('actual_returncode') == 0 and proof.get('sample') == sample
            and proof.get('source_frame') == mappings[sample]['source_frame']
            and proof.get('source_actor_id') == actor['carla_id']
            and proof.get('readback_xy') == actor['readback_sumo_front_bumper_xy_m']
            and proof.get('readback_yaw') == actor['readback_sumo_nav_angle_deg']
            and proof.get('readback_lane') == '' and proof.get('CARLA_connected_or_ticked') is False
            and proof.get('native_or_replay_modified') is False, 'Actual viewport reproduction proof differs')
    require(completely_outside(actor,proof['actual_after_step_boundary']),
            'The complete offnetwork vehicle footprint is visible in the actual native GUI viewport')
    return {'qualification':TYPE,'all_actor_lane_membership':False,'offnetwork_actor_samples':1,
            'source_sample':sample,'native_frame':mappings[sample]['source_frame'],
            'actor_role':actor['role'],'ego_all_lane_membership':True,
            'entire_offnetwork_actor_outside_actual_viewport':True,
            'connected_endpoint_distance_m':distance,'exact_pose_preserved':True,
            'lane_field_filled':False,'source_modified':False,
            'previous_sumo_lane':before['readback_sumo_lane_id'],
            'following_sumo_lane':after['readback_sumo_lane_id']}


def validate_evidence(replay, strict, scope, proof):
    strict_failure_gate(strict,scope)
    manifest = read(replay/'manifest.json'); mappings = rows(replay/'mapping.jsonl')
    require(sha(replay/'mapping.jsonl') == strict['input_sha256']['mapping']
            and sha(replay/'manifest.json') == strict['input_sha256']['replay_manifest'], 'Replay inputs changed')
    network = Path(manifest['network'])
    require(sha(network) == strict['input_sha256']['network'], 'Actual network changed')
    run = Path(manifest['source_run'])
    for key, path in (('raw_manifest',run/'manifest.json'),('raw_frames',run/'frames.jsonl'),('raw_xodr',run/'map.xodr')):
        require(sha(path) == strict['input_sha256'][key], 'Original source changed')
    require(sha(run.parent.parent/'executions'/(run.name+'.json')) == strict['input_sha256']['raw_execution'],
            'Actual source process receipt changed')
    investigation=network.parent.parent/'investigation.json'
    require(sha(investigation) == strict['input_sha256']['network_investigation'], 'Network conversion provenance changed')
    converter=Path(read(investigation)['official_converter'])
    require(sha(converter) == strict['input_sha256']['official_converter'] and
            sha(Path(manifest['official_bridge_helper'])) == strict['input_sha256']['official_bridge_helper'],
            'Official conversion or BridgeHelper source changed')
    require(proof.get('source_mapping_sha256') == strict['input_sha256']['mapping']
            and proof.get('native_GUI_settings_sha256') == proof.get('source_GUI_settings_sha256')
            == sha(replay/'gui-settings.xml'), 'Actual GUI reproduction settings or source mapping changed')
    row = mappings[scope['sample']]
    png = Path(row['native_sumo_png']); probe = Path(proof['probe_png'])
    require(Path(proof['native_source_png']).resolve() == png.resolve()
            and sha(png) == row['native_sumo_png_sha256'] == proof['native_source_png_sha256']
            == sha(probe) == proof['probe_png_sha256'], 'Probe/source PNG bytes are not actually identical')
    from PIL import Image
    with Image.open(png) as a, Image.open(probe) as b:
        a.load(); b.load()
        require(a.size == b.size == (1280,720) and a.mode == b.mode and a.tobytes() == b.tobytes(),
                'Actual full native viewport PNG pixels differ')
    return qualification_facts(mappings, ET.parse(network).getroot(),proof,scope)


def revalidate_report(report):
    require(report.get('audit_type') == TYPE and report.get('passed') is True
            and report.get('all_actor_lane_membership') is False
            and report.get('mode') == 'post_capture_synchronized_replay', 'Unknown or misleading qualified mode')
    paths = report['qualification_evidence']
    for key in ('original_strict_report','strict_recheck','scope','actual_viewport_proof','outer_replay_process'):
        require(sha(paths[key]) == paths[key+'_sha256'], 'Qualified evidence receipt changed')
    require(sha(__file__) == report['qualification_auditor_sha256'], 'Qualified auditor changed since verification')
    require(sha(Path(__file__).with_name('audit_sumo_sync_replay.py')) == report['input_sha256']['auditor'],
            'Original strict auditor changed')
    require(paths['outer_replay_process_sha256'] == report['input_sha256']['outer_replay_process_record'],
            'Outer SUMO process receipt differs from full strict recheck')
    original,recheck = read(paths['original_strict_report']),read(paths['strict_recheck'])
    for key in ('checks','failed_checks','failures_count','failures_examples','samples_checked',
                'actors_checked','native_pngs_fully_decoded','input_sha256','maximum_recomputed_errors'):
        require(original[key] == recheck[key], 'Strict original and independent full recheck disagree')
    facts = validate_evidence(Path(report['replay']),recheck,read(paths['scope']),read(paths['actual_viewport_proof']))
    require(facts == report['sumo_replay_qualification'] and report['input_sha256'] == recheck['input_sha256'],
            'Qualified public facts or immutable strict source bindings changed')
    return facts


def audit(replay, original, process, proof, scope, out):
    require(not out.exists(), 'Do not overwrite a prior audit')
    recheck = out.with_name(out.stem+'.strict-recheck.json')
    require(not recheck.exists(), 'Do not overwrite a prior strict recheck')
    sys.path.insert(0,str(Path(__file__).resolve().parent))
    from audit_sumo_sync_replay import audit as strict_audit
    strict_audit(replay,recheck,process)
    original_report,rechecked = read(original),read(recheck)
    for key in ('checks','failed_checks','failures_count','failures_examples','samples_checked',
                'actors_checked','native_pngs_fully_decoded','input_sha256','maximum_recomputed_errors'):
        require(original_report[key] == rechecked[key], 'Original strict FAIL no longer matches the actual full recheck')
    facts = validate_evidence(replay,rechecked,read(scope),read(proof))
    evidence = {}
    for k,p in (('original_strict_report',original),('strict_recheck',recheck),('scope',scope),('actual_viewport_proof',proof),('outer_replay_process',process)):
        evidence[k] = str(p.resolve()); evidence[k+'_sha256'] = sha(p)
    report = dict(rechecked)
    report.update(audit_type=TYPE,created_utc=datetime.now(timezone.utc).isoformat(),passed=True,
                  checks={'all_unaffected_strict_checks_pass':True,'exactly_one_pinned_background_endpoint':True,
                          'complete_actor_outside_actual_native_viewport':True,'original_strict_FAIL_preserved':True},
                  failed_checks=[],failures_count=0,failures_examples=[],all_actor_lane_membership=False,
                  original_strict_all_lane_audit_passed=False,sumo_replay_qualification=facts,
                  qualification_evidence=evidence,qualification_auditor_sha256=sha(__file__),
                  qualification_scope='One offnetwork background sample; all ego lanes and all exact source poses/times/images verified')
    report['limitations'].append('Original all-actor lane membership is false: one invisible background endpoint sample has an unchanged empty SUMO lane field. It is explicitly qualified, not repaired or claimed all-lane verified.')
    revalidate_report(report)
    out.parent.mkdir(parents=True,exist_ok=True)
    temp=out.with_name(out.name+'.tmp');temp.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');temp.replace(out)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('replay','original-strict-report','process-file','viewport-proof','scope','out'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    r=audit(a.replay.resolve(),a.original_strict_report.resolve(),a.process_file.resolve(),
            a.viewport_proof.resolve(),a.scope.resolve(),a.out.resolve())
    print(json.dumps({'passed':r['passed'],'all_actor_lane_membership':False,
                     'qualified_actor_samples':1,'samples_checked':r['samples_checked']}),flush=True)


if __name__ == '__main__':
    main()
