"""Offline supplementary native swept-body conflict audit; retains broad-gate FAIL.

The report refines only a second-entry gate that spans multiple native lanes.
It cannot waive any other original audit failure or physical clearance limit.
"""
from pathlib import Path
import argparse
import itertools
import json
import math
import sys

ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT), str(ROOT/'verification')]
import carla
import numpy as np
from audit_native_actions import obb, obb_signed_clearance, file_sha
from audit_junction_interaction import convex_hull, overlap, reference, project, runs
from audit_v3_junction import native_reference_checked


def area(poly):
    p = np.asarray(poly, float)
    return float(abs(np.sum(p[:, 0]*np.roll(p[:, 1], -1)-p[:, 1]*np.roll(p[:, 0], -1)))/2) if len(p)>=3 else 0.


def intersection(subject, clip):
    """Convex Sutherland-Hodgman intersection, including exact boundary vertices."""
    result = [np.asarray(p, float) for p in subject]
    boundary = list(clip)
    signed = sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(boundary, boundary[1:]+boundary[:1]))
    if signed < 0:
        boundary.reverse()
    def cross(a,b):
        return a[0]*b[1]-a[1]*b[0]
    for aa,bb in zip(boundary, boundary[1:]+boundary[:1]):
        if not result:
            break
        a,b=np.asarray(aa,float),np.asarray(bb,float)
        source,result=result,[]
        for start,end in zip(source[-1:]+source[:-1], source):
            ds,de=cross(b-a,start-a),cross(b-a,end-a)
            ins,ine=ds>=-1e-9,de>=-1e-9
            if ins != ine:
                fraction=ds/(ds-de)
                result.append(start+fraction*(end-start))
            if ine:
                result.append(end)
    return [p.tolist() for p in result]


def rectangle(point, direction, half_length, half_width):
    tangent=np.asarray(direction,float);tangent/=np.linalg.norm(tangent)
    normal=np.array([-tangent[1],tangent[0]])
    return convex_hull([np.asarray(point,float)+sx*half_length*tangent+sy*half_width*normal
                        for sx,sy in itertools.product([-1,1],repeat=2)])


def corridor(points, half_length, half_width, gate):
    cells=[];seen=set()
    for start,end in zip(points,points[1:]):
        start,end=np.asarray(start)[:2],np.asarray(end)[:2]
        direction=end-start
        if np.linalg.norm(direction)<1e-6:
            continue
        swept=convex_hull(rectangle(start,direction,half_length,half_width)+rectangle(end,direction,half_length,half_width))
        clipped=intersection(swept,gate)
        key=tuple(sorted(tuple(round(x,5) for x in p) for p in clipped))
        if area(clipped)>1e-8 and key not in seen:
            cells.append(clipped);seen.add(key)
    return cells


def conflict_patches(a_cells,b_cells):
    patches=[]
    for a in a_cells:
        for b in b_cells:
            if overlap(a,b):
                poly=intersection(a,b)
                if area(poly)>1e-8:
                    patches.append(poly)
    return patches


def simultaneous_patch_occupation(a_body,b_body,patches):
    return any(overlap(a_body,patch) and overlap(b_body,patch) for patch in patches)


def measured_envelope(trace, path):
    """Width covers actual body corners relative to the native reference, not lane IDs."""
    width=0.;length=0.
    for vehicle in trace:
        position=vehicle['transform']['location_m'];yaw=vehicle['transform']['rotation_deg'][1]
        fit=project(position,yaw,path)
        a,b=np.asarray(fit['native_segment_xyz_m'])[:,:2]
        tangent=(b-a)/np.linalg.norm(b-a);normal=np.array([-tangent[1],tangent[0]])
        closest=a+fit['segment_fraction']*(b-a)
        corners=obb(vehicle)['vertices'][:,:2]
        width=max(width,float(np.max(np.abs((corners-closest)@normal))))
        length=max(length,float(np.max(np.abs((corners-np.asarray(position)[:2])@tangent))))
    # 1 cm numerical buffer; not an acceptance threshold reduction.
    return length+.01,width+.01


def audit(run, strict_path, out):
    strict=json.loads(strict_path.read_text())
    rows=[json.loads(x) for x in (run/'frames.jsonl').read_text().splitlines()]
    manifest=json.loads((run/'manifest.json').read_text())
    plan=manifest['route_specification'];native_map=carla.Map(plan['map'],(run/'map.xodr').read_text())
    role=plan['expected_evidence']['second_entry_role']
    second=next(s for s in plan['traffic'] if s['name']==role)
    gate=next(s['yield_gate'] for s in second['stages'] if 'yield_gate' in s)
    specs={'hero':plan,**{s['name']:s for s in plan['traffic']}}
    by_role={name:[next(v for v in r['vehicles'] if v['role_name']==name) for r in rows]
             for name in [role]+gate['actor_roles']}
    paths={name:native_reference_checked(native_map,specs[name]['stages']) for name in by_role}
    # The native_reference_checked return is already the actual CARLA reference.
    occupancy=[overlap(convex_hull(obb(v)['vertices'][:,:2]),gate['polygon_xy']) for v in by_role[role]]
    episodes=runs(list(range(len(rows))),lambda i:occupancy[i])
    first=episodes[0] if episodes else []
    if not first:
        raise ValueError('No actual second entry body pass')
    last=first[-1]
    current=by_role[role][last]
    fit=project(current['transform']['location_m'],current['transform']['rotation_deg'][1],paths[role])
    # Preserve the first native entry and several metres after it; exclude a later lap.
    second_path=paths[role][:fit['segment_index']+10]
    env={name:measured_envelope(trace,paths[name]) for name,trace in by_role.items()}
    cells={name:corridor(second_path if name==role else paths[name],*env[name],gate['polygon_xy']) for name in by_role}
    results={}
    for other in gate['actor_roles']:
        patches=conflict_patches(cells[role],cells[other])
        broad=[];precise=[];minimum={'clearance_m':float('inf')}
        for i in range(last+1):
            a,b=by_role[role][i],by_role[other][i];aa,bb=obb(a),obb(b)
            bodya,bodyb=convex_hull(aa['vertices'][:,:2]),convex_hull(bb['vertices'][:,:2])
            if not occupancy[i]:
                continue
            gap=obb_signed_clearance(aa,bb)
            sample={'frame':rows[i]['frame'],'time_s':rows[i]['timestamp']-rows[0]['timestamp'],
                    'clearance_m':gap,'second_waypoint':a.get('waypoint'),'other_waypoint':b.get('waypoint'),
                    'second_xyz':a['transform']['location_m'],'other_xyz':b['transform']['location_m']}
            if gap<minimum['clearance_m']:
                minimum=sample
            if overlap(bodyb,gate['polygon_xy']):
                broad.append(sample)
            if simultaneous_patch_occupation(bodya,bodyb,patches):
                precise.append(sample)
        results[other]={'native_swept_body_conflict_patch_count':len(patches),
                       'conflict_patch_total_area_with_overlapping_cells_m2':sum(area(p) for p in patches),
                       'broad_gate_simultaneous_samples':len(broad),
                       'precise_conflict_simultaneous_samples':len(precise),
                       'precise_conflict_samples':precise,'minimum_entry_phase_obb':minimum,
                       'broad_gate_first_sample':broad[0] if broad else None,
                       'broad_gate_last_sample':broad[-1] if broad else None}
    allowed_failure=role+'_actual_second_native_entry'
    detail=strict['evidence'][allowed_failure]
    physical_entry=detail['tracking']['passed'] and detail['first_road10_time_s'] is not None and 30 in detail['native_road_order'] and 1772 in detail['native_road_order']
    checks={'strict_evidence_bound_to_unchanged_capture':all(file_sha(run/k)==v for k,v in strict['input_sha256'].items()),
            'only_original_wide_gate_failure':strict['failed_checks']==[allowed_failure],
            'original_all_other_source_motion_path_and_clearance_checks_pass':all(v for k,v in strict['checks'].items() if k!=allowed_failure),
            'actual_second_native_entry_and_tracking_pass':physical_entry,
            'no_simultaneous_native_swept_body_conflict':all(v['precise_conflict_simultaneous_samples']==0 for v in results.values())}
    report={'schema':'v3_native_entry_corridor_supplement_v1','supplementary_precise_geometry_passed':all(checks.values()),
            'strict_broad_gate_report_remains_failed':not strict['scenario_actions_validated'],
            'strict_broad_gate_report':str(strict_path.resolve()),'strict_report_sha256':file_sha(strict_path),
            'run':str(run.resolve()),'source_sha256':strict['input_sha256'],'auditor_sha256':file_sha(Path(__file__)),
            'checks':checks,'failed_checks':[k for k,v in checks.items() if not v],
            'second_entry_role':role,'gate_polygon_xy':gate['polygon_xy'],
            'actual_first_entry_sample_interval':[first[0],last],
            'native_reference_measured_body_envelopes_m':{k:{'half_length':v[0],'half_width':v[1]} for k,v in env.items()},
            'pair_reviews':results,
            'method':['Resolve each actual route through CARLA native Driving waypoints and native successor topology.',
                      'Construct convex swept-body cells along native physical paths; include each measured full body lateral envelope and 1cm numerical buffer.',
                      'Intersect each pair of native swept-body corridors inside the original broad gate; never classify conflicts using lane ID equality.',
                      'Use actual full recorded body polygons at every frame of the first entry to reject simultaneous occupation of any common conflict patch.',
                      'Keep original all-frame 3D OBB clearance, motion, source and route requirements mandatory.'],
            'limitations':['Finite native path segments and20Hz observed snapshots do not prove arbitrary continuous-time safety.',
                           'This is a supplementary precise geometry decision; it does not turn the retained broad-gate FAIL into a strict PASS.',
                           'Actual observed envelopes apply only to this frozen capture, not unseen executions.'],
            'source_modified':False,'server_connected_or_ticked':False}
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x',encoding='utf-8') as f:
        json.dump(report,f,indent=2)
    print(json.dumps({'out':str(out),'passed':report['supplementary_precise_geometry_passed'],'failed_checks':report['failed_checks'],
                      'pairs':{k:{x:v[x] for x in ['native_swept_body_conflict_patch_count','broad_gate_simultaneous_samples','precise_conflict_simultaneous_samples']} for k,v in results.items()}}))
    return report['supplementary_precise_geometry_passed']


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--strict-report',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    raise SystemExit(0 if audit(args.run,args.strict_report,args.out) else 1)
