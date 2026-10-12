"""Report measured local traffic and interaction from unchanged native snapshots."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent/'verification'))
import numpy as np
from audit_motion_quality import local_polynomial_derivatives
from audit_native_actions import obb, obb_signed_clearance


def evaluate(run, out):
    rows = [json.loads(x) for x in (run/'frames.jsonl').read_text().splitlines()]
    records, minima = [], {'all':None, 'ego':None}
    previous_frame = None
    for row in rows:
        ego = next(v for v in row['vehicles'] if v['ego'])
        ex, ey, ez = ego['transform']['location_m']
        yaw = math.radians(ego['transform']['rotation_deg'][1])
        rear, same, front, local = [], [], [], []
        boxes = {v['id']:obb(v) for v in row['vehicles']}
        for v in row['vehicles']:
            if v['ego']: continue
            x,y,z = v['transform']['location_m']
            dx,dy = x-ex,y-ey
            along, lateral = dx*math.cos(yaw)+dy*math.sin(yaw), -dx*math.sin(yaw)+dy*math.cos(yaw)
            same_direction = abs((v['transform']['rotation_deg'][1]-math.degrees(yaw)+180)%360-180) <= 45
            if abs(z-ez)<=3 and math.hypot(dx,dy)<=80 and same_direction: local.append(v['role_name'])
            if abs(z-ez)<=3 and same_direction and -60<=along<=-5 and abs(lateral)<=7:
                rear.append(v['role_name'])
                if abs(lateral)<=1.75: same.append(v['role_name'])
            if abs(z-ez)<=3 and same_direction and 5<=along<=60 and abs(lateral)<=1.75: front.append(v['role_name'])
        for i,a in enumerate(row['vehicles']):
            for b in row['vehicles'][i+1:]:
                if math.dist(a['transform']['location_m'],b['transform']['location_m']) > 16: continue
                clearance = float(obb_signed_clearance(boxes[a['id']],boxes[b['id']]))
                item={'clearance_m':clearance,'time_s':row['time_s'],'frame':row['frame'],'roles':[a['role_name'],b['role_name']]}
                for key in ['all']+(['ego'] if a['ego'] or b['ego'] else []):
                    if minima[key] is None or clearance < minima[key]['clearance_m']: minima[key]=item
        d = row['rule_controller']
        records.append({'time_s':row['time_s'],'frame':row['frame'],'speed_kmh':ego['speed_mps']*3.6,
            'rear60':rear,'same_width_rear60':same,'front60':front,'local80':len(local),
            'stage':d.get('stage',d.get('waiting')),'stage_index':d.get('stage_index'),
            'gap_waiting':bool(d.get('gap_waiting')),'emergency':bool(d.get('emergency_braking')),
            'ready':row.get('traffic_readiness',{}).get('ready')})
    def summary(items):
        speed = np.asarray([r['speed_kmh'] for r in items])
        return {'frames':len(items),'speed_kmh':{'min':float(speed.min()),'mean':float(speed.mean()),'max':float(speed.max())},
            **{key+'_pct':100*sum(bool(r[key]) for r in items)/len(items) for key in ['rear60','same_width_rear60','front60']},
            'local80_mean':float(np.mean([r['local80'] for r in items])),
            'gap_wait_frames':sum(r['gap_waiting'] for r in items),'emergency_frames':sum(r['emergency'] for r in items),
            'stages':dict(collections.Counter(r['stage'] for r in items))}
    speed = [next(v['speed_mps'] for v in r['vehicles'] if v['ego']) for r in rows]
    accel,jerk=local_polynomial_derivatives(np.array([r['timestamp'] for r in rows]),np.array(speed),.5)
    result={'run':str(run.resolve()),'frames_sha256':hashlib.sha256((run/'frames.jsonl').read_bytes()).hexdigest(),
        'source_changed':False,'all':summary(records),'after_preroll_10s':summary([r for r in records if r['time_s']>=10]) if records[-1]['time_s']>=10 else None,
        'minimum_native_obb_clearance':minima,'windowed_actual_accel_min':float(min(accel)),'windowed_actual_accel_max':float(max(accel)),
        'windowed_actual_abs_jerk_max':float(max(abs(jerk))),'windowed_actual_abs_jerk_p95':float(np.quantile(abs(jerk),.95)),
        'definition':'Same-direction native center positions: rear longitudinal [-60,-5]m, lateral <=7m, same-width <=1.75m, heading <=45deg. Curved lane geometry may undercount; no camera visibility claim.',
        'accepted':False,'acceptance_reason':'Diagnostic report only; full sensor, action, native motion and final visual checks remain required.'}
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    out.with_suffix('.series.json').write_text(json.dumps(records),encoding='utf-8')
    print(json.dumps(result),flush=True)


if __name__ == '__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--run',type=Path,required=True)
    cli.add_argument('--out',type=Path,required=True)
    args=cli.parse_args()
    evaluate(args.run,args.out)
