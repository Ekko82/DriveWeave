"""Compare proposed native stop centers to recorded circulator boxes; no Client/tick.

Candidates are planning geometry, never altered capture poses or a passed run.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import argparse, copy, json, hashlib, math
import carla
from audit_native_actions import obb, obb_signed_clearance


def main():
    cli=argparse.ArgumentParser()
    cli.add_argument('--run', type=Path, required=True)
    cli.add_argument('--out', type=Path, required=True)
    args=cli.parse_args()
    frames=[json.loads(line) for line in (args.run/'frames.jsonl').read_text(encoding='utf-8').splitlines()]
    native=carla.Map('Town03',(args.run/'map.xodr').read_text(encoding='utf-8'))
    stop=native.get_waypoint_xodr(1577,-2,3.2)
    hero=next(v for v in frames[0]['vehicles'] if v['ego'])
    candidates=[]
    for back in (0,1,2,3,4,5):
        points=stop.previous(float(back)) if back else [stop]
        if len(points)!=1:raise RuntimeError('Ambiguous native stop predecessor')
        wp=points[0]
        pose=wp.transform
        planned=copy.deepcopy(hero)
        planned['transform']['location_m']=[pose.location.x,pose.location.y,float(hero['transform']['location_m'][2])]
        planned['transform']['rotation_deg']=[0,pose.rotation.yaw,0]
        pose.location.z=float(hero['transform']['location_m'][2])
        pose.rotation=carla.Rotation(yaw=pose.rotation.yaw)
        planned['transform']['world_from_local']=pose.get_matrix()
        box=obb(planned)
        minimum={'signed_clearance_m':float('inf')}
        for frame in frames:
            for vehicle in frame['vehicles']:
                if vehicle.get('role_name') not in ('circulating_0','circulating_1'):continue
                clearance=obb_signed_clearance(box,obb(vehicle))
                if clearance<minimum['signed_clearance_m']:
                    minimum={'signed_clearance_m':float(clearance),'frame':frame['frame'],'time_s':frame['time_s'],'role':vehicle['role_name'],'actual_actor_id':vehicle['id']}
        candidates.append({'upstream_distance_m':back,'native_waypoint':[wp.road_id,wp.lane_id,wp.s],
            'planned_root_center_m':planned['transform']['location_m'],'planned_native_yaw_deg':pose.rotation.yaw,
            'minimum_vs_recorded_circulators':minimum})
    report={'run':str(args.run.resolve()),'runtime_accepted':False,'server_connected':False,'world_ticked':False,
        'source_frames_sha256':hashlib.sha256((args.run/'frames.jsonl').read_bytes()).hexdigest(),
        'method':'Static native predecessor stop geometry with recorded actor boxes. Not a simulated stopping outcome or a corrected capture.',
        'candidates':candidates}
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
