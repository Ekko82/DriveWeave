"""Read-only action audit of real CARLA frames and the independent exit record."""
from pathlib import Path
import argparse
import hashlib
import json
import math


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def location(vehicle):
    return vehicle['transform']['location_m']


def body_extent_along(vehicle, axis):
    matrix=vehicle['transform']['world_from_local']
    bbox=vehicle['bounding_box']
    center_local=bbox['center_local_m']
    center=[sum(matrix[row][col]*center_local[col] for col in range(3))+matrix[row][3] for row in range(3)]
    # Native model bboxes may have a local rotation, so combine it with the
    # actual actor transform before measuring front/rear support distances.
    roll,pitch,yaw=map(math.radians,bbox.get('rotation_rpy_deg',[0,0,0]))
    cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
    local=[[cp*cy,cy*sp*sr-sy*cr,-cy*sp*cr-sy*sr],
           [cp*sy,sy*sp*sr+cy*cr,-sy*sp*cr+cy*sr],
           [sp,-cp*sr,cp*cr]]
    world=[[sum(matrix[r][k]*local[k][c] for k in range(3)) for c in range(3)] for r in range(3)]
    extent=sum(abs(sum(axis[r]*world[r][c] for r in range(3)))*bbox['extent_m'][c] for c in range(3))
    projection=sum(center[r]*axis[r] for r in range(3))
    return projection-extent,projection+extent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    run=args.run.resolve()
    manifest=json.loads((run/'manifest.json').read_text())
    rows=[json.loads(line) for line in (run/'frames.jsonl').read_text().splitlines()]
    execution_path=run.parent.parent/'executions'/(run.name+'.json')
    execution=json.loads(execution_path.read_text()) if execution_path.exists() else {}
    checks={'actual_native_process_exit_zero':execution.get('returncode')==0,
        'execution_manifest_hash_matches':execution.get('manifest_sha256')==sha(run/'manifest.json'),
        'execution_frames_hash_matches':execution.get('frames_jsonl_sha256')==sha(run/'frames.jsonl'),
        'capture_manifest_complete':manifest.get('status')=='complete',
        'native_cleanup_complete':manifest.get('native_cleanup_complete') is True,
        'native_collision_zero':not json.loads((run/'collisions.json').read_text())}
    ego_rows=[next(v for v in row['vehicles'] if v['ego']) for row in rows]
    transitions=[]
    previous=None
    for row,ego in zip(rows,ego_rows):
        wp=ego.get('waypoint') or {}
        key=(wp.get('road_id'),wp.get('lane_id'))
        if key!=previous:
            transitions.append({'time_s':row['time_s'],'frame':row['frame'],'road_lane':list(key),'s_m':wp.get('s_m')})
            previous=key
    stages=manifest.get('route_specification',{}).get('stages',[])
    events=json.loads((run/'controller-events.json').read_text()) if (run/'controller-events.json').exists() else []
    completed=[e for e in events if e['event']=='stage_completed']
    checks['all_planned_stages_recorded_complete']=len(completed)==len(stages)>0
    if stages and stages[-1]['kind']=='follow_path':
        stage=stages[-1]
        if stage.get('path'):
            endpoint=stage['path'][-1]
        else:
            import carla
            road_map=carla.Map(manifest['map'],(run/'map.xodr').read_text())
            wp=road_map.get_waypoint_xodr(*stage['xodr_path'][-1])
            p=wp.transform.location
            endpoint=[p.x,p.y,p.z]
        checks['actual_final_reference_endpoint_reached']=any(math.dist(location(e)[:2],endpoint[:2])<2.5 for e in ego_rows)
    result={'source':'actual native CARLA frames; no server connection or replay',
        'run':str(run),'scenario':manifest.get('scenario'),'checks':checks,'road_lane_transitions':transitions,
        'completion_display_time_s':max((e['timestamp']-rows[0]['timestamp'] for e in completed),default=None),
        'final_video_validated':False,'sensor_processing_validated':False}
    if manifest.get('scenario')=='highway_lane_change_overtake_return':
        highway=[t['road_lane'][1] for t in transitions if t['road_lane'][0]==40]
        checks['actual_original_left_return_lane_order']=highway==[-3,-2,-3]
        paired=[]
        for row,ego in zip(rows,ego_rows):
            truck=next(v for v in row['vehicles'] if v['role_name']=='slow_lead')
            # Town04 road40 travels west. The actual ego forward axis is also
            # retained in the evidence; fixed west projection avoids rewarding
            # lateral steering rotations as longitudinal overtaking progress.
            eback,efront=body_extent_along(ego,[-1,0,0])
            tback,tfront=body_extent_along(truck,[-1,0,0])
            paired.append({'time_s':row['time_s'],'frame':row['frame'],
                'ego_lane':ego['waypoint']['lane_id'],'lead_initial_gap_m':tback-efront,
                'ego_rear_ahead_of_truck_front_m':eback-tfront})
        checks['actual_slow_lead_initially_ahead']=paired[0]['lead_initial_gap_m']>0
        passed=[p for p in paired if p['ego_lane']==-2 and p['ego_rear_ahead_of_truck_front_m']>0]
        checks['actual_vehicle_bodies_fully_passed_before_return']=bool(passed)
        first_left=next((t['time_s'] for t in transitions if t['road_lane']==[40,-2]),None)
        return_time=next((t['time_s'] for t in transitions if t['road_lane']==[40,-3] and first_left is not None and t['time_s']>first_left),None)
        returned=next((p for p in paired if return_time is not None and p['time_s']>=return_time and p['ego_lane']==-3),None)
        checks['actual_positive_return_gap']=bool(returned and returned['ego_rear_ahead_of_truck_front_m']>0)
        result['overtake_evidence']={'initial_lead_body_gap_m':paired[0]['lead_initial_gap_m'],
            'first_fully_passed_sample':passed[0] if passed else None,'first_return_sample':returned}
    elif 'left_turn' in str(manifest.get('scenario')) or 'roundabout' in str(manifest.get('scenario')):
        expected=manifest.get('route_specification',{}).get('expected_evidence',{})
        conflict_stage=next((s for s in stages if s.get('yield_gate')),None)
        gate=(conflict_stage or {}).get('yield_gate',{})
        polygon=gate.get('polygon_xy',[])
        def inside(point):
            value=False
            for i,a in enumerate(polygon):
                b=polygon[i-1]
                if (a[1]>point[1])!=(b[1]>point[1]) and point[0]<(b[0]-a[0])*(point[1]-a[1])/(b[1]-a[1])+a[0]:value=not value
            return value
        conflicts=[(r,e) for r,e in zip(rows,ego_rows) if r.get('rule_controller',{}).get('waiting')=='native_conflict_traffic']
        checks['actual_stopped_for_native_conflict']=any(e['speed_mps']<.5 for r,e in conflicts)
        role_names=gate.get('actor_roles') or expected.get('opposing_role_names') or expected.get('circulating_roles',[])
        ego_entry=next((r['time_s'] for r,e in zip(rows,ego_rows) if inside(location(e))),None)
        opponent_entries={}
        passed_roles=[]
        for role in role_names:
            times=[r['time_s'] for r in rows for v in r['vehicles'] if v['role_name']==role and inside(location(v))]
            before=[t for t in times if ego_entry is not None and t<ego_entry]
            occupied_at_entry=any(abs(t-ego_entry)<.001 for t in times) if ego_entry is not None else True
            if before and not occupied_at_entry:passed_roles.append(role)
            opponent_entries[role]={'first_time_s':min(times) if times else None,
                'last_time_s':max(times) if times else None,
                'last_occupancy_before_ego_entry_s':max(before) if before else None,
                'occupied_at_ego_entry':occupied_at_entry}
        checks['actual_conflict_vehicle_passed_before_ego']=bool(role_names and len(passed_roles)==len(role_names))
        result['yield_evidence']={'ego_first_inside_conflict_time_s':ego_entry,'opponent_occupancy':opponent_entries,
            'stopped_wait_samples':sum(e['speed_mps']<.5 for r,e in conflicts)}
        if 'left_turn' in str(manifest.get('scenario')):
            roads=[]
            for t in transitions:
                road=t['road_lane'][0]
                if not roads or road!=roads[-1]:roads.append(road)
            wanted=expected.get('ego_road_order',[])
            cursor=0
            for road in roads:
                if cursor<len(wanted) and road==wanted[cursor]:cursor+=1
            # Native get_waypoint chooses the nearest lane. Intersecting
            # junction connectors can briefly win that nearest-lane query.
            checks['actual_native_left_turn_road_order']=bool(wanted and cursor==len(wanted))
            delta=(ego_rows[-1]['transform']['rotation_deg'][1]-ego_rows[0]['transform']['rotation_deg'][1]+180)%360-180
            checks['actual_left_heading_rotation']=delta<-55
            checks['actual_unprotected_stop']=all(not r['ego_is_at_traffic_light'] for r in rows if r.get('rule_controller',{}).get('waiting')=='native_conflict_traffic')
            result['heading_change_deg']=delta
            exit_road=wanted[-1] if wanted else None
            exit_samples=[(r,e) for r,e in zip(rows,ego_rows) if e['waypoint']['road_id']==exit_road and e['speed_mps']>1]
            if exit_samples:
                exit_first=exit_samples[0][0]['time_s']
                exit_last=exit_samples[-1][0]['time_s']
                max_exit_s=max(e['waypoint']['s_m'] for r,e in exit_samples)
                body_cleared=max_exit_s>max(e['bounding_box']['extent_m'][0] for r,e in exit_samples)+3
            else:
                exit_first=exit_last=max_exit_s=None
                body_cleared=False
            result['left_turn_exit_evidence']={'first_moving_exit_time_s':exit_first,
                'last_moving_exit_time_s':exit_last,'maximum_actual_exit_s_m':max_exit_s,
                'body_fully_beyond_junction_exit_with_3m_margin':body_cleared,
                'at_least_2s_moving_in_exit':bool(exit_first is not None and exit_last-exit_first>=2)}
            core_names=['actual_native_process_exit_zero','execution_manifest_hash_matches',
                'execution_frames_hash_matches','capture_manifest_complete','native_cleanup_complete',
                'native_collision_zero','actual_stopped_for_native_conflict',
                'actual_conflict_vehicle_passed_before_ego','actual_native_left_turn_road_order',
                'actual_left_heading_rotation','actual_unprotected_stop']
            result['core_left_turn_completed']=all(checks[n] for n in core_names) and body_cleared and result['left_turn_exit_evidence']['at_least_2s_moving_in_exit']
            result['actual_opponent_road_sequences']={}
            for role in role_names:
                sequence=[]
                for r in rows:
                    actor=next((v for v in r['vehicles'] if v['role_name']==role),None)
                    if actor and (not sequence or actor['waypoint']['road_id']!=sequence[-1]):sequence.append(actor['waypoint']['road_id'])
                result['actual_opponent_road_sequences'][role]=sequence
        else:
            exit_pair=expected.get('exit_road_lane')
            checks['actual_native_east_exit']=any(t['road_lane']==exit_pair for t in transitions)
            center=expected.get('ring_center_xy_m',[-.5,.4])
            north=any(location(e)[1]>15 and abs(location(e)[0]-center[0])<20 for e in ego_rows)
            west=any(location(e)[0]<-16 and abs(location(e)[1]-center[1])<20 for e in ego_rows)
            checks['actual_west_and_north_circulation']=north and west
    result['action_audit_passed']=all(checks.values())
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({'report':str(args.out.resolve()),'passed':result['action_audit_passed'],'checks':checks},indent=2))
    return 0 if result['action_audit_passed'] else 2


if __name__=='__main__':
    raise SystemExit(main())
