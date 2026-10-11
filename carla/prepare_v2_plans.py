"""Prepare denser native-road versions; no simulator client or world tick."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent


def main():
    sources=['town04-highway','town06-ramp-entry','town06-ramp-exit']
    for name in sources:
        plan=json.loads((ROOT/'plans'/'base'/f'{name}.json').read_text(encoding='utf-8'))
        plan.update(hot_start=True,preroll_min_seconds=3.,preroll_max_seconds=12.,
                    capture_start_speed_mps=4.,version='v2',finish_capture_on_plan_complete=True)
        plan['source_inventory']=f'map-inventory/{plan["map"]}/map.xodr'
        if name=='town04-highway':
            plan['ego_spawn']['s_m']=.5
            plan['ego_spawn'].pop('native_position_m',None)
            plan['stages'][0]['xodr_path']=[[40,-3,.5]]+[[40,-3,float(s)] for s in range(1,36)]
            plan['dynamic_traffic']={'count':24,'seed':17,
                'corridors':[{'road_id':40,'lane_ids':[-4,-1,3,4,5,6],
                    's_start':8,'s_end':228,'spacing_m':10}],
                'exclude_xodr':[{'road_id':40,'lane_ids':[-3,-2],'s_start':0,'s_end':230}],
                'speed_range_kmh':[40,62],'time_headway_range_s':[1.4,1.9],
                'require_count':True}
        else:
            entry=name.endswith('entry')
            if not entry:
                plan['ego_spawn']['s_m']=85.
                plan['ego_spawn'].pop('native_position_m',None)
                plan['stages'][0]['xodr_path']=[[37,-4,85.+.7*i] for i in range(79)]+[[37,-4,140.]]
            plan['dynamic_traffic']={'count':24,'seed':29 if entry else 31,
                'corridors':[{'road_id':37,'lane_ids':[-7,-6,-5,-4,-3],
                    's_start':8,'s_end':195,'spacing_m':10},
                    {'road_id':36,'lane_ids':[-7,-6,-5,-4,-3],
                    's_start':4,'s_end':52,'spacing_m':8}],
                'exclude_xodr':[{'road_id':37,'lane_ids':[-7,-6] if entry else [-4,-3],
                    's_start':0,'s_end':198}],
                'speed_range_kmh':[40,60],'time_headway_range_s':[1.4,1.9],
                'require_count':True}
        traffic=plan['dynamic_traffic']
        traffic.update(ego_lane_change_protection_m=65.,lane_change_cooldown_s=8.,
            lane_change_exclude_xodr=traffic['exclude_xodr'],lane_change_probability_range_pct=[.02,.05])
        if 'highway' in name:
            plan['traffic']=[v for v in plan['traffic'] if v['name']!='opposite_flow']
        else:
            traffic['count']=18
        plan['expected_evidence']['minimum_car_body_clearance_m']=.75
        out=ROOT/'plans'/f'v2-{name}.json'
        out.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(out.name)


if __name__=='__main__':
    main()
