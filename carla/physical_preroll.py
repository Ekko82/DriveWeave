"""Unrecorded physical driving before a hot-start capture; never set velocity.

The caller supplies the sole native/co-simulation tick and drains same-frame
sensor queues. All accepted preroll speeds are measured from CARLA physics.
"""
import json
import math


def rolling_preroll(world, ego, owner_tick, drain, apply_controls, run, plan,
                    settle_seconds, fps):
    minimum = float(plan.get('preroll_min_seconds', 3.0))
    maximum = float(plan.get('preroll_max_seconds', 12.0))
    threshold = float(plan.get('capture_start_speed_mps', 4.0))
    if not 0 < minimum <= maximum or threshold <= 0:
        raise ValueError('Invalid physical preroll thresholds')
    for _ in range(round(settle_seconds*fps)):
        frame=owner_tick()
        drain(frame)
    stable=0
    records=[]
    with (run/'physical-preroll.jsonl').open('w',encoding='utf-8') as handle:
        for index in range(round(maximum*fps)):
            diagnostics=apply_controls(world.get_snapshot())
            frame=owner_tick()
            drain(frame)
            snapshot=world.get_snapshot()
            if snapshot.frame != frame:
                raise RuntimeError('Preroll has a second world ticker')
            v=ego.get_velocity()
            speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
            record={'frame':frame,'timestamp':snapshot.timestamp.elapsed_seconds,
                'elapsed_rolling_s':(index+1)/fps,'actual_speed_mps':speed,
                'ego_location_m':[ego.get_location().x,ego.get_location().y,ego.get_location().z],
                'controls':diagnostics}
            records.append(record)
            handle.write(json.dumps(record,separators=(',',':'))+'\n')
            stable=stable+1 if speed>=threshold else 0
            if (index+1)/fps>=minimum and stable>=round(.5*fps):
                return {'source':'native physics driving with VehicleControl before capture',
                    'velocity_override':False,'settle_seconds':settle_seconds,
                    'rolling_seconds':(index+1)/fps,'measured_final_speed_mps':speed,
                    'required_start_speed_mps':threshold,'stable_duration_s':stable/fps,
                    'first_rolling_frame':records[0]['frame'],'last_rolling_frame':frame,
                    'log':'physical-preroll.jsonl'}
    raise RuntimeError(f'Physical preroll did not achieve {threshold} m/s within {maximum}s; '
                       f'last measured speed={records[-1]["actual_speed_mps"]:.3f}')
