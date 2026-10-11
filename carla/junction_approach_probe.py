"""Owner-only 20-second native approach diagnostic; never an accepted clip.

Load the selected plan's fresh native world, keep only its Tesla and first
stop stage, and run the same physical NativeRuleController. No sensors, NPCs,
state overrides, or modified physics. Only the runtime owner may execute it.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / 'verification'))
import argparse
import copy
import json
import math
import traceback
import carla
from native_controller import NativeRuleController, remaining_path_arc
from signal_policy import setup_signal_policy
from brake_physics_probe import sha, vector, control_record, physics_record, derivatives
from audit_motion_quality import load_thresholds


def parser():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--host',default='127.0.0.1')
    cli.add_argument('--port',type=int,default=2000)
    cli.add_argument('--plan',type=Path,required=True)
    cli.add_argument('--out',type=Path,required=True)
    cli.add_argument('--owned-fresh-world',action='store_true',required=True)
    cli.add_argument('--maximum-sim-seconds',type=float,default=20.)
    cli.add_argument('--thresholds',type=Path,default=Path(__file__).resolve().parent/'verification'/'motion-quality-thresholds-v2.json')
    return cli


def main():
    cli=parser();args=cli.parse_args()
    if not 1.<args.maximum_sim_seconds<=20.:
        cli.error('Diagnostic simulation cap must be >1 and <=20 seconds')
    if args.out.exists():cli.error('Use a fresh output directory; preserve old evidence')
    original_plan_bytes=args.plan.read_bytes()
    plan=json.loads(original_plan_bytes)
    if not plan.get('stages') or not plan['stages'][0].get('stop_at_end'):
        cli.error('Require the real first native stop_at_end approach stage')
    if plan.get('longitudinal_stop_policy',{}).get('enabled') is not True:
        cli.error('Require explicit calibrated opt-in native coasting stop')
    args.out.mkdir(parents=True)
    (args.out/'source-plan.json').write_bytes(original_plan_bytes)
    for name in ['junction_approach_probe.py','native_controller.py','signal_policy.py']:
        (args.out/('executed-'+name)).write_bytes(Path(__file__).with_name(name).read_bytes())
    trial_plan=copy.deepcopy(plan)
    trial_plan['stages']=trial_plan['stages'][:1]
    trial_plan['traffic']=[];trial_plan['dynamic_traffic']={'enabled':False}
    (args.out/'diagnostic-first-stage-plan.json').write_text(json.dumps(trial_plan,indent=2),encoding='utf-8')
    result={'probe_not_capture':True,'scene_accepted':False,'source_plan_sha256':sha(args.out/'source-plan.json'),
        'executed_script_sha256':sha(args.out/'executed-junction_approach_probe.py'),
        'controller_sha256':sha(args.out/'executed-native_controller.py'),
        'signal_policy_sha256':sha(args.out/'executed-signal_policy.py'),
        'sensor_count':0,'npc_count':0,'native_state_override':False,
        'maximum_sim_seconds':args.maximum_sim_seconds,'thresholds_sha256':sha(args.thresholds),
        'error':None,'cleanup_complete':False}
    rows=[];vehicle=None;world=None;original_settings=None;session=None;controller=None
    try:
        client=carla.Client(args.host,args.port);client.set_timeout(120.)
        result.update(client_version=client.get_client_version(),server_version=client.get_server_version())
        if result['client_version']!=result['server_version']:raise RuntimeError('Native client/server differ')
        world=client.load_world(plan['map'])
        original_settings=world.get_settings();settings=world.get_settings()
        settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
        settings.substepping=True;settings.max_substep_delta_time=.01;settings.max_substeps=10
        world.apply_settings(settings)
        road_map=world.get_map()
        (args.out/'map.xodr').write_bytes(road_map.to_opendrive().encode('utf-8'))
        result['map_sha256']=sha(args.out/'map.xodr')
        if plan.get('signal_policy',{}).get('enable') is True:
            world.tick()
            session=setup_signal_policy(world,plan['signal_policy'],carla_api=carla)
            stable=0
            for _ in range(20):
                frame=world.tick();snapshot=world.get_snapshot()
                if snapshot.frame!=frame:raise RuntimeError('Second native world ticker during signal settling')
                observed=session.settling_observe(snapshot)
                stable=stable+1 if observed['pass'] else 0
                if stable>=5:break
            if stable<5:raise RuntimeError('Required native signals did not settle for five ticks')
        start=road_map.get_waypoint_xodr(int(plan['ego_spawn']['road_id']),
            int(plan['ego_spawn']['lane_id']),float(plan['ego_spawn']['s_m']))
        if start is None or start.lane_type!=carla.LaneType.Driving:
            raise RuntimeError('Missing real native approach spawn')
        bp=world.get_blueprint_library().find('vehicle.tesla.model3')
        bp.set_attribute('role_name','junction_approach_physics_probe')
        pose=start.transform;pose.location.z+=.3
        vehicle=world.try_spawn_actor(bp,pose)
        if vehicle is None:raise RuntimeError('Native Tesla spawn failed')
        result.update(actor_id=vehicle.id,blueprint=vehicle.type_id,native_unmodified_physics=physics_record(vehicle))
        vehicle.apply_control(carla.VehicleControl(throttle=0.,brake=.35))
        world.tick();snapshot=world.get_snapshot()
        if session:session.observe(snapshot)
        if snapshot.find(vehicle.id) is None:raise RuntimeError('Native spawn snapshot missing')
        controller=NativeRuleController(world,vehicle,trial_plan,carla)
        path=controller.load_path(trial_plan['stages'][0])
        origin=snapshot.timestamp.elapsed_seconds
        completed_at=None;coast_start=None;stopped_at=None;rolling=None;prior_frame=snapshot.frame
        for _ in range(math.ceil(args.maximum_sim_seconds/.05)):
            prior=world.get_snapshot();actor=prior.find(vehicle.id)
            speed=math.sqrt(sum(v*v for v in vector(actor.get_velocity())))
            elapsed=prior.timestamp.elapsed_seconds-origin
            if rolling is None and elapsed>=3.-1e-7 and speed>=4.:
                rolling={'native_frame':prior.frame,'sim_time_s':prior.timestamp.elapsed_seconds,
                    'actual_speed_mps':speed,'stage_index':controller.index,
                    'stage0_incomplete':controller.index==0}
            requested=controller.step(prior)
            if controller.diagnostics.get('junction_coast_stop',{}).get('requested') and coast_start is None:
                coast_start=prior.timestamp.elapsed_seconds
                result['actual_coast_latch_snapshot']={'native_frame':prior.frame,
                    'sim_time_s':coast_start,'actual_speed_mps':speed,
                    'location_m':vector(actor.get_transform().location),
                    'diagnostics':copy.deepcopy(controller.diagnostics)}
            vehicle.apply_control(requested)
            frame=world.tick();snapshot=world.get_snapshot()
            if frame!=prior_frame+1 or snapshot.frame!=frame:raise RuntimeError('Unexpected native frame ticker')
            prior_frame=frame
            if session:session.observe(snapshot)
            actor=snapshot.find(vehicle.id)
            if actor is None:raise RuntimeError('Native Tesla snapshot missing')
            counts={'vehicle_count':len(world.get_actors().filter('vehicle.*')),
                    'sensor_count':len(world.get_actors().filter('sensor.*'))}
            if counts!={'vehicle_count':1,'sensor_count':0}:raise RuntimeError('Not a one-Tesla/no-sensor probe')
            transform=actor.get_transform();velocity=actor.get_velocity();acceleration=actor.get_acceleration()
            speed=math.sqrt(sum(v*v for v in vector(velocity)))
            forward=transform.get_forward_vector();wp=road_map.get_waypoint(transform.location)
            nearest=min(range(len(path)),key=lambda i:math.dist(vector(transform.location),path[i]))
            row={'sample':len(rows),'frame':frame,'sim_time_s':snapshot.timestamp.elapsed_seconds,
                'time_s':snapshot.timestamp.elapsed_seconds-origin,
                'location_m':vector(transform.location),'world_from_local':transform.get_matrix(),
                'velocity_mps':vector(velocity),'speed_mps':speed,
                'native_snapshot_acceleration_mps2':vector(acceleration),
                'native_acceleration_forward_mps2':sum(a*b for a,b in zip(vector(acceleration),vector(forward))),
                'requested_control':control_record(requested),'actual_control':control_record(vehicle.get_control()),
                'native_waypoint':{'road_id':wp.road_id,'lane_id':wp.lane_id,'s_m':wp.s},
                'native_lane_center_error_m':wp.transform.location.distance(transform.location),
                'remaining_path_arc_m':remaining_path_arc(vector(transform.location),path,nearest),
                'endpoint_distance_m':math.dist(vector(transform.location),path[-1]),
                'controller_diagnostics':copy.deepcopy(controller.diagnostics),**counts}
            rows.append(row)
            if speed<=.1 and coast_start is not None and stopped_at is None:
                stopped_at=snapshot.timestamp.elapsed_seconds
                result['actual_stop_snapshot']=copy.deepcopy(row)
            if controller.index>=len(controller.stages) and completed_at is None:completed_at=snapshot.timestamp.elapsed_seconds
            if completed_at is not None and stopped_at is not None and snapshot.timestamp.elapsed_seconds-stopped_at>=.6:
                result['termination_reason']='actual_stage_completed_and_real_stop_held_0p6s';break
        else:result['termination_reason']='20_second_native_simulation_limit'
        result.update(samples=len(rows),actual_rolling_preroll_candidate=rolling,
            actual_stop_observed=stopped_at is not None,stage0_completed=controller.index>=len(controller.stages),
            controller_events=controller.events)
        if len(rows)>=5:result['actual_derivative_metrics']=derivatives(rows,load_thresholds(args.thresholds),coast_start)
        metrics=result.get('actual_derivative_metrics',{}).get('braking_with_actual_transition',{})
        result['actual_motion_targets_met']=bool(metrics.get('acceleration_target_met') and metrics.get('jerk_target_met'))
        result['actual_native_probe_targets_met']=bool(controller.index>=len(controller.stages) and stopped_at is not None
            and result['actual_motion_targets_met'] and result['termination_reason']=='actual_stage_completed_and_real_stop_held_0p6s')
    except Exception:
        result['error']=traceback.format_exc()
    finally:
        errors=[]
        if vehicle is not None:
            try:
                if not vehicle.destroy():errors.append('Native Tesla destroy returned false')
            except Exception:errors.append(traceback.format_exc())
        if session is not None:
            try:result['signal_restoration']=session.restore()
            except Exception:errors.append(traceback.format_exc())
            result['signal_evidence']=session.evidence
        if world is not None and original_settings is not None:
            try:world.apply_settings(original_settings)
            except Exception:errors.append(traceback.format_exc())
        result.update(cleanup_errors=errors,cleanup_complete=not errors)
        (args.out/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows),encoding='utf-8')
        result['frames_sha256']=sha(args.out/'frames.jsonl')
        result['strict_probe_pass']=bool(result.get('actual_native_probe_targets_met') and result['cleanup_complete'] and result['error'] is None)
        (args.out/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps({'out':str(args.out),'strict_probe_pass':result['strict_probe_pass'],
            'actual_stop':result.get('actual_stop_snapshot',{}).get('endpoint_distance_m'),
            'stage_completed':result.get('stage0_completed'),'error':result['error']},ensure_ascii=False),flush=True)
    raise SystemExit(0 if result['strict_probe_pass'] else 1)


if __name__=='__main__':main()
