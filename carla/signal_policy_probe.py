"""Owner-run native signal diagnostic. No vehicles/sensors or accepted capture.

This script deliberately loads fresh Town05 worlds and owns their ticks. Only
the root runtime owner may run it when no other simulation is in progress.
Settling mismatches stay in evidence; they never become accepted media frames.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import argparse
import hashlib
import json
import time
import traceback
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import carla
from signal_policy import setup_signal_policy, town05_wide_left_signal_config


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot_rows(session, snapshot):
    rows=[]
    for selected in session._selections:
        light=selected['actor']
        state=light.get_state()
        durations=[light.get_green_time(),light.get_yellow_time(),light.get_red_time()]
        required_frozen=session._freeze_world or session._originals[int(light.id)]['is_frozen']
        rows.append({'actor_id':light.id,'native_state':str(state),
            'desired_state':selected['desired_state'],'approach_roles':selected['roles'],
            'actor_state':str(light.actor_state),'is_active':bool(light.is_active),
            'is_dormant':bool(light.is_dormant),'is_frozen':bool(light.is_frozen()),
            'green_yellow_red_s':durations,
            'elapsed_s':light.get_elapsed_time(),
            'duration_hold_matches':all(abs(float(t)-selected['duration_hold_s'])<=.05 for t in durations),
            'freeze_state_matches':bool(light.is_frozen())==bool(required_frozen),
            'state_matches':state==getattr(carla.TrafficLightState,selected['desired_state'])})
    return {'native_frame':snapshot.frame,'sim_time_s':snapshot.timestamp.elapsed_seconds,
        'observed_monotonic_s':time.monotonic(),'lights':rows,
        'desired_states_all_match':all(q['state_matches'] for q in rows),
        'all_selected_native_actors_active':all(q['is_active'] and not q['is_dormant'] for q in rows),
        'all_selected_duration_holds_match':all(q['duration_hold_matches'] for q in rows),
        'all_selected_freeze_states_match':all(q['freeze_state_matches'] for q in rows)}


def one_mode(client, mode, outdir, args):
    world=client.load_world('Town05')
    settings=world.get_settings()
    original_settings=world.get_settings()
    settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
    settings.substepping=True;settings.max_substep_delta_time=.01;settings.max_substeps=10
    world.apply_settings(settings)
    session=None
    result={'mode':mode,'map':world.get_map().name,'native_probe_not_capture':True,
        'has_vehicles_or_sensor_outputs':False,'owner_only_tick':True,
        'setup_ticked_native_frames':[],'observations':[],'mutation_events':[],
        'restore_post_tick':None,'error':None,'recorded_capture_accepted':False}
    try:
        setup_frame=world.tick()
        result['setup_ticked_native_frames'].append(setup_frame)
        config=town05_wide_left_signal_config()
        frozen=mode in ('global_frozen_sync_barrier','sequential_states_frozen')
        config.update(freeze_world_lights=frozen,owned_fresh_world=frozen)
        session=setup_signal_policy(world,config,carla_api=carla)
        result['selected_originals']=session.evidence['originals']
        result['native_signal_geometry']=session.evidence['native_signal_geometry']
        result['native_map_xodr_sha256']=session.evidence['native_map_xodr_sha256']
        result['requested_duration_holds_s']=session.evidence['duration_holds_s']
        result['global_freeze_called']=session.evidence['global_freeze_called']
        if mode!='long_phase_no_barrier':
            # Official CallAndWait RPC; this is a diagnostic ordering comparison,
            # not a promise that arbitrary async commands have completed.
            result['sync_group_barriers']=[{'actor_id':q['actor'].id,
                'group_actor_ids':[p.id for p in q['actor'].get_group_traffic_lights()]} for q in session._selections]
        else:result['sync_group_barriers']=[]
        for i in range(args.settle_ticks+args.verify_ticks):
            frame=world.tick();snapshot=world.get_snapshot()
            if snapshot.frame!=frame:raise RuntimeError('Second native world ticker detected')
            row=snapshot_rows(session,snapshot)
            row.update(phase='settle' if i<args.settle_ticks else 'verify',phase_index=i if i<args.settle_ticks else i-args.settle_ticks)
            result['observations'].append(row)
        verification=result['observations'][args.settle_ticks:]
        result['all_verification_native_states_match']=bool(verification) and all(
            q['desired_states_all_match'] and q['all_selected_native_actors_active'] and
            q['all_selected_duration_holds_match'] and q['all_selected_freeze_states_match'] for q in verification)
        if mode=='sequential_states_frozen':
            greens=[q for q in session._selections if q['desired_state']=='Green']
            if len(greens)!=2:raise RuntimeError('Sequential probe needs exactly two verified opposing green actors')
            commands=[(greens[0],carla.TrafficLightState.Red),(greens[1],carla.TrafficLightState.Red),
                (greens[0],carla.TrafficLightState.Green),(greens[1],carla.TrafficLightState.Green),
                (greens[0],carla.TrafficLightState.Green)]
            for selected,state in commands:
                selected['actor'].set_state(state)
                group=[p.id for p in selected['actor'].get_group_traffic_lights()]
                event={'actor_id':selected['actor'].id,'requested_state':str(state),
                    'native_tick_before_request':world.get_snapshot().frame,'sync_group_actor_ids':group,'actual_post_tick_rows':[]}
                for _ in range(args.settle_ticks):
                    frame=world.tick();snapshot=world.get_snapshot()
                    if snapshot.frame!=frame:raise RuntimeError('Second native world ticker detected')
                    event['actual_post_tick_rows'].append(snapshot_rows(session,snapshot))
                result['mutation_events'].append(event)
    except Exception as exc:
        result['error']=repr(exc);result['traceback']=traceback.format_exc()
    finally:
        if session is not None:
            try:
                result['restoration_requests']=session.restore()
                # Verify restoration with a fresh native snapshot while we still
                # own this disposable episode, rather than claiming RPC success.
                world.tick();result['restore_post_tick']=snapshot_rows(session,world.get_snapshot())
                result['restored_original_native_states_match']=all(
                    q['native_state']==session.evidence['originals'][str(q['actor_id'])]['state_name']
                    for q in result['restore_post_tick']['lights'])
            except Exception as exc:
                result['restore_error']=repr(exc)
        try:world.apply_settings(original_settings)
        except Exception as exc:result['world_settings_restore_error']=repr(exc)
        path=outdir/(mode+'.json')
        path.write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps({'mode':mode,'path':str(path),'actual_native_states_match':result.get('all_verification_native_states_match',False),'error':result['error']},ensure_ascii=False),flush=True)
    return result


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--host',default='127.0.0.1');cli.add_argument('--port',type=int,default=2000)
    cli.add_argument('--out',type=Path,required=True)
    cli.add_argument('--owned-fresh-world',action='store_true',required=True)
    cli.add_argument('--settle-ticks',type=int,default=5);cli.add_argument('--verify-ticks',type=int,default=100)
    cli.add_argument('--mode',action='append',choices=['long_phase_no_barrier','long_phase_sync_barrier','global_frozen_sync_barrier','sequential_states_frozen'])
    args=cli.parse_args()
    if args.settle_ticks<1 or args.verify_ticks<1:cli.error('Require positive actual settle and verification ticks')
    if args.out.exists():cli.error('Use a fresh output directory; existing probe evidence is immutable')
    args.out.mkdir(parents=True)
    client=carla.Client(args.host,args.port);client.set_timeout(120)
    metadata={'native_diagnostic_probe_not_capture':True,'client_version':client.get_client_version(),
        'server_version':client.get_server_version(),'script_sha256':sha(Path(__file__)),
        'signal_policy_sha256':sha(Path(__file__).with_name('signal_policy.py')),
        'settle_ticks':args.settle_ticks,'verification_ticks':args.verify_ticks,
        'vehicle_count':0,'sensor_count':0,'capture_acceptance':False,
        'modes':args.mode or ['long_phase_no_barrier','long_phase_sync_barrier','global_frozen_sync_barrier','sequential_states_frozen']}
    if metadata['client_version']!=metadata['server_version']:raise RuntimeError('Native client/server versions differ')
    (args.out/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    results=[one_mode(client,mode,args.out,args) for mode in metadata['modes']]
    summary={'metadata':metadata,'modes':[{k:r.get(k) for k in ['mode','all_verification_native_states_match','error','restored_original_native_states_match']} for r in results],
        'probe_demonstrates_capture_success':False}
    (args.out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
