"""Owner-only physical traffic trials. Logs native snapshots; no camera media."""
import argparse
import datetime
import faulthandler
import gc
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback
sys.path.insert(0, str(Path(__file__).resolve().parent))
import carla
from capture_native import serialize_transform, waypoint_record, CallbackBarrier
from native_controller import NativeRuleController, spawn_from_xodr
from signal_policy import setup_signal_policy
from dynamic_traffic import setup_dynamic_traffic


def main():
    faulthandler.enable(all_threads=True)
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--plan', type=Path, required=True)
    cli.add_argument('--out', type=Path, required=True)
    cli.add_argument('--seconds', type=float, default=55)
    cli.add_argument('--seed', type=int, default=17)
    cli.add_argument('--owned-fresh-world', action='store_true', required=True)
    args = cli.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    plan = json.loads(args.plan.read_text(encoding='utf-8'))
    (args.out/'plan.json').write_bytes(args.plan.read_bytes())
    source_hashes = {}
    for name in ['v3_physics_probe.py', 'native_controller.py', 'dynamic_traffic.py', 'signal_policy.py', 'v3_traffic.py']:
        source = Path(__file__).with_name(name)
        if source.exists():
            payload = source.read_bytes()
            (args.out/('executed-'+name)).write_bytes(payload)
            source_hashes[name] = hashlib.sha256(payload).hexdigest()
    v3 = any(s.get('controller') == 'v3_traffic' for s in plan.get('traffic', []))
    if v3:
        from v3_traffic import V3TrafficController, traffic_ready
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(60)
    world = client.load_world(plan['map'])
    previous = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = .05
    settings.substepping = True
    settings.max_substep_delta_time = .01
    settings.max_substeps = 10
    settings.no_rendering_mode = True
    world.apply_settings(settings)
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    tm.set_random_device_seed(args.seed)
    road_map = world.get_map()
    (args.out/'map.xodr').write_text(road_map.to_opendrive(), encoding='utf-8')
    vehicles, controllers, sensors, collisions = [], {}, [], []
    barrier, callbacks = CallbackBarrier(), []
    signal = None
    traffic_session = None
    result = {'diagnostic_only': True, 'accepted_capture': False, 'error': None,
        'native_state_override': False, 'camera_count': 0, 'source_sha256':source_hashes,
        'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat()}
    try:
        specs = [dict(plan, name='hero', blueprint='vehicle.tesla.model3', spawn_xodr=plan['ego_spawn'])] + plan.get('traffic', [])
        for spec in specs:
            bp = world.get_blueprint_library().find(spec.get('blueprint', 'vehicle.tesla.model3'))
            bp.set_attribute('role_name', spec['name'])
            actor = world.try_spawn_actor(bp, spawn_from_xodr(road_map, spec['spawn_xodr'], carla))
            if actor is None:
                raise RuntimeError(f"Cannot spawn {spec['name']}")
            vehicles.append(actor)
            if spec.get('controller') == 'v3_traffic':
                controllers[actor.id] = V3TrafficController(world, actor, spec, carla_api=carla)
            elif spec.get('controller') == 'native_rule':
                controllers[actor.id] = NativeRuleController(world, actor, spec, carla)
            else:
                actor.set_autopilot(True, 8000)
                tm.auto_lane_change(actor, False)
                tm.set_desired_speed(actor, spec.get('desired_speed_kmh', 35))
                if spec.get('path'):
                    tm.set_path(actor, [carla.Location(x=p[0], y=p[1], z=p[2]) for p in spec['path']])
            sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'), carla.Transform(), attach_to=actor)
            callback = barrier.wrap(lambda event, actor_id=actor.id: collisions.append({'frame': event.frame, 'actor_id': actor_id, 'other_id': event.other_actor.id}), allow_during_shutdown=True)
            callbacks.append(callback)
            sensor.listen(callback)
            sensors.append(sensor)
        world.tick()
        if plan.get('dynamic_traffic', {}).get('count', 0):
            traffic_session = setup_dynamic_traffic(world, tm, vehicles[0], plan['dynamic_traffic'],
                carla_api=carla, register_actor=vehicles.append)
            result['dynamic_traffic'] = traffic_session.evidence
        signal = setup_signal_policy(world, plan.get('signal_policy'), carla_api=carla)
        if signal:
            stable = 0
            for _ in range(20):
                world.tick()
                observed = signal.settling_observe(world.get_snapshot())
                stable = stable+1 if observed['pass'] else 0
                if stable >= 5: break
            if stable < 5: raise RuntimeError('Signals did not settle')
        for _ in range(15): world.tick()
        ego = vehicles[0]
        first = world.get_snapshot().timestamp.elapsed_seconds
        last = world.get_snapshot().frame
        with (args.out/'frames.jsonl').open('w', encoding='utf-8') as handle:
            for sample in range(round(args.seconds * 20)):
                snapshot = world.get_snapshot()
                if traffic_session: traffic_session.update(snapshot)
                # Match capture_native: traffic control precedes ego control.
                for actor in vehicles[1:] + vehicles[:1]:
                    if actor.id in controllers:
                        actor.apply_control(controllers[actor.id].step(snapshot))
                frame = world.tick()
                snapshot = world.get_snapshot()
                if frame != last+1 or snapshot.frame != frame:
                    raise RuntimeError('Second world ticker')
                last = frame
                if signal: signal.observe(snapshot)
                states = []
                for actor in vehicles:
                    native = snapshot.find(actor.id)
                    t, v, a, w = native.get_transform(), native.get_velocity(), native.get_acceleration(), native.get_angular_velocity()
                    box, c = actor.bounding_box, actor.get_control()
                    states.append({'id': actor.id, 'role_name': actor.attributes.get('role_name'), 'ego': actor.id == ego.id,
                        'blueprint': actor.type_id, 'transform': serialize_transform(t), 'waypoint': waypoint_record(road_map.get_waypoint(t.location)),
                        'velocity_mps': [v.x,v.y,v.z], 'speed_mps': math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
                        'acceleration_mps2': [a.x,a.y,a.z], 'angular_velocity_deg_s': [w.x,w.y,w.z],
                        'bounding_box': {'center_local_m': [box.location.x,box.location.y,box.location.z],
                            'extent_m':[box.extent.x,box.extent.y,box.extent.z], 'rotation_rpy_deg':[box.rotation.roll,box.rotation.pitch,box.rotation.yaw]},
                        'control': {key:getattr(c,key) for key in ['throttle','brake','steer','hand_brake','reverse']}})
                row = {'sample':sample,'frame':frame,'timestamp':snapshot.timestamp.elapsed_seconds,
                    'time_s':snapshot.timestamp.elapsed_seconds-first,'vehicles':states,'sensors':{},
                    'rule_controller': controllers[ego.id].diagnostics,
                    'traffic_control_diagnostics': {str(i): c.diagnostics for i,c in controllers.items()}}
                if v3: row['traffic_readiness'] = traffic_ready(snapshot, ego, {i:c for i,c in controllers.items() if i != ego.id}, plan.get('traffic_readiness',{}))
                handle.write(json.dumps(row,separators=(',',':'))+'\n')
                if sample % 200 == 0:
                    print(json.dumps({'sample':sample,'seconds':round(row['time_s'],2),'ego_kmh':round(states[0]['speed_mps']*3.6,2),
                        'stage':row['rule_controller'].get('stage'), 'waiting':row['rule_controller'].get('waiting'), 'collisions':len(collisions)}),flush=True)
                if plan.get('finish_capture_on_plan_complete') and controllers[ego.id].index >= len(controllers[ego.id].stages):
                    result['end_reason'] = 'native_ego_plan_completed'
                    break
        result.update(frames=sample+1, collisions=len(collisions), controller_events={str(i):c.events for i,c in controllers.items()})
    except Exception:
        result['error'] = traceback.format_exc()
    finally:
        # Persist physical results before any native subscription teardown.
        (args.out/'physics-complete.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        barrier.close_intake()
        for s in sensors: s.stop()
        result['callback_shutdown'] = barrier.wait_quiescent()
        snapshot = native = row = t = v = a = w = None
        gc.collect()
        if signal: result['signal_restoration'] = signal.restore()
        responses = client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in sensors+vehicles], False)
        errors = [r.error for r in responses if r.has_error()]
        tm.set_synchronous_mode(False)
        world.apply_settings(previous)
        controllers.clear()
        sensors.clear()
        vehicles.clear()
        callbacks.clear()
        traffic_session = signal = actor = ego = s = callback = responses = None
        tm = road_map = world = client = None
        gc.collect()
        result['cleanup_errors'] = errors
        result['cleanup_complete'] = not errors
        (args.out/'collisions.json').write_text(json.dumps(collisions),encoding='utf-8')
        (args.out/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in result.items() if k not in ('controller_events','dynamic_traffic')}),flush=True)
    return 1 if result['error'] or not result['cleanup_complete'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
