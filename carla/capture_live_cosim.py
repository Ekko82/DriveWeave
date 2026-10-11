"""Capture real live CARLA/SUMO co-simulation using the official 0.9.16 bridge.

This client requires an already running matching CARLA server. It starts SUMO
through the official bridge and does not generate stand-in sensor data. Only this client
should tick the world. A run receives a new directory; old data is never reused.
"""
from pathlib import Path
import sys
# Embedded Python isolates sys.path and does not add the script directory.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import argparse
import datetime
import faulthandler
import gc
import hashlib
import json
import math
import os
import queue
import random
import threading
import time


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--sumo-cfg', type=Path, required=True)
    p.add_argument('--sumo-home', type=Path, help='SUMO distribution; otherwise SUMO_HOME environment')
    p.add_argument('--carla-sumo', type=Path, help='Official bridge; otherwise CARLA_SUMO_ROOT or CARLA_ROOT')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--tm-port', type=int, default=8000)
    p.add_argument('--map', default='Town04')
    p.add_argument('--seconds', type=float, default=20)
    p.add_argument('--fps', type=int, choices=[10, 20, 30], default=20)
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=720)
    p.add_argument('--fov', type=float, default=90)
    p.add_argument('--traffic', type=int, default=12)
    p.add_argument('--spawn-index', type=int, default=0)
    p.add_argument('--seed', type=int, default=20261011)
    p.add_argument('--warmup-seconds', type=float, default=2)
    p.add_argument('--sensor-timeout', type=float, default=30)
    p.add_argument('--route-json', type=Path, help='Optional verified native map path/route settings.')
    p.add_argument('--chase', action='store_true', help='Add a native rear RGB camera showing the Tesla.')
    p.add_argument('--chase-x', type=float, default=-6.5)
    p.add_argument('--chase-y', type=float, default=2.1)
    p.add_argument('--chase-z', type=float, default=2.05)
    p.add_argument('--chase-pitch', type=float, default=-8)
    p.add_argument('--chase-yaw', type=float, default=-14)
    p.add_argument('--weather', choices=['clear', 'cloudy'], default='clear')
    p.add_argument('--instance', action='store_true', help='Add native instance-segmentation RGB codes for true instance masks.')
    p.add_argument('--keep-raw-bgra', action='store_true', help='Also keep exact camera raw_data bytes.')
    p.add_argument('--out', type=Path, default=Path(__file__).resolve().parent / 'captures')
    return p


def serialize_transform(transform):
    return {'location_m': [transform.location.x, transform.location.y, transform.location.z],
            'rotation_deg': [transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll],
            'world_from_local': transform.get_matrix()}


def waypoint_record(wp):
    return None if wp is None else {'road_id': wp.road_id, 'section_id': wp.section_id,
        'lane_id': wp.lane_id, 's_m': wp.s, 'junction': wp.is_junction,
        'lane_width_m': wp.lane_width, 'lane_change': str(wp.lane_change)}


def intrinsic(width, height, fov):
    focal = width / (2 * math.tan(math.radians(fov) / 2))
    return [[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]]


def same_frame(stream, frame, timeout):
    deadline = time.monotonic() + timeout
    while True:
        item = stream.get(timeout=max(.001, deadline - time.monotonic()))
        if item.frame == frame:
            return item
        if item.frame > frame:
            raise RuntimeError(f'Sensor skipped world frame {frame}, received {item.frame}')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def atomic_json(path, value):
    """Never truncate the last durable JSON if a native process dies mid-write."""
    temporary = path.with_name(path.name + '.tmp')
    payload = json.dumps(value, indent=2).encode('utf-8')
    with temporary.open('wb') as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


class CallbackBarrier:
    """Close intake, stop subscriptions, then wait for active callbacks to leave."""
    def __init__(self):
        self.condition = threading.Condition()
        self.accepting = True
        self.active = 0
        self.total = 0
        self.last_activity = time.monotonic()

    def wrap(self, handler, allow_during_shutdown=False):
        def receive(data):
            with self.condition:
                self.active += 1
                self.total += 1
                allowed = self.accepting or allow_during_shutdown
                self.last_activity = time.monotonic()
            try:
                if allowed:
                    handler(data)
            finally:
                with self.condition:
                    self.active -= 1
                    self.last_activity = time.monotonic()
                    self.condition.notify_all()
        return receive

    def close_intake(self):
        with self.condition:
            self.accepting = False

    def wait_quiescent(self, timeout=5, quiet_seconds=.25):
        deadline = time.monotonic()+timeout
        with self.condition:
            while self.active or time.monotonic()-self.last_activity < quiet_seconds:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(f'Sensor callbacks did not quiesce; active={self.active}')
                self.condition.wait(min(.05, remaining))
            return {'active':self.active,'total_callbacks':self.total,'quiet_seconds':quiet_seconds}


def main():
    faulthandler.enable(all_threads=True)
    args = parser().parse_args()
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
    from runtime_config import RuntimeConfig
    runtime_environment = dict(os.environ)
    if args.sumo_home is not None:
        runtime_environment['SUMO_HOME'] = str(args.sumo_home)
    if args.carla_sumo is not None:
        runtime_environment['CARLA_SUMO_ROOT'] = str(args.carla_sumo)
    runtime = RuntimeConfig.resolve(require_sumo=True,require_bridge=True,environ=runtime_environment)
    runtime.activate()
    args.sumo_home, args.carla_sumo = runtime.sumo_home, runtime.carla_sumo_root
    if args.seconds <= 0 or args.traffic < 0 or args.width < 64 or args.height < 64:
        raise SystemExit('Invalid duration, traffic count or image size.')
    # Deferred imports permit --help and syntax checks without an installed runtime.
    import carla
    import numpy as np
    from PIL import Image
    from native_controller import NativeRuleController, spawn_from_xodr
    from dynamic_traffic import setup_dynamic_traffic
    from physical_preroll import rolling_preroll
    from live_cosim_bridge import LiveCosimBridge

    route = json.loads(args.route_json.read_text(encoding='utf-8')) if args.route_json else {}
    if route.get('map') and route['map'] != args.map:
        raise RuntimeError('Route specification was built for a different map.')
    rule_mode = route.get('controller') == 'native_rule'

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.sensor_timeout)
    server_version, client_version = client.get_server_version(), client.get_client_version()
    if server_version != client_version:
        raise RuntimeError(f'CARLA client/server mismatch: {client_version} / {server_version}')
    available = client.get_available_maps()
    matches = [name for name in available if name.rsplit('/', 1)[-1] == args.map]
    if not matches:
        raise RuntimeError(f'Map {args.map} unavailable. Actual maps: {available}')
    world = client.load_world(matches[0])
    road_map = world.get_map()
    prior_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1 / args.fps
    settings.no_rendering_mode = False
    settings.substepping = True
    settings.max_substep_delta_time = .01
    settings.max_substeps = 10
    tm = None
    live_sync = None
    rng = random.Random(args.seed)
    actors, sensors, queues = [], {}, {}
    callbacks = []
    callback_barrier = CallbackBarrier()
    controller = None
    traffic_session = None
    npc_controllers = {}
    collisions, lane_events, callback_errors = [], [], []
    run = args.out.resolve() / datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    run.mkdir(parents=True, exist_ok=False)
    for folder in ['rgb', 'semantic_labels', 'semantic_palette', 'lidar'] + (['chase'] if args.chase else []) + (['instance'] if args.instance else []):
        (run / folder).mkdir()
    if args.keep_raw_bgra:
        (run / 'raw_bgra').mkdir()
    manifest = {'status': 'incomplete', 'server_version': server_version, 'client_version': client_version,
        'map': road_map.name, 'ego_blueprint': 'vehicle.tesla.model3',
        'source': 'native CARLA sensor callbacks after official live SimulationSynchronization.tick',
        'synthetic_sensor_fallback': False, 'trained_policy': False,
        'controller': 'native CARLA VehicleControl / transparent rules' if rule_mode else 'CARLA TrafficManager',
        'scenario': route.get('scenario', 'sensor_smoke'), 'route_specification': route,
        'seed': args.seed, 'fps': args.fps, 'fixed_delta_seconds': 1 / args.fps,
        'coordinate_system': {'axes': 'Unreal left-handed: x forward, y right, z up', 'units': 'metres',
            'lidar_reference': 'sensor-local XYZ; intensity in fourth float',
            'transform': 'world_from_local 4x4; no SUMO front-bumper conversion'},
        'camera_intrinsic': intrinsic(args.width, args.height, args.fov),
        'semantic_encoding': 'native BGRA red channel is class ID; 0.9.14+ taxonomy',
        'settings': vars(args) | {'out': str(args.out), 'route_json': str(args.route_json) if args.route_json else None,
            'sumo_cfg': str(args.sumo_cfg), 'sumo_home': str(args.sumo_home), 'carla_sumo': str(args.carla_sumo)},
        'frames_captured': 0}
    manifest['client_sources_sha256'] = {name:digest((Path(__file__).parent/name).read_bytes())
        for name in [Path(__file__).name,'native_controller.py','dynamic_traffic.py','physical_preroll.py']}
    atomic_json(run / 'manifest.json', manifest)
    print('CAPTURE_RUN '+str(run), flush=True)
    (run / 'map.xodr').write_text(road_map.to_opendrive(), encoding='utf-8')
    lifecycle = {'capture_run':str(run),'stages':[],'cleanup_complete':False}
    evidence_offsets = {'controller':0,'collision':0,'lane':0}

    def stage_log(name, **details):
        record = {'stage':name,'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),**details}
        lifecycle['current_stage'] = name
        lifecycle['stages'].append(record)
        atomic_json(run/'lifecycle.json',lifecycle)
        print('LIFECYCLE '+json.dumps(record,separators=(',',':')),flush=True)

    def persist_events():
        events = controller.events if controller is not None else []
        for name, entries, offset_name in [('controller',events,'controller'),
                ('collision',collisions,'collision'),('lane',lane_events,'lane')]:
            new = entries[evidence_offsets[offset_name]:]
            if new:
                with (run/(name+'-events-live.jsonl')).open('a',encoding='utf-8') as handle:
                    for entry in new:
                        handle.write(json.dumps(entry,separators=(',',':'))+'\n')
                    handle.flush()
                    os.fsync(handle.fileno())
                evidence_offsets[offset_name] += len(new)

    def persist_evidence():
        persist_events()
        atomic_json(run/'collisions.json',list(collisions))
        atomic_json(run/'lane-invasions.json',list(lane_events))
        if controller is not None:
            atomic_json(run/'controller-events.json',list(controller.events))

    stage_log('initial_evidence_persisted')
    try:
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon if args.weather == 'clear' else carla.WeatherParameters.CloudyNoon)
        tm = client.get_trafficmanager(args.tm_port)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(args.seed)
        tm.set_hybrid_physics_mode(False)
        tm.set_global_distance_to_leading_vehicle(3)
        tm.global_percentage_speed_difference(20)
        blueprint_library = world.get_blueprint_library()
        spawn_points = road_map.get_spawn_points()
        spawn_index = int(route.get('spawn_index', args.spawn_index))
        if not route.get('ego_spawn') and not 0 <= spawn_index < len(spawn_points):
            raise RuntimeError(f'spawn-index must be in 0..{len(spawn_points)-1}')
        bp = blueprint_library.find('vehicle.tesla.model3')
        bp.set_attribute('role_name', 'hero')
        if bp.has_attribute('color'):
            bp.set_attribute('color', '20,45,85')
        ego_spawn = spawn_from_xodr(road_map, route['ego_spawn'], carla) if route.get('ego_spawn') else spawn_points[spawn_index]
        ego = world.try_spawn_actor(bp, ego_spawn)
        if ego is None:
            raise RuntimeError('Tesla spawn occupied; select a verified clear native spawn index.')
        actors.append(ego)
        ego.set_autopilot(not rule_mode, args.tm_port)
        tm.distance_to_leading_vehicle(ego, 4)
        tm.ignore_lights_percentage(ego, 0)
        tm.ignore_signs_percentage(ego, 0)
        tm.ignore_vehicles_percentage(ego, 0)
        tm.auto_lane_change(ego, True)
        if route.get('path') and route.get('route'):
            raise RuntimeError('Choose a native path OR junction route instructions, not both.')
        if route.get('path'):
            tm.set_path(ego, [carla.Location(x=p[0], y=p[1], z=p[2]) for p in route['path']])
        if route.get('route'):
            if any(command not in ['Left', 'Right', 'Straight'] for command in route['route']):
                raise RuntimeError('Unsupported TrafficManager junction route command.')
            tm.set_route(ego, route['route'])
        if 'desired_speed_kmh' in route:
            tm.set_desired_speed(ego, float(route['desired_speed_kmh']))
        manifest['route_specification'] = route
        for traffic_spec in route.get('traffic', []):
            traffic_bp = blueprint_library.find(traffic_spec.get('blueprint', 'vehicle.audi.a2'))
            traffic_bp.set_attribute('role_name', traffic_spec['name'])
            traffic_pose = spawn_from_xodr(road_map, traffic_spec['spawn_xodr'], carla) if traffic_spec.get('spawn_xodr') else spawn_points[int(traffic_spec['spawn_index'])]
            traffic_actor = world.try_spawn_actor(traffic_bp, traffic_pose)
            if traffic_actor is None:
                raise RuntimeError(f'Explicit scenario actor could not spawn: {traffic_spec["name"]}')
            actors.append(traffic_actor)
            traffic_actor.set_autopilot(traffic_spec.get('controller') != 'native_rule', args.tm_port)
            tm.auto_lane_change(traffic_actor, traffic_spec.get('auto_lane_change', False))
            tm.ignore_vehicles_percentage(traffic_actor, 0)
            tm.ignore_lights_percentage(traffic_actor, 0)
            tm.ignore_signs_percentage(traffic_actor, 0)
            tm.distance_to_leading_vehicle(traffic_actor, 6.0)
            if 'desired_speed_kmh' in traffic_spec:
                tm.set_desired_speed(traffic_actor, float(traffic_spec['desired_speed_kmh']))
            if traffic_spec.get('route'):
                tm.set_route(traffic_actor, traffic_spec['route'])
            if traffic_spec.get('path'):
                tm.set_path(traffic_actor, [carla.Location(x=p[0],y=p[1],z=p[2]) for p in traffic_spec['path']])
            if traffic_spec.get('controller') == 'native_rule':
                npc_controllers[traffic_actor.id] = NativeRuleController(world, traffic_actor, traffic_spec, carla)
        # Refresh native actor caches before non-ticking traffic spawn guards.
        setup_frame = world.tick()
        stage_log('native_spawn_cache_refreshed',frame=setup_frame)
        if route.get('dynamic_traffic') is not None:
            traffic_session = setup_dynamic_traffic(world, tm, ego, route['dynamic_traffic'],
                carla_api=carla, register_actor=actors.append)
            manifest['dynamic_traffic'] = traffic_session.evidence

        traffic_bps = [b for b in blueprint_library.filter('vehicle.*')
                       if b.has_attribute('number_of_wheels') and int(b.get_attribute('number_of_wheels')) == 4
                       and b.id not in ['vehicle.tesla.cybertruck', 'vehicle.carlamotors.carlacola']]
        background_spawn_indices = list(range(len(spawn_points)))
        rng.shuffle(background_spawn_indices)
        for index in ([] if traffic_session is not None else background_spawn_indices):
            if len(actors) >= args.traffic + 1:
                break
            if spawn_points[index].location.distance(ego.get_location()) < 15:
                continue
            bg_bp = rng.choice(traffic_bps)
            bg_bp.set_attribute('role_name', 'autopilot')
            if bg_bp.has_attribute('color'):
                bg_bp.set_attribute('color', rng.choice(bg_bp.get_attribute('color').recommended_values))
            bg = world.try_spawn_actor(bg_bp, spawn_points[index])
            if bg is not None:
                actors.append(bg)
                bg.set_autopilot(True, args.tm_port)

        def attach(name, blueprint_id, pose, attributes):
            sensor_bp = blueprint_library.find(blueprint_id)
            for key, value in attributes.items():
                if sensor_bp.has_attribute(key):
                    sensor_bp.set_attribute(key, str(value))
                else:
                    raise RuntimeError(f'{blueprint_id} has no requested attribute {key}')
            sensor = world.spawn_actor(sensor_bp, pose, attach_to=ego, attachment_type=carla.AttachmentType.Rigid)
            sensors[name] = sensor
            actors.append(sensor)
            stream = queue.Queue(maxsize=16)
            queues[name] = stream
            def receive(data):
                try:
                    stream.put_nowait(data)
                except queue.Full:
                    callback_errors.append(f'{name}: sensor queue overflow at frame {data.frame}')
            wrapped = callback_barrier.wrap(receive)
            callbacks.append(wrapped)
            sensor.listen(wrapped)
            manifest.setdefault('sensors', {})[name] = {'blueprint': blueprint_id, 'attributes': attributes,
                'relative_transform': serialize_transform(pose)}

        camera_attributes = {'image_size_x': args.width, 'image_size_y': args.height, 'fov': args.fov,
            'sensor_tick': 0, 'lens_k': 0, 'lens_kcube': 0}
        front_pose = carla.Transform(carla.Location(x=1.65, z=1.65))
        attach('rgb', 'sensor.camera.rgb', front_pose, camera_attributes | {'motion_blur_intensity': 0})
        attach('semantic', 'sensor.camera.semantic_segmentation', front_pose, camera_attributes)
        if args.instance:
            attach('instance', 'sensor.camera.instance_segmentation', front_pose, camera_attributes)
        attach('lidar', 'sensor.lidar.ray_cast', carla.Transform(carla.Location(z=2.3)),
            {'sensor_tick': 0, 'channels': 32, 'range': 70, 'points_per_second': 320000,
             'rotation_frequency': args.fps, 'upper_fov': 10, 'lower_fov': -30,
             'noise_stddev': .01, 'dropoff_general_rate': .1})
        if args.chase:
            attach('chase', 'sensor.camera.rgb',
                carla.Transform(carla.Location(x=args.chase_x, y=args.chase_y, z=args.chase_z),
                    carla.Rotation(pitch=args.chase_pitch, yaw=args.chase_yaw)),
                camera_attributes | {'motion_blur_intensity': 0})
        for sensor_id, bucket, reader in [
            ('sensor.other.collision', collisions, lambda e: {'other_actor': e.other_actor.id,
                'other_type': e.other_actor.type_id, 'normal_impulse': [e.normal_impulse.x, e.normal_impulse.y, e.normal_impulse.z]}),
            ('sensor.other.lane_invasion', lane_events, lambda e: {'markings': [str(m.type) for m in e.crossed_lane_markings]})]:
            event_sensor = world.spawn_actor(blueprint_library.find(sensor_id), carla.Transform(), attach_to=ego)
            actors.append(event_sensor)
            sensors[sensor_id] = event_sensor
            wrapped = callback_barrier.wrap(lambda e, store=bucket, details=reader: store.append(
                {'frame': e.frame, 'timestamp': e.timestamp} | details(e)),allow_during_shutdown=True)
            callbacks.append(wrapped)
            event_sensor.listen(wrapped)
        manifest['actor_ids'] = [a.id for a in actors if a.type_id.startswith('vehicle.')]
        if rule_mode:
            controller = NativeRuleController(world, ego, route, carla)
            ego.apply_control(carla.VehicleControl(brake=1))
        live_sync = LiveCosimBridge(args, run, world)
        manifest['live_co_simulation'] = live_sync.metadata
        def apply_controls(snapshot):
            state = traffic_session.update(snapshot) if traffic_session is not None else None
            for actor_id, npc in npc_controllers.items():
                npc.ego.apply_control(npc.step(snapshot))
            if controller is not None:
                ego.apply_control(controller.step(snapshot))
            persist_events()
            return {'ego':dict(controller.diagnostics) if controller is not None else None,
                'npc':{str(i):dict(c.diagnostics) for i,c in npc_controllers.items()},
                'dynamic_traffic':state}

        def drain(frame):
            for stream in queues.values():
                same_frame(stream, frame, args.sensor_timeout)
            if callback_errors:
                raise RuntimeError(callback_errors[0])

        if route.get('hot_start', False):
            manifest['physical_preroll'] = rolling_preroll(world, ego, live_sync.tick, drain,
                apply_controls, run, route, args.warmup_seconds, args.fps)
            if controller is not None and controller.index != 0:
                raise RuntimeError('Preroll completed approach stage; extend native approach before recording')
        else:
            for _ in range(round(args.warmup_seconds * args.fps)):
                for npc in npc_controllers.values():
                    npc.ego.apply_control(npc.step(world.get_snapshot()))
                drain(live_sync.tick())
        first_time, previous_frame, maximum_skew = None, None, 0
        stage_log('capture_started')
        atomic_json(run/'manifest.json',manifest)
        with (run / 'frames.jsonl').open('w', encoding='utf-8') as log:
            for sample in range(round(args.seconds * args.fps)):
                control_state = apply_controls(world.get_snapshot())
                frame = live_sync.tick()
                snapshot = world.get_snapshot()
                data = {name: same_frame(stream, frame, args.sensor_timeout) for name, stream in queues.items()}
                if callback_errors:
                    raise RuntimeError(callback_errors[0])
                if snapshot.frame != frame or previous_frame is not None and frame != previous_frame + 1:
                    raise RuntimeError('Another client ticked the world or world snapshot is inconsistent.')
                previous_frame = frame
                current_time = snapshot.timestamp.elapsed_seconds
                if first_time is None:
                    first_time = current_time
                skew = max(abs(item.timestamp - current_time) for item in data.values())
                maximum_skew = max(maximum_skew, skew)
                if skew > 1e-5:
                    raise RuntimeError(f'Sensor/world timestamp skew {skew}s at frame {frame}')
                sensor_info = {}
                for name, item in data.items():
                    sensor_info[name] = {'frame': item.frame, 'timestamp': item.timestamp,
                        'transform': serialize_transform(item.transform), 'raw_sha256': digest(item.raw_data)}
                    if name == 'lidar':
                        points = np.frombuffer(item.raw_data, dtype='<f4').reshape(-1, 4)
                        if not len(points) or not np.isfinite(points).all():
                            raise RuntimeError(f'Empty/invalid native LiDAR frame {frame}')
                        np.save(run / 'lidar' / f'{frame:08d}.npy', points, allow_pickle=False)
                        sensor_info[name].update({'points': len(points), 'horizontal_angle_rad': item.horizontal_angle,
                            'channels': item.channels, 'counts_per_channel': [item.get_point_count(i) for i in range(item.channels)]})
                    else:
                        pixels = np.frombuffer(item.raw_data, dtype=np.uint8).reshape(item.height, item.width, 4)
                        if args.keep_raw_bgra:
                            (run / 'raw_bgra' / f'{name}_{frame:08d}.bin').write_bytes(item.raw_data)
                        if name == 'semantic':
                            labels = pixels[:, :, 2].copy()
                            sensor_info[name]['class_ids'] = np.unique(labels).tolist()
                            Image.fromarray(labels).save(run / 'semantic_labels' / f'{frame:08d}.png')
                            item.save_to_disk(str(run / 'semantic_palette' / f'{frame:08d}.png'), carla.ColorConverter.CityScapesPalette)
                        else:
                            rgb = pixels[:, :, [2, 1, 0]]
                            if name != 'instance' and rgb.std() < 1:
                                raise RuntimeError(f'Uniform/empty native {name} frame {frame}')
                            Image.fromarray(rgb).save(run / name / f'{frame:08d}.png')
                        sensor_info[name].update({'width': item.width, 'height': item.height, 'fov': item.fov})
                vehicle_states = []
                # Include official SUMO mirrors in raw visibility/OBB evidence.
                for actor in world.get_actors().filter('vehicle.*'):
                    if not actor.type_id.startswith('vehicle.'):
                        continue
                    actor_snapshot = snapshot.find(actor.id)
                    if actor_snapshot is None:
                        continue
                    transform = actor_snapshot.get_transform()
                    velocity = actor_snapshot.get_velocity()
                    acceleration = actor_snapshot.get_acceleration()
                    angular_velocity = actor_snapshot.get_angular_velocity()
                    ctrl = actor.get_control()
                    if not all(math.isfinite(v) for v in (ctrl.throttle,ctrl.brake,ctrl.steer)):
                        raise RuntimeError(f'Native nonfinite control for actor {actor.id} at frame {frame}')
                    bbox = actor.bounding_box
                    vehicle_states.append({'id': actor.id, 'blueprint': actor.type_id,
                        'role_name': actor.attributes.get('role_name'), 'ego': actor.id == ego.id,
                        'control_authority': 'SUMO official kinematic mirror' if actor.id in live_sync.sync.sumo2carla_ids.values() else 'CARLA native physics',
                        'transform': serialize_transform(transform), 'velocity_mps': [velocity.x, velocity.y, velocity.z],
                        'speed_mps': math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2),
                        'acceleration_mps2':[acceleration.x,acceleration.y,acceleration.z],
                        'angular_velocity_deg_s':[angular_velocity.x,angular_velocity.y,angular_velocity.z],
                        'bounding_box': {'center_local_m': [bbox.location.x, bbox.location.y, bbox.location.z],
                            'extent_m': [bbox.extent.x, bbox.extent.y, bbox.extent.z],
                            'rotation_rpy_deg': [bbox.rotation.roll, bbox.rotation.pitch, bbox.rotation.yaw]},
                        'waypoint': waypoint_record(road_map.get_waypoint(transform.location)),
                        'control': {'throttle': ctrl.throttle, 'brake': ctrl.brake, 'steer': ctrl.steer,
                            'hand_brake': ctrl.hand_brake, 'reverse': ctrl.reverse}})
                record = {'sample': sample, 'frame': frame, 'timestamp': current_time,
                    'time_s': current_time - first_time, 'sensors': sensor_info, 'vehicles': vehicle_states,
                    'ego_traffic_light_state': str(ego.get_traffic_light_state()),
                    'ego_is_at_traffic_light': ego.is_at_traffic_light()}
                record['traffic_control_diagnostics'] = control_state
                if controller is not None:
                    record['rule_controller'] = controller.diagnostics
                record['live_co_simulation'] = live_sync.record(sample, snapshot)
                log.write(json.dumps(record, separators=(',', ':')) + '\n')
                manifest['frames_captured'] += 1
                # Drop SensorData and NumPy views before the next tick and,
                # crucially, before subscriptions or actors are destroyed.
                data.clear()
                item = points = pixels = rgb = labels = None
                persist_events()
                if sample % args.fps == 0:
                    log.flush()
                    os.fsync(log.fileno())
                    atomic_json(run/'manifest.json',manifest)
                    persist_evidence()
                    print(f'CAPTURE frame={frame} sample={sample} points={sensor_info["lidar"]["points"]}', flush=True)
                if route.get('finish_capture_on_plan_complete',False) and controller is not None and controller.index == len(controller.stages):
                    manifest['recording_end_reason']='all native controller stages physically completed'
                    stage_log('physical_action_recording_finished',frame=frame,sample=sample)
                    break
            log.flush()
            os.fsync(log.fileno())
        manifest.update({'status': 'samples_complete_pending_cleanup', 'capture_samples_complete':True,
            'native_cleanup_complete':False,'first_sensor_time_s': first_time,
            'duration_s': manifest['frames_captured'] / args.fps, 'max_sensor_timestamp_skew_s': maximum_skew,
            'collision_count': len(collisions), 'lane_invasion_count': len(lane_events),
            'checks': {'all_streams_same_world_frame': True, 'consecutive_frames': True,
                'finite_nonempty_lidar': True, 'nonuniform_rgb': True},
            'scenario_actions_validated': False, 'final_video_validated': False})
        manifest['live_co_simulation'].update(live_sync.summary())
        if controller is not None:
            manifest['controller_plan_completed'] = controller.index == len(controller.stages)
        persist_evidence()
        # This is durable sample evidence, not a claim of successful native
        # process exit. The wrapper independently reports the actual exit code.
        atomic_json(run/'capture-completion.json',dict(manifest))
        atomic_json(run/'manifest.json',manifest)
        stage_log('sample_completion_persisted',frames=manifest['frames_captured'])
    except BaseException as error:
        manifest['error'] = f'{type(error).__name__}: {error}'
        atomic_json(run/'manifest.json',manifest)
        persist_evidence()
        stage_log('python_error',error=manifest['error'])
        raise
    finally:
        stage_log('cleanup_begin')
        faulthandler.dump_traceback_later(40, repeat=False)
        callback_barrier.close_intake()
        cleanup_errors = []
        # Stop every data AND event subscription before destroying any actor.
        for sensor_name, sensor in list(sensors.items()):
            try:
                sensor.stop()
                stage_log('sensor_stopped',sensor=sensor_name,actor_id=sensor.id)
            except RuntimeError as error:
                cleanup_errors.append(f'stop {sensor_name}: {error}')
        try:
            stage_log('callbacks_quiescent',**callback_barrier.wait_quiescent())
        except RuntimeError as error:
            cleanup_errors.append(str(error))
        for stream in queues.values():
            while True:
                try:
                    queued = stream.get_nowait()
                    del queued
                except queue.Empty:
                    break
        # Explicitly remove all retained native data prior to actor teardown.
        data = item = points = pixels = rgb = labels = snapshot = actor_snapshot = None
        gc.collect()
        stage_log('sensor_data_released')
        if live_sync is not None:
            try:
                live_sync.close()
                stage_log('official_live_synchronization_closed')
            except BaseException as error:
                cleanup_errors.append(f'live sync close: {error}')
            finally:
                live_sync = None
                gc.collect()
        persist_evidence()
        manifest['collision_count'] = len(collisions)
        manifest['lane_invasion_count'] = len(lane_events)
        sensor_ids = [a.id for a in actors if a.type_id.startswith('sensor.')]
        vehicle_ids = [a.id for a in actors if a.type_id.startswith('vehicle.')]
        stage_log('actor_batch_destroy_begin',sensor_ids=sensor_ids,vehicle_ids=vehicle_ids)
        try:
            responses = client.apply_batch_sync([carla.command.DestroyActor(i) for i in sensor_ids+vehicle_ids],False)
            for response in responses:
                if response.has_error():
                    cleanup_errors.append(f'destroy actor {response.actor_id}: {response.error}')
            stage_log('actor_batch_destroy_finished',errors=list(cleanup_errors))
        except RuntimeError as error:
            cleanup_errors.append(f'batch destroy: {error}')
        try:
            if tm is not None:
                tm.set_synchronous_mode(False)
            stage_log('traffic_manager_sync_restored')
            world.apply_settings(prior_settings)
            stage_log('world_settings_restored')
        except RuntimeError as cleanup_error:
            cleanup_errors.append(str(cleanup_error))
        npc_controllers.clear()
        traffic_session = None
        npc = None
        sensors.clear()
        queues.clear()
        callbacks.clear()
        actors.clear()
        sensor = event_sensor = actor = traffic_actor = bg = ego = controller = None
        responses = None
        gc.collect()
        stage_log('actor_python_references_released')
        # Release native networking contexts while Python and its callback
        # runtime are still fully alive, rather than at interpreter shutdown.
        tm = road_map = world = client = None
        gc.collect()
        stage_log('native_client_contexts_released')
        lifecycle['cleanup_complete'] = not cleanup_errors
        manifest['native_cleanup_complete'] = not cleanup_errors
        if cleanup_errors:
            manifest['cleanup_errors'] = cleanup_errors
            manifest['status'] = 'cleanup_failed'
        elif manifest.get('capture_samples_complete'):
            manifest['status'] = 'complete'
        atomic_json(run/'manifest.json',manifest)
        stage_log('cleanup_finished',errors=cleanup_errors)
        faulthandler.cancel_dump_traceback_later()
        print(json.dumps({'run': str(run), 'status': manifest['status'], 'frames': manifest['frames_captured']}), flush=True)
        if cleanup_errors:
            raise RuntimeError('Native cleanup failed: '+'; '.join(cleanup_errors))


if __name__ == '__main__':
    main()
