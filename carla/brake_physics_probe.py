"""Owner-only native Tesla braking diagnostic, never an accepted recording.

Each trial loads a fresh Town05 world, spawns one native Tesla at 6/-1/s15.5,
and uses VehicleControl only. No NPCs, sensors, pose/velocity setters, altered
physics, or fabricated samples. Run only after the sole runtime owner is idle.
"""
from __future__ import annotations

from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / 'verification'))
import argparse
import hashlib
import json
import math
import time
import traceback

import carla
import numpy as np
from audit_motion_quality import local_polynomial_derivatives, summarize, load_thresholds

MODES = ['fixed_0.01', 'fixed_0.03', 'fixed_0.05', 'fixed_0.08', 'smooth_cap',
         'coast_only', 'neutral_low', 'tiny_throttle_low',
         'tapered_coast', 'tapered_tiny_drive']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vector(v):
    return [float(v.x), float(v.y), float(v.z)]


def control_record(c):
    return {k: getattr(c, k) for k in ('throttle', 'brake', 'steer', 'hand_brake',
                                     'reverse', 'manual_gear_shift', 'gear')}


def angular_delta(a, b):
    return (float(b) - float(a) + 180.) % 360. - 180.


def brake_request(mode, speed):
    if speed <= .1:
        return .35
    if mode == 'smooth_cap':
        return .015 + .06 * min(max(speed, 0.) / 4., 1.) ** 2
    if mode == 'coast_only':
        return 0.
    if mode in ('neutral_low', 'tiny_throttle_low'):
        return .01
    if mode in ('tapered_coast', 'tapered_tiny_drive'):
        return .01 * min(max((speed-2.5) / .3, 0.), 1.)
    return float(mode.removeprefix('fixed_'))


def diagnostic_control_request(mode, phase, speed, low_elapsed_s=None):
    """VehicleControl candidates only; measured speed activates the low phase.

    The neutral gear request is intentionally discrete and native, not a claim
    of a continuous drivetrain response. Tiny throttle ramps over 0.5s. All
    outcomes must be decided from the retained actual snapshots.
    """
    if mode not in MODES:
        raise ValueError('Unknown native braking diagnostic mode')
    throttle = .45 if phase == 'accelerate' else 0.
    brake = .35 if phase in ('settle', 'hold') else brake_request(mode, speed) if phase == 'brake' else 0.
    manual = mode == 'neutral_low' and low_elapsed_s is not None and phase in ('brake', 'hold')
    if mode == 'tiny_throttle_low' and low_elapsed_s is not None and phase == 'brake':
        throttle = .02 * min(max(low_elapsed_s / .5, 0.), 1.)
    if mode in ('tapered_coast', 'tapered_tiny_drive') and low_elapsed_s is not None and phase == 'brake':
        # Once actual speed crossed 2.5, keep brake exactly zero even if a later
        # physical response raises speed; no hidden epsilon brake is permitted.
        brake = 0.
        if mode == 'tapered_tiny_drive':
            ramp_up = min(max(low_elapsed_s / .5, 0.), 1.)
            taper_time = max(1. - low_elapsed_s / 6., 0.)
            taper_speed = min(max(speed / 2.5, 0.), 1.)
            throttle = .02 * ramp_up * taper_time * taper_speed
    return {'throttle': throttle, 'brake': brake, 'manual_gear_shift': manual, 'gear': 0}


def native_reference(start, minimum_length_m=400.):
    """Select genuine next waypoints with smallest heading change at forks."""
    points = [start]
    total = 0.
    while total < minimum_length_m:
        candidates = points[-1].next(.5)
        if not candidates:
            raise RuntimeError('Native physics-probe road reference ends early')
        q = min(candidates, key=lambda p: abs(angular_delta(
            points[-1].transform.rotation.yaw, p.transform.rotation.yaw)))
        distance = q.transform.location.distance(points[-1].transform.location)
        if q.lane_type != carla.LaneType.Driving or distance > 1. or distance < .01:
            raise RuntimeError('Invalid native physics-probe reference segment')
        points.append(q)
        total += distance
    return points


def steering(reference, transform, speed):
    """Physical lane tracking; never moves the actor to a waypoint."""
    loc = transform.location
    nearest = min(range(len(reference)), key=lambda i: reference[i].transform.location.distance(loc))
    distance = 0.
    target = nearest
    lookahead = max(3., 3. + speed * .5)
    while target + 1 < len(reference) and distance < lookahead:
        distance += reference[target].transform.location.distance(reference[target+1].transform.location)
        target += 1
    world = reference[target].transform.location
    forward, right = transform.get_forward_vector(), transform.get_right_vector()
    delta = np.asarray(vector(world)) - np.asarray(vector(loc))
    x, y = float(delta @ np.asarray(vector(forward))), float(delta @ np.asarray(vector(right)))
    tire_angle = math.atan2(2.8 * 2. * y, max(x*x + y*y, .1))
    return max(-.4, min(.4, tire_angle / math.radians(70.))), nearest


def physics_record(vehicle):
    p = vehicle.get_physics_control()
    result = {k: getattr(p, k) for k in ('mass', 'drag_coefficient', 'use_gear_autobox') if hasattr(p, k)}
    result['center_of_mass'] = vector(p.center_of_mass)
    result['wheels'] = []
    for w in p.wheels:
        row = {k: getattr(w, k) for k in ('tire_friction', 'damping_rate', 'max_steer_angle',
                                         'radius', 'max_brake_torque', 'max_handbrake_torque')
               if hasattr(w, k)}
        row['position'] = vector(w.position)
        result['wheels'].append(row)
    return result


def derivatives(rows, thresholds, braking_start_time):
    times = np.asarray([q['sim_time_s'] for q in rows], float)
    speed = np.asarray([q['speed_mps'] for q in rows], float)
    raw_a = np.gradient(speed, times, edge_order=2)
    raw_j = np.gradient(raw_a, times, edge_order=2)
    a, j = local_polynomial_derivatives(times, speed, .5)
    native_a = np.asarray([q['native_acceleration_forward_mps2'] for q in rows], float)
    for i, row in enumerate(rows):
        row.update(raw_speed_acceleration_mps2=float(raw_a[i]), raw_speed_jerk_mps3=float(raw_j[i]),
                   windowed_acceleration_mps2=float(a[i]), windowed_jerk_mps3=float(j[i]))
    subsets = {'whole_trial': np.ones(len(rows), bool)}
    if braking_start_time is not None:
        subsets['braking_with_actual_transition'] = times >= braking_start_time - .5 - 1e-7
        subsets['low_speed_braking_and_hold'] = (times >= braking_start_time - 1e-7) & (speed <= 2.)
    result = {}
    for name, selected in subsets.items():
        if not selected.any():
            continue
        aa, jj, tt = a[selected], j[selected], times[selected] - times[0]
        result[name] = {'samples': int(selected.sum()),
            'actual_windowed_acceleration_mps2': summarize(aa, tt),
            'actual_windowed_jerk_mps3': summarize(jj, tt),
            'raw_acceleration_mps2': summarize(raw_a[selected], tt),
            'raw_jerk_mps3': summarize(raw_j[selected], tt),
            'native_snapshot_acceleration_forward_mps2': summarize(native_a[selected], tt),
            'acceleration_target_met': bool(aa.max() <= thresholds.max_windowed_acceleration_mps2
                and aa.min() >= -thresholds.max_windowed_braking_mps2),
            'jerk_target_met': bool(np.quantile(abs(jj), .95) <= thresholds.p95_abs_windowed_jerk_mps3
                and abs(jj).max() <= thresholds.max_abs_windowed_jerk_mps3)}
    return result


def one_trial(client, mode, args, thresholds):
    out = args.out / mode
    out.mkdir()
    rows, events, vehicle = [], [], None
    world = client.load_world('Town05')
    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = .05
    settings.substepping = True
    settings.max_substep_delta_time = .01
    settings.max_substeps = 10
    world.apply_settings(settings)
    result = {'mode': mode, 'diagnostic_only': True, 'accepted_capture': False,
              'error': None, 'cleanup_complete': False, 'target_reached': False,
              'true_speed_stop_observed': False, 'braking_start_time_s': None,
              'sensor_count': 0, 'npc_count': 0, 'events': events}
    try:
        road_map = world.get_map()
        native_xodr = road_map.to_opendrive().encode('utf-8')
        (out / 'map.xodr').write_bytes(native_xodr)
        result['map_sha256'] = hashlib.sha256(native_xodr).hexdigest()
        start = road_map.get_waypoint_xodr(6, -1, 15.5)
        if start is None or start.lane_type != carla.LaneType.Driving:
            raise RuntimeError('Missing native 6/-1/s15.5 driving spawn')
        reference = native_reference(start)
        (out / 'native-reference.json').write_text(json.dumps([
            {'road_id': w.road_id, 'lane_id': w.lane_id, 's_m': w.s,
             'location_m': vector(w.transform.location), 'yaw_deg': w.transform.rotation.yaw}
            for w in reference], indent=2), encoding='utf-8')
        result['native_reference_sha256'] = sha(out / 'native-reference.json')
        bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        bp.set_attribute('role_name', 'brake_physics_probe')
        spawn_transform = start.transform
        spawn_transform.location.z += .3
        vehicle = world.try_spawn_actor(bp, spawn_transform)
        if vehicle is None:
            raise RuntimeError('Native Tesla spawn failed')
        result.update(actor_id=vehicle.id, blueprint=vehicle.type_id,
                      native_unmodified_physics=physics_record(vehicle))
        # A newly spawned actor is not present in the preceding world snapshot.
        # Apply a real hold and advance one owned tick before reading its state.
        vehicle.apply_control(carla.VehicleControl(throttle=0., brake=.35,
            steer=0., hand_brake=False, reverse=False, manual_gear_shift=False))
        initial_frame = world.tick()
        initial_snapshot = world.get_snapshot()
        initial_actor = initial_snapshot.find(vehicle.id)
        if initial_snapshot.frame != initial_frame or initial_actor is None:
            raise RuntimeError('Native Tesla did not appear in its owned spawn tick')
        result['spawn_setup_snapshot'] = {'frame': initial_frame,
            'sim_time_s': initial_snapshot.timestamp.elapsed_seconds,
            'location_m': vector(initial_actor.get_transform().location),
            'velocity_mps': vector(initial_actor.get_velocity()),
            'native_acceleration_mps2': vector(initial_actor.get_acceleration()),
            'actual_control': control_record(vehicle.get_control())}
        phase, origin, phase_start, stopped_at = 'settle', None, None, None
        low_started = None
        prior_frame = None
        for _ in range(math.ceil((1. + args.acceleration_timeout + args.braking_timeout + 2.) / .05)):
            prior = world.get_snapshot()
            actor_before = prior.find(vehicle.id)
            if actor_before is None:
                raise RuntimeError('Native Tesla snapshot missing before tick')
            v = actor_before.get_velocity()
            speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            sim_before = prior.timestamp.elapsed_seconds
            if origin is None:
                origin = phase_start = sim_before
            if phase == 'settle' and sim_before - phase_start >= .5 - 1e-7:
                phase, phase_start = 'accelerate', sim_before
                events.append({'event': 'accelerate_start', 'native_frame': prior.frame, 'sim_time_s': sim_before})
            if phase == 'accelerate' and speed >= args.target_speed:
                phase, phase_start = 'brake', sim_before
                result['target_reached'] = True
                result['braking_start_time_s'] = sim_before
                events.append({'event': 'actual_target_speed_reached', 'native_frame': prior.frame,
                               'sim_time_s': sim_before, 'actual_speed_mps': speed})
            if phase == 'accelerate' and sim_before - phase_start > args.acceleration_timeout:
                result['termination_reason'] = 'acceleration_timeout'
                break
            if phase in ('brake', 'hold') and sim_before - result['braking_start_time_s'] > args.braking_timeout:
                result['termination_reason'] = 'braking_timeout'
                break
            if phase == 'brake' and speed <= .1:
                phase, stopped_at = 'hold', sim_before
                result['true_speed_stop_observed'] = True
                events.append({'event': 'actual_speed_at_or_below_0p1', 'native_frame': prior.frame,
                               'sim_time_s': sim_before, 'actual_speed_mps': speed})
            if phase == 'brake' and mode in ('neutral_low', 'tiny_throttle_low', 'tapered_coast', 'tapered_tiny_drive') and speed <= 2.5 and low_started is None:
                low_started = sim_before
                events.append({'event': 'actual_speed_below_2p5_low_candidate_start',
                    'native_frame': prior.frame, 'sim_time_s': sim_before,
                    'actual_speed_mps': speed, 'mode': mode})
            # A fixed physical throttle reaches the measured threshold; a
            # proportional target controller can asymptote below it forever.
            low_elapsed = sim_before - low_started if low_started is not None else None
            candidate = diagnostic_control_request(mode, phase, speed, low_elapsed)
            steer, nearest = steering(reference, actor_before.get_transform(), speed)
            if phase in ('settle', 'hold'):
                steer = 0.
            requested = carla.VehicleControl(**candidate, steer=steer,
                hand_brake=False, reverse=False)
            vehicle.apply_control(requested)
            frame = world.tick()
            snapshot = world.get_snapshot()
            if snapshot.frame != frame or prior_frame is not None and frame != prior_frame + 1:
                raise RuntimeError('Unexpected native frame / second world ticker')
            prior_frame = frame
            actor = snapshot.find(vehicle.id)
            if actor is None:
                raise RuntimeError('Native Tesla snapshot missing after tick')
            t, velocity, acceleration = actor.get_transform(), actor.get_velocity(), actor.get_acceleration()
            forward = t.get_forward_vector()
            actual_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            wp = road_map.get_waypoint(t.location)
            counts = {'vehicle_count': len(world.get_actors().filter('vehicle.*')),
                      'sensor_count': len(world.get_actors().filter('sensor.*'))}
            if counts != {'vehicle_count': 1, 'sensor_count': 0}:
                raise RuntimeError('Probe world no longer contains only its Tesla and no sensors')
            row = {'sample': len(rows), 'frame': frame, 'sim_time_s': snapshot.timestamp.elapsed_seconds,
                'time_s': snapshot.timestamp.elapsed_seconds - origin, 'phase': phase,
                'location_m': vector(t.location),
                'rotation_rpy_deg': [t.rotation.roll, t.rotation.pitch, t.rotation.yaw],
                'world_from_local': t.get_matrix(), 'velocity_mps': vector(velocity),
                'speed_mps': actual_speed,
                'forward_speed_mps': velocity.x*forward.x + velocity.y*forward.y + velocity.z*forward.z,
                'native_snapshot_acceleration_mps2': vector(acceleration),
                'native_acceleration_forward_mps2': acceleration.x*forward.x + acceleration.y*forward.y + acceleration.z*forward.z,
                'requested_control': control_record(requested), 'actual_control': control_record(vehicle.get_control()),
                'candidate_diagnostic': {'mode': mode, 'low_phase_activated_from_actual_speed': low_started is not None,
                    'low_phase_elapsed_s': low_elapsed, 'low_threshold_mps': 2.5,
                    'tiny_throttle_ramp_seconds': .5, 'native_state_override': False,
                    'tapered_brake_from_mps': 2.8,
                    'tapered_tiny_drive_max_time_s': 6.,
                    'manual_neutral_requested': bool(candidate['manual_gear_shift'])},
                'reference_nearest_index': nearest,
                'native_lane_center_error_m': wp.transform.location.distance(t.location),
                'native_waypoint': {'road_id': wp.road_id, 'lane_id': wp.lane_id, 's_m': wp.s}, **counts}
            rows.append(row)
            if phase == 'hold' and snapshot.timestamp.elapsed_seconds - stopped_at >= 2.:
                result['termination_reason'] = 'actual_stop_plus_2s_hold'
                break
        if len(rows) < 5:
            raise RuntimeError('Insufficient actual native samples for fixed-window derivatives')
        result['actual_derivative_metrics'] = derivatives(rows, thresholds, result['braking_start_time_s'])
        result['samples'] = len(rows)
        result['native_delta_seconds_max_error'] = float(max(abs(np.diff([q['sim_time_s'] for q in rows])-.05)))
        result['maximum_native_lane_center_error_m'] = max(q['native_lane_center_error_m'] for q in rows)
        result['stop_snapshot_neighborhood'] = [q for q in rows
            if stopped_at is not None and abs(q['sim_time_s']-stopped_at) <= 1. + 1e-7]
    except Exception:
        result['error'] = traceback.format_exc()
    finally:
        errors = []
        if vehicle is not None:
            try:
                if not vehicle.destroy():
                    errors.append('Native Tesla destroy returned false')
            except Exception:
                errors.append(traceback.format_exc())
        try:
            world.apply_settings(original)
        except Exception:
            errors.append(traceback.format_exc())
        result['cleanup_errors'] = errors
        result['cleanup_complete'] = not errors
        (out / 'frames.jsonl').write_text(''.join(json.dumps(q) + '\n' for q in rows), encoding='utf-8')
        result['frames_sha256'] = sha(out / 'frames.jsonl')
        result['derivative_method'] = {'function': 'audit_motion_quality.local_polynomial_derivatives',
            'window_s': .5, 'minimum_actual_samples': 5, 'no_spike_deletion': True,
            'no_resampling': True, 'native_snapshot_acceleration_kept_separately': True}
        metric = result.get('actual_derivative_metrics', {}).get('braking_with_actual_transition', {})
        result['braking_candidate_meets_project_motion_targets'] = bool(
            result['error'] is None and result['cleanup_complete'] and result['target_reached']
            and result['true_speed_stop_observed'] and result.get('termination_reason') == 'actual_stop_plus_2s_hold'
            and metric.get('acceleration_target_met') and metric.get('jerk_target_met'))
        (out / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({'mode': mode, 'samples': len(rows), 'error': result['error'],
        'stop': result['true_speed_stop_observed'], 'motion_targets': result['braking_candidate_meets_project_motion_targets']}), flush=True)
    return result


def build_parser():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--host', default='127.0.0.1')
    cli.add_argument('--port', type=int, default=2000)
    cli.add_argument('--owned-fresh-world', action='store_true', required=True)
    cli.add_argument('--out', type=Path, required=True)
    cli.add_argument('--target-speed', type=float, default=6.)
    cli.add_argument('--acceleration-timeout', type=float, default=20.)
    cli.add_argument('--braking-timeout', type=float, default=30.)
    selection = cli.add_mutually_exclusive_group()
    selection.add_argument('--mode', action='append', choices=MODES)
    selection.add_argument('--modes', nargs='+', choices=MODES)
    cli.add_argument('--thresholds', type=Path, default=Path(__file__).resolve().parent / 'verification/motion-quality-thresholds-v2.json')
    return cli


def main():
    cli = build_parser()
    args = cli.parse_args()
    if args.out.exists():
        cli.error('Use a fresh output directory; prior native probe evidence must remain immutable')
    if args.target_speed <= 0 or args.acceleration_timeout <= 0 or args.braking_timeout <= 0:
        cli.error('Speed and timeouts must be positive')
    args.out.mkdir(parents=True)
    thresholds = load_thresholds(args.thresholds)
    if thresholds.derivative_window_seconds != .5:
        cli.error('Probe must use the same fixed 0.5s motion-auditor window')
    modes = args.mode or args.modes or ['fixed_0.01', 'fixed_0.03', 'fixed_0.05', 'fixed_0.08', 'smooth_cap']
    if len(modes) != len(set(modes)):
        cli.error('Each independent mode may be selected once per fresh output directory')
    (args.out / 'executed-brake_physics_probe.py').write_bytes(Path(__file__).read_bytes())
    client = carla.Client(args.host, args.port)
    client.set_timeout(120.)
    metadata = {'diagnostic_only': True, 'accepted_capture': False,
        'script_sha256': sha(Path(__file__)), 'derivative_auditor_sha256': sha(Path(__file__).parent / 'verification/audit_motion_quality.py'),
        'thresholds_sha256': sha(args.thresholds), 'client_version': client.get_client_version(),
        'server_version': client.get_server_version(), 'modes': modes, 'target_speed_mps': args.target_speed,
        'native_hz': 20, 'physics_mutated': False, 'velocity_or_transform_overridden': False,
        'each_trial_new_world_and_native_actor': True,
        'candidate_policy': {'low_speed_threshold_mps': 2.5,
            'coast_only': 'Zero throttle and brake until an actual <=0.1m/s stop, then real brake0.35 hold.',
            'neutral_low': 'Below actual2.5m/s latch manual_gear_shift=True gear0; keep brake0.01. Actual native gear is recorded.',
            'tiny_throttle_low': 'Below actual2.5m/s ramp throttle0..0.02 over0.5s while brake0.01 remains; no state overwrite.',
            'tapered_coast': 'At actual2.8..2.5m/s taper brake0.01 to exactly0; latch zero brake below actual2.5, throttle0.',
            'tapered_tiny_drive': 'Same exact-zero brake latch. Below actual2.5m/s throttle=0.02*min(low_elapsed/0.5,1)*max(1-low_elapsed/6,0)*min(speed/2.5,1); actual speed/time feedback, no state overwrite.',
            'continuous_drivetrain_response_not_assumed': True},
        'start_monotonic_s': time.monotonic(), 'argc_parameters': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}}
    (args.out / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    if metadata['client_version'] != metadata['server_version']:
        raise RuntimeError('Native client/server versions differ')
    results = [one_trial(client, mode, args, thresholds) for mode in modes]
    report = {'metadata': metadata, 'trials': [{k: q.get(k) for k in ('mode', 'samples', 'termination_reason',
        'target_reached', 'true_speed_stop_observed', 'cleanup_complete', 'error',
        'braking_candidate_meets_project_motion_targets', 'actual_derivative_metrics', 'frames_sha256')} for q in results],
        'runtime_diagnostic_finished': all(q['error'] is None and q['cleanup_complete'] for q in results),
        'accepted_capture': False, 'end_monotonic_s': time.monotonic()}
    (args.out / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'out': str(args.out), 'finished': report['runtime_diagnostic_finished'],
        'candidates': [q['mode'] for q in results if q['braking_candidate_meets_project_motion_targets']]}), flush=True)
    if not report['runtime_diagnostic_finished']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
