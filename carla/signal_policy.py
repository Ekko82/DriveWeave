"""Hold a verified native opposing green axis; the caller owns every tick.

Integration:
    session = setup_signal_policy(world, plan.get('signal_policy'), carla_api=carla)
    # after EACH owner-controlled warmup/capture tick, before accepting samples:
    if session: signal_diagnostics = session.observe(world.get_snapshot())
    # in finally, before unloading the map:
    if session: session.restore()
    # persist session.evidence, including observations and restoration requests.

CARLA 0.9.16 TrafficLight.freeze() calls FreezeAllTrafficLights. By default this
module never calls it. Selected native state durations extend beyond the capture
horizon, then exact native states are checked every frame. This is a duration
hold, not a claim that is_frozen becomes True. Other junctions retain their own
timers. All geometry is validated before any light is modified. No Client,
world.tick, wait_for_tick, vehicle state mutation, or TM ignore-lights call.
An explicit freeze_world_lights=True plus owned_fresh_world=True opts into the
documented WORLD-WIDE freeze in a freshly created owner-exclusive simulation.
That branch snapshots every light and unfreezes/restores all of them in cleanup.

Official API/source:
https://carla.readthedocs.io/en/0.9.16/python_api/#carlatrafficlight
https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/client/TrafficLight.cpp
"""
from __future__ import annotations

import hashlib
import importlib
import math
from copy import deepcopy


class SignalPolicyError(RuntimeError):
    pass


def town05_wide_left_signal_config():
    """Native junction1126 lane identities; runtime stop evidence remains required."""
    return {
        'enable': True, 'map': 'Town05', 'junction_id': 1126,
        'mode': 'permissive_opposing_axis_duration_hold',
        'junction_center_xy_m': [-50.69066619873047, -92.60450744628906],
        'green_approaches': [
            {'role': 'ego_westbound', 'road_id': 6, 'lane_ids': [-1, -2], 'yaw_deg': -179.9136962890625},
            {'role': 'opposing_eastbound', 'road_id': 7, 'lane_ids': [1, 2], 'yaw_deg': 0.0863037109375}],
        'red_approaches': [
            {'role': 'cross_from_south', 'road_id': 46, 'lane_ids': [-1, -2], 'yaw_deg': 90.0},
            {'role': 'cross_from_north', 'road_id': 47, 'lane_ids': [1, 2], 'yaw_deg': 269.6625061035156}],
        'max_stop_distance_m': 40.0, 'max_predecessor_trace_m': 60.0,
        'max_heading_error_deg': 20.0, 'hold_seconds': 120.0,
        'freeze_world_lights': False, 'owned_fresh_world': False,
        'no_protected_left_arrow_claim': True,
    }


def _angle(a, b):
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _wp_record(w):
    p = w.transform.location
    return {
        'road_id': int(w.road_id), 'lane_id': int(w.lane_id), 's_m': float(w.s),
        'xyz_m': [float(p.x), float(p.y), float(p.z)],
        'yaw_deg': float(w.transform.rotation.yaw), 'is_junction': bool(w.is_junction),
        'junction_id': int(w.get_junction().id) if w.is_junction else None,
    }


def _incoming_reference(w, junction_id, maximum_m):
    """Resolve an internal native stop point to its actual native predecessor."""
    original = w
    distance = 0.0
    while w.is_junction:
        if int(w.get_junction().id) != junction_id:
            raise SignalPolicyError('Stop point belongs to another native junction')
        choices = list(w.previous(0.5))
        if len(choices) != 1:
            raise SignalPolicyError('Ambiguous native predecessor at a light stop point')
        w = choices[0]
        distance += 0.5
        if distance > maximum_m:
            raise SignalPolicyError('Stop point has no nearby external incoming predecessor')
    if not original.is_junction:
        forward = w
        for _ in range(math.ceil(maximum_m / 0.5)):
            if forward.is_junction:
                if int(forward.get_junction().id) != junction_id:
                    raise SignalPolicyError('Native stop lane enters a different junction')
                break
            choices = list(forward.next(0.5))
            if not choices:
                raise SignalPolicyError('Native light stop lane terminates before the junction')
            junction_choices = [q for q in choices if q.is_junction]
            if junction_choices:
                ids = {int(q.get_junction().id) for q in junction_choices}
                if ids != {junction_id}:
                    raise SignalPolicyError('Native stop lane has an ambiguous junction entry')
                break
            if len(choices) != 1:
                raise SignalPolicyError('Native incoming lane branches before its junction')
            forward = choices[0]
        else:
            raise SignalPolicyError('No target junction ahead of native light stop lane')
    return w, distance


def _original(light):
    return {
        'state': light.get_state(), 'state_name': str(light.get_state()),
        'green_s': float(light.get_green_time()),
        'yellow_s': float(light.get_yellow_time()),
        'red_s': float(light.get_red_time()),
        'elapsed_s': float(light.get_elapsed_time()),
        'is_frozen': bool(light.is_frozen()),
        'actor_state': str(light.actor_state),
        'is_active': bool(light.is_active), 'is_dormant': bool(light.is_dormant),
    }


class SignalPolicySession:
    def __init__(self, selections, originals, config, evidence, carla_api, world_lights):
        self._selections = selections
        self._originals = originals
        self._config = config
        self._carla = carla_api
        self._world_lights = world_lights
        self._freeze_world = config.get('freeze_world_lights') is True
        self._restored = False
        self._first_time = None
        self._last_frame = None
        self.evidence = evidence
        self.evidence['observations'] = []
        self.evidence['settling_observations'] = []
        self.evidence['settling_consecutive_pass_frames'] = 0
        self.evidence['settling_ready'] = False
        self.evidence['restoration'] = {'requested': False, 'requests_completed': False}
        self.actor_ids = [int(row['actor'].id) for row in selections]

    def _apply(self):
        if self._freeze_world:
            # CARLA's official API deliberately freezes the WHOLE episode.
            self._world_lights[0].freeze(True)
            self.evidence['global_freeze_called'] = True
        # Official set_state is asynchronous. Send each actor's final state
        # once, cross-axis Reds before the requested opposing Greens. Actual
        # native snapshot states, rather than RPC requests, decide acceptance.
        for row in self._selections:
            light = row['actor']
            old = self._originals[int(light.id)]
            hold = max(float(self._config['hold_seconds']) + old['elapsed_s'] + 60.0,
                       old['green_s'], old['yellow_s'], old['red_s'])
            row['duration_hold_s'] = hold
            light.set_green_time(hold)
            light.set_yellow_time(hold)
            light.set_red_time(hold)
        for row in self._selections:
            if row['desired_state'] == 'Red':
                row['actor'].set_state(self._carla.TrafficLightState.Red)
        for row in self._selections:
            if row['desired_state'] == 'Green':
                row['actor'].set_state(self._carla.TrafficLightState.Green)
        self.evidence['modified_actor_ids'] = self.actor_ids
        self.evidence['runtime_validated'] = False
        self.evidence['duration_holds_s'] = {
            str(row['actor'].id): row['duration_hold_s'] for row in self._selections}

    def _observe_native(self, snapshot, *, settling):
        if self._restored:
            raise SignalPolicyError('Cannot observe a restored signal policy')
        if settling and self.evidence['observations']:
            raise SignalPolicyError('Initialization settling cannot resume after capture verification begins')
        frame = int(snapshot.frame)
        time = float(snapshot.timestamp.elapsed_seconds)
        if self._last_frame is not None and frame <= self._last_frame:
            raise SignalPolicyError('Signal observations must follow strictly increasing native frames')
        if self._first_time is None:
            self._first_time = time
        if time - self._first_time >= float(self._config['hold_seconds']):
            raise SignalPolicyError('Signal hold horizon exceeded; re-plan capture duration')
        rows, errors = [], []
        for row in self._selections:
            light = row['actor']
            state = light.get_state()
            desired = getattr(self._carla.TrafficLightState, row['desired_state'])
            frozen = bool(light.is_frozen())
            durations = [float(light.get_green_time()), float(light.get_yellow_time()),
                         float(light.get_red_time())]
            item = {'actor_id': int(light.id), 'native_state': str(state),
                    'desired_state': row['desired_state'], 'is_frozen': frozen,
                    'actor_state': str(light.actor_state),
                    'is_active': bool(light.is_active), 'is_dormant': bool(light.is_dormant),
                    'elapsed_s': float(light.get_elapsed_time()),
                    'green_yellow_red_s': durations, 'approach_roles': row['roles']}
            rows.append(item)
            if not light.is_active or light.is_dormant:
                errors.append(f'light {light.id}: native signal actor is not active')
            if state != desired:
                errors.append(f'light {light.id}: actual {state}, required {desired}')
            required_frozen = True if self._freeze_world else self._originals[int(light.id)]['is_frozen']
            if frozen != required_frozen:
                errors.append(f'light {light.id}: unexpected freeze-state change')
            if any(abs(v - row['duration_hold_s']) > 0.05 for v in durations):
                errors.append(f'light {light.id}: native duration changed during capture')
        unrelated = []
        if self._freeze_world:
            selected_ids = set(self.actor_ids)
            for light in self._world_lights:
                if int(light.id) in selected_ids:
                    continue
                old = self._originals[int(light.id)]
                state, frozen = light.get_state(), bool(light.is_frozen())
                unrelated.append({'actor_id': int(light.id), 'state': str(state), 'is_frozen': frozen})
                if state != old['state'] or not frozen:
                    errors.append(f'world-frozen unrelated light {light.id} did not retain its native state')
        result = {'native_frame': frame, 'sim_time_s': time, 'lights': rows,
                  'world_frozen_unrelated_lights': unrelated, 'pass': not errors, 'errors': errors}
        self._last_frame = frame
        if settling:
            result['phase'] = 'native_initialization_settling_not_accepted_capture'
            self.evidence['settling_observations'].append(result)
            self.evidence['settling_consecutive_pass_frames'] = (
                self.evidence['settling_consecutive_pass_frames'] + 1 if not errors else 0)
            self.evidence['settling_ready'] = self.evidence['settling_consecutive_pass_frames'] >= 5
            result['consecutive_pass_frames'] = self.evidence['settling_consecutive_pass_frames']
            result['settling_ready'] = self.evidence['settling_ready']
            return result
        self.evidence['observations'].append(result)
        if errors:
            self.evidence['runtime_validated'] = False
            raise SignalPolicyError('Native light policy mismatch: ' + '; '.join(errors))
        self.evidence['runtime_validated'] = True
        self.evidence['observed_native_frames'] = len(self.evidence['observations'])
        return result

    def observe(self, snapshot):
        """Read actual warmup/capture states; every mismatch still raises."""
        return self._observe_native(snapshot, settling=False)

    def settling_observe(self, snapshot):
        """Record owner-ticked initialization states, including real mismatches.

        Never ticks or sends state commands. The owner must cap setup to 20
        native ticks and require five consecutive passes before strict warmup.
        These frames remain separate from accepted capture observations.
        """
        return self._observe_native(snapshot, settling=True)

    update = observe

    def restore(self):
        """Restore native state/durations and undo an explicitly enabled global freeze."""
        if self._restored:
            return self.evidence['restoration']
        result = {'requested': True, 'requests_completed': False,
                  'original_elapsed_phase_cannot_be_restored_by_public_api': True,
                  'actors': [], 'errors': []}
        # Original elapsed times are recorded, but CARLA has no setter for them.
        restore_lights = self._world_lights if self._freeze_world else [q['actor'] for q in self._selections]
        if self._freeze_world:
            try:
                self._world_lights[0].freeze(False)
                result['global_unfreeze_requested'] = True
            except Exception as exc:
                result['errors'].append({'action': 'global_unfreeze', 'error': repr(exc)})
        for light in restore_lights:
            old = self._originals[int(light.id)]
            try:
                light.set_green_time(old['green_s'])
                light.set_yellow_time(old['yellow_s'])
                light.set_red_time(old['red_s'])
                light.set_state(old['state'])
                result['actors'].append(int(light.id))
            except Exception as exc:
                result['errors'].append({'actor_id': int(light.id), 'error': repr(exc)})
        self._restored = True
        result['requests_completed'] = not result['errors']
        self.evidence['restoration'] = result
        if result['errors']:
            raise SignalPolicyError('Some native light restoration requests failed: ' + repr(result['errors']))
        return result


def setup_signal_policy(world, config, *, carla_api=None):
    """Return None unless explicitly enabled; validate actual native light lanes."""
    if config is None or config.get('enable') is not True:
        return None
    config = deepcopy(config)
    api = carla_api or importlib.import_module('carla')
    if config.get('mode') != 'permissive_opposing_axis_duration_hold':
        raise SignalPolicyError('Unsupported signal policy mode')
    carla_map = world.get_map()
    expected_map = str(config['map'])
    if carla_map.name.rsplit('/', 1)[-1] != expected_map:
        raise SignalPolicyError('Actual native map does not match signal plan')
    center = list(map(float, config['junction_center_xy_m']))
    junction = int(config['junction_id'])
    maximum = float(config.get('max_stop_distance_m', 40.0))
    resolution = float(config.get('max_predecessor_trace_m', 60.0))
    if len(center) != 2 or not all(math.isfinite(v) for v in center):
        raise SignalPolicyError('Invalid native junction center')
    hold = float(config['hold_seconds'])
    if not math.isfinite(hold) or hold <= 0:
        raise SignalPolicyError('Invalid signal hold duration')
    freeze_world = config.get('freeze_world_lights') is True
    if freeze_world and config.get('owned_fresh_world') is not True:
        raise SignalPolicyError('World-wide freeze requires an explicitly owner-exclusive fresh world')
    approaches, required = {}, set()
    roles_by_state = {'Green': set(), 'Red': set()}
    for name, state in [('green_approaches', 'Green'), ('red_approaches', 'Red')]:
        for item in config[name]:
            role = str(item['role'])
            roles_by_state[state].add(role)
            for lane in item['lane_ids']:
                key = (int(item['road_id']), int(lane))
                if key in approaches:
                    raise SignalPolicyError('An incoming native lane has conflicting policies')
                approaches[key] = {'state': state, 'role': role,
                                   'yaw_deg': float(item['yaw_deg'])}
                required.add(key)
    if len(roles_by_state['Green']) < 2 or not roles_by_state['Red']:
        raise SignalPolicyError('Opposing green and crossing red approaches are both required')
    greens = [float(x['yaw_deg']) for x in config['green_approaches']]
    if len(greens) != 2 or _angle(greens[0], greens[1]) < 160:
        raise SignalPolicyError('Green approaches are not true opposite directions')
    for item in config['red_approaches']:
        if min(_angle(item['yaw_deg'], g) for g in greens) < 60:
            raise SignalPolicyError('A red approach is not transverse to the opposing green axis')

    candidates = list(world.get_actors().filter('traffic.traffic_light*'))
    selections, covered, by_id = [], set(), {}
    diagnostics = []
    for light in candidates:
        stop_points = list(light.get_stop_waypoints())
        if not stop_points:
            continue
        close = [w for w in stop_points if math.hypot(
            w.transform.location.x - center[0], w.transform.location.y - center[1]) <= maximum]
        if not close:
            continue
        rows, states, roles, unresolved = [], set(), set(), []
        for point in stop_points:
            try:
                external, back_m = _incoming_reference(point, junction, resolution)
                key = (int(external.road_id), int(external.lane_id))
                selected = approaches.get(key)
                if selected is None:
                    raise SignalPolicyError(f'Unknown incoming lane {key}')
                error = _angle(external.transform.rotation.yaw, selected['yaw_deg'])
                if error > float(config.get('max_heading_error_deg', 20.0)):
                    raise SignalPolicyError(f'Incoming direction mismatch {key}: {error:.3f} degrees')
                if math.hypot(point.transform.location.x - center[0],
                              point.transform.location.y - center[1]) > maximum:
                    raise SignalPolicyError('The same native light also controls a distant junction')
                rows.append({'native_stop_waypoint': _wp_record(point),
                             'resolved_incoming_waypoint': _wp_record(external),
                             'predecessor_trace_m': back_m, 'heading_error_deg': error,
                             'role': selected['role'], 'desired_state': selected['state']})
                states.add(selected['state']); roles.add(selected['role'])
                covered.add(key)
            except SignalPolicyError as exc:
                unresolved.append({'native_stop_waypoint': _wp_record(point), 'reason': str(exc)})
        diagnostics.append({'actor_id': int(light.id), 'resolved_stop_points': rows,
                            'unresolved_stop_points': unresolved})
        if unresolved:
            raise SignalPolicyError('Cannot classify every stop lane of nearby native light ' +
                                    str(light.id) + ': ' + repr(unresolved))
        if len(states) != 1:
            raise SignalPolicyError('One native light affects incompatible green/red approaches')
        row = {'actor': light, 'desired_state': states.pop(), 'roles': sorted(roles),
               'native_stop_evidence': rows}
        selections.append(row); by_id[int(light.id)] = row
    missing = sorted(required - covered)
    if missing:
        raise SignalPolicyError('Native traffic-light stop evidence does not cover required lanes: ' + repr(missing))
    if not selections or not any(q['desired_state'] == 'Red' for q in selections):
        raise SignalPolicyError('No verified transverse red native lights')
    # A native scheduler group must be wholly classified before any mutation.
    for row in selections:
        ids = {int(q.id) for q in row['actor'].get_group_traffic_lights()}
        if not ids or not ids.issubset(by_id):
            raise SignalPolicyError('Native light group contains unmatched signal actors: ' +
                                    repr(sorted(ids - set(by_id))))
    save_lights = candidates if freeze_world else [q['actor'] for q in selections]
    originals = {int(light.id): _original(light) for light in save_lights}
    if any(q['is_frozen'] for q in originals.values()):
        raise SignalPolicyError('Native selected lights were already frozen; cannot promise isolated timing hold')
    evidence = {
        'mode': config['mode'], 'map': carla_map.name, 'junction_id': junction,
        'native_map_xodr_sha256': hashlib.sha256(carla_map.to_opendrive().encode('utf-8')).hexdigest(),
        'config': config, 'geometry_validated_before_mutation': True,
        'used_native_stop_waypoints': True, 'used_native_predecessors': True,
        'global_freeze_called': False, 'timer_freeze_claimed': freeze_world,
        'global_freeze_opt_in': freeze_world,
        'global_freeze_scope': 'ALL native traffic lights in the owner-exclusive fresh episode' if freeze_world else None,
        'other_junction_lights_modified': freeze_world, 'no_protected_left_arrow_claim': True,
        'runtime_actions_verified': False, 'native_signal_geometry': diagnostics,
        'originals': {str(k): {f: v for f, v in q.items() if f != 'state'} for k, q in originals.items()},
        'desired_states': {str(row['actor'].id): row['desired_state'] for row in selections},
        'restore_limitation': 'Public API can restore state/durations but not original elapsed phase.',
    }
    session = SignalPolicySession(selections, originals, config, evidence, api, candidates)
    try:
        session._apply()
    except Exception:
        session.restore()
        raise
    return session


setup = setup_signal_policy
