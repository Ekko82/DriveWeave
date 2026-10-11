"""Seeded native CARLA traffic; the caller exclusively owns simulation stepping.

Integration (after explicit scenario actors exist, before physical preroll):
    session = setup_dynamic_traffic(world, tm, ego, config, carla_api=carla)
    owned_actors.extend(session.actors)     # capture + cleanup must include these
    # Each owner-controlled step, including preroll:
    traffic_diagnostics = session.update(world.get_snapshot())
    # Then the owner applies ego controls and performs its ONLY world/sync tick.

No Client, tick, actor relocation, forced lane change, target velocity, dormant
respawn, collision disabling or hybrid physics is used here. This controls only
its newly spawned background actors. Exact conflict opponents stay under the
scenario's own route controller; a TM route is not proof of a junction maneuver.

API basis: CARLA 0.9.16 Python TrafficManager API and adv_traffic_manager docs.
distance_to_leading_vehicle is a front-to-back gap (global distance is different).
set_desired_speed is km/h; CARLA vehicle.get_speed_limit() also returns km/h.
Determinism requires identical synchronous initial state, seed and tick schedule.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
import importlib
import math
import random
from typing import Any


DEFAULT_BLUEPRINTS = (
    'vehicle.audi.a2', 'vehicle.audi.tt', 'vehicle.tesla.model3',
    'vehicle.nissan.patrol', 'vehicle.mercedes.coupe_2020',
    'vehicle.lincoln.mkz_2020', 'vehicle.dodge.charger_2020',
    'vehicle.carlamotors.carlacola',
)


@dataclass(frozen=True)
class TrafficConfig:
    count: int = 24
    seed: int = 7
    # Explicit native corridors are preferred. s_start <= s_end; both lane signs
    # are valid. Example {road_id:40,lane_ids:[-4,-3,-2,-1,4,5],s_start:8,
    # s_end:250,spacing_m:14}; actual lane width/type/junctions are checked.
    corridors: tuple[dict, ...] = ()
    # Optional native paths spanning the full scenario + physical preroll.
    # These replace random-junction spawn selection and disable lane changes.
    native_routes: tuple[dict, ...] = ()
    spawn_indices: tuple[int, ...] = ()
    focus_points: tuple[tuple[float, float, float], ...] = ()
    focus_radius_m: float = 100.0
    # Keep exact scenario entry/merge/conflict areas clear at INITIAL spawn.
    exclude_xodr: tuple[dict, ...] = ()
    exclude_circles: tuple[dict, ...] = ()
    lane_change_exclude_xodr: tuple[dict, ...] = ()
    blueprints: tuple[str, ...] = DEFAULT_BLUEPRINTS
    speed_range_kmh: tuple[float, float] = (36.0, 66.0)
    speed_limit_ratio_range: tuple[float, float] = (.76, .94)
    time_headway_range_s: tuple[float, float] = (1.4, 2.1)
    standstill_gap_range_m: tuple[float, float] = (3.5, 5.0)
    spawn_time_headway_s: float = 1.5
    min_spawn_body_gap_m: float = 5.0
    lateral_body_clearance_m: float = .75
    # Conservative envelope covers the curated cars/SUV/carlacola van.
    max_half_length_m: float = 3.2
    max_half_width_m: float = 1.45
    elevation_separation_m: float = 4.0
    spawn_z_offset_m: float = .3
    lane_change_probability_range_pct: tuple[float, float] = (.06, .20)
    ego_lane_change_protection_m: float = 18.0
    lane_change_cooldown_s: float = 4.0
    update_interval_s: float = .20
    speed_recovery_acceleration_mps2: float = 1.2
    role_prefix: str = 'dynamic_bg'
    # Do not silently claim the requested density when native space is scarce.
    require_count: bool = False

    @classmethod
    def from_mapping(cls, config: dict | TrafficConfig | None) -> TrafficConfig:
        if isinstance(config, cls):
            return config
        raw = dict(config or {})
        sequence_keys = ('corridors', 'native_routes', 'spawn_indices', 'focus_points',
                         'exclude_xodr', 'exclude_circles', 'lane_change_exclude_xodr', 'blueprints',
                         'speed_range_kmh', 'speed_limit_ratio_range',
                         'time_headway_range_s', 'standstill_gap_range_m',
                         'lane_change_probability_range_pct')
        for key in sequence_keys:
            if key in raw:
                raw[key] = tuple(raw[key])
        obj = cls(**raw)
        if obj.count < 0 or obj.update_interval_s <= 0 or obj.focus_radius_m <= 0:
            raise ValueError('Invalid traffic count, update interval or focus radius')
        for key in sequence_keys[-5:]:
            interval = getattr(obj, key)
            if len(interval) != 2 or not 0 <= interval[0] <= interval[1]:
                raise ValueError(f'Invalid {key}')
        if not 0 < obj.speed_limit_ratio_range[0] <= obj.speed_limit_ratio_range[1] <= 1:
            raise ValueError('Speed-limit ratios must be in (0,1]')
        if obj.time_headway_range_s[0] <= 0:
            raise ValueError('Time headway must be positive')
        for key in ('spawn_time_headway_s', 'min_spawn_body_gap_m',
                    'lateral_body_clearance_m', 'spawn_z_offset_m',
                    'speed_recovery_acceleration_mps2'):
            if getattr(obj, key) < 0:
                raise ValueError(f'{key} must be nonnegative')
        if obj.max_half_length_m <= 0 or obj.max_half_width_m <= 0:
            raise ValueError('Vehicle envelope must be positive')
        if obj.lane_change_probability_range_pct[1] > 100:
            raise ValueError('Lane change probability must be <=100')
        if obj.native_routes:
            if obj.corridors or obj.spawn_indices:
                raise ValueError('Native routes replace corridor/spawn-index selection')
            if obj.count > len(obj.native_routes):
                raise ValueError('Native route traffic may not fall back to unplanned actors')
            ids = [route['route_id'] for route in obj.native_routes]
            if len(set(ids)) != len(ids):
                raise ValueError('Native route IDs must be unique')
        return obj


@dataclass(frozen=True)
class Footprint:
    x: float
    y: float
    z: float
    yaw: float
    half_length: float
    half_width: float
    road_id: int | None = None
    lane_id: int | None = None
    s_m: float | None = None
    speed_mps: float = 0.0
    actor_id: int | None = None


def _vertices(body: Footprint) -> list[tuple[float, float]]:
    c, s = math.cos(math.radians(body.yaw)), math.sin(math.radians(body.yaw))
    return [(body.x + c*x - s*y, body.y + s*x + c*y)
            for x, y in ((-body.half_length, -body.half_width),
                         (body.half_length, -body.half_width),
                         (body.half_length, body.half_width),
                         (-body.half_length, body.half_width))]


def _point_segment(p, a, b) -> float:
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy) /
                       max(1e-12, dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def horizontal_body_clearance(a: Footprint, b: Footprint) -> float:
    """Exact distance between yaw-oriented footprint polygons; 0 if overlapping.

    Spawn guard only. Post-run safety needs the project's separate full 3D OBB
    auditor (pitch, roll, bbox offsets and all sampled frames).
    """
    pa, pb = _vertices(a), _vertices(b)
    separated = False
    for yaw in (a.yaw, b.yaw):
        for angle in (math.radians(yaw), math.radians(yaw)+math.pi/2):
            axis = math.cos(angle), math.sin(angle)
            va = [x*axis[0]+y*axis[1] for x, y in pa]
            vb = [x*axis[0]+y*axis[1] for x, y in pb]
            if max(va) < min(vb) or max(vb) < min(va):
                separated = True
    if not separated:
        return 0.
    return min(_point_segment(p, q[i], q[(i+1)%4])
               for pset, q in ((pa, pb), (pb, pa)) for p in pset for i in range(4))


def spawn_rejection(candidate: Footprint, occupied: list[Footprint],
                    config: TrafficConfig) -> str | None:
    for other in occupied:
        if abs(candidate.z-other.z) >= config.elevation_separation_m:
            continue
        clearance = horizontal_body_clearance(candidate, other)
        same_lane = (candidate.road_id is not None and
                     candidate.road_id == other.road_id and
                     candidate.lane_id == other.lane_id)
        required = config.lateral_body_clearance_m
        if same_lane:
            required = config.min_spawn_body_gap_m + config.spawn_time_headway_s * max(
                candidate.speed_mps, other.speed_mps)
        if clearance < required:
            return 'same_lane_time_gap' if same_lane else 'body_clearance'
    for region in config.exclude_xodr:
        lanes = region.get('lane_ids', [region.get('lane_id')])
        if candidate.road_id == int(region['road_id']) and candidate.lane_id in lanes:
            if float(region.get('s_start', 0)) <= candidate.s_m <= float(region['s_end']):
                return 'reserved_native_corridor'
    for region in config.exclude_circles:
        if (abs(candidate.z-float(region.get('z', candidate.z))) < config.elevation_separation_m
                and math.hypot(candidate.x-float(region['x']), candidate.y-float(region['y']))
                < float(region['radius_m']) + math.hypot(candidate.half_length, candidate.half_width)):
            return 'reserved_world_area'
    return None


def _driving(wp) -> bool:
    return wp is not None and str(wp.lane_type).split('.')[-1] == 'Driving'


def _native_key(wp) -> tuple:
    return int(wp.road_id), int(wp.section_id), int(wp.lane_id), round(float(wp.s), 2)


def _continuous_lane_change_corridor(wp, distance_m: float) -> bool:
    """Keep autonomous changes away from junctions or disappearing native lanes."""
    if wp is None or wp.is_junction:
        return False
    initial = (wp.road_id, wp.lane_id)
    current = wp
    for _ in range(math.ceil(distance_m/5)):
        following = [p for p in current.next(5.) if not p.is_junction and
                     (p.road_id, p.lane_id) == initial and _driving(p)]
        if len(following) != 1:
            return False
        current = following[0]
    return True


def build_spawn_candidates(road_map, config: TrafficConfig, carla_api, ego_location=None):
    """Resolve only existing native waypoints, then deduplicate independently of input order."""
    candidates = {}
    sources = []
    for route in config.native_routes:
        start = route['spawn_xodr']
        sources.append(road_map.get_waypoint_xodr(int(start['road_id']),
            int(start['lane_id']), float(start['s_m'])))
    for corridor in config.corridors:
        start, end = float(corridor['s_start']), float(corridor['s_end'])
        spacing = float(corridor.get('spacing_m', 12))
        if spacing <= 0 or end < start:
            raise ValueError('Corridor spacing must be positive and s_end >= s_start')
        for lane in corridor['lane_ids']:
            for index in range(int(math.floor((end-start)/spacing))+1):
                wp = road_map.get_waypoint_xodr(int(corridor['road_id']), int(lane), start+index*spacing)
                sources.append(wp)
    if not config.native_routes and (not config.corridors or config.spawn_indices):
        spawns = road_map.get_spawn_points()
        indexes = config.spawn_indices if config.spawn_indices else range(len(spawns))
        for index in indexes:
            if not 0 <= int(index) < len(spawns):
                raise ValueError(f'Invalid native spawn index {index}')
            sources.append(road_map.get_waypoint(spawns[int(index)].location))
    focus = list(config.focus_points)
    if not focus and not config.corridors and not config.native_routes and ego_location is not None:
        focus = [(ego_location.x, ego_location.y, ego_location.z)]
    for wp in sources:
        if not _driving(wp) or wp.is_junction or wp.lane_width < 2*config.max_half_width_m+.10:
            continue
        pose = wp.transform
        loc = pose.location
        if focus and min(math.dist((loc.x, loc.y, loc.z), p) for p in focus) > config.focus_radius_m:
            continue
        # Fresh transform: never change a waypoint or a user-owned spawn pose.
        transform = carla_api.Transform(carla_api.Location(x=loc.x, y=loc.y,
            z=loc.z+config.spawn_z_offset_m), carla_api.Rotation(
                pitch=pose.rotation.pitch, yaw=pose.rotation.yaw, roll=pose.rotation.roll))
        candidates[_native_key(wp)] = (wp, transform)
    return [candidates[key] for key in sorted(candidates)]


def _actor_footprint(actor, road_map, snapshot=None, spawn_pose=None) -> Footprint | None:
    state = snapshot.find(actor.id) if snapshot is not None else actor
    if state is None:
        return None
    pose, velocity = spawn_pose or state.get_transform(), state.get_velocity()
    bbox = actor.bounding_box
    # Transform.transform mutates its Location argument in CARLA's binding.
    # Multiply a copied coordinate instead of touching the bbox local offset.
    matrix = pose.get_matrix()
    local = (bbox.location.x, bbox.location.y, bbox.location.z)
    center = [sum(matrix[i][j]*local[j] for j in range(3))+matrix[i][3] for i in range(3)]
    wp = road_map.get_waypoint(pose.location)
    return Footprint(*center, pose.rotation.yaw+bbox.rotation.yaw,
        bbox.extent.x, bbox.extent.y, wp.road_id if wp else None,
        wp.lane_id if wp else None, wp.s if wp else None,
        math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2), actor.id)


def sample_driver_profile(rng: random.Random, config: TrafficConfig) -> dict:
    return {'cruise_cap_kmh': rng.uniform(*config.speed_range_kmh),
            'speed_limit_ratio': rng.uniform(*config.speed_limit_ratio_range),
            'time_headway_s': rng.uniform(*config.time_headway_range_s),
            'standstill_gap_m': rng.uniform(*config.standstill_gap_range_m),
            'lane_change_probability_pct': rng.uniform(*config.lane_change_probability_range_pct)}


def _desired_cruise(actor, profile) -> float:
    limit = float(actor.get_speed_limit())
    # A map without a positive limit receives a conservative 30 km/h cap.
    return min(profile['cruise_cap_kmh'], (limit if limit > 0 else 30.) * profile['speed_limit_ratio'])


def configure_actor(tm, actor, profile: dict, tm_port: int):
    actor.set_autopilot(True, tm_port)
    tm.ignore_lights_percentage(actor, 0.)
    tm.ignore_signs_percentage(actor, 0.)
    tm.ignore_vehicles_percentage(actor, 0.)
    tm.ignore_walkers_percentage(actor, 0.)
    # Native snapshot omits actors just created until the owner steps once.
    # The first update with an observed state enables only eligible sides.
    tm.auto_lane_change(actor, False)
    tm.update_vehicle_lights(actor, True)
    tm.distance_to_leading_vehicle(actor, profile['standstill_gap_m'])
    tm.set_desired_speed(actor, _desired_cruise(actor, profile))
    tm.random_left_lanechange_percentage(actor, 0.)
    tm.random_right_lanechange_percentage(actor, 0.)


@dataclass
class TrafficSession:
    world: Any
    tm: Any
    ego: Any
    road_map: Any
    config: TrafficConfig
    actors: list = field(default_factory=list)
    profiles: dict[int, dict] = field(default_factory=dict)
    evidence: dict = field(default_factory=dict)
    _last_update: float | None = None
    _states: dict[int, dict] = field(default_factory=dict)
    native_paths: dict[int, dict] = field(default_factory=dict)

    def update(self, snapshot, observed_actors=None) -> dict:
        """Adjust TM goals from measured states before the caller's single tick.

        Time gaps, gradual recovery, native marking permissions and nearby ego
        protection complement TM's own physical collision/traffic-light planner.
        Return diagnostics for capture evidence; these are intentions, not proof
        that any vehicle maneuver actually happened.
        """
        now = float(snapshot.timestamp.elapsed_seconds)
        if self._last_update is not None and now < self._last_update:
            raise ValueError('Snapshot time moved backwards')
        if self._last_update is not None and now-self._last_update < self.config.update_interval_s-1e-6:
            return {'frame': int(snapshot.frame), 'updated': False}
        dt = self.config.update_interval_s if self._last_update is None else now-self._last_update
        self._last_update = now
        vehicles = observed_actors if observed_actors is not None else self.world.get_actors().filter('vehicle.*')
        footprints = {a.id: _actor_footprint(a, self.road_map, snapshot) for a in vehicles}
        ego_body = footprints.get(self.ego.id)
        rows = []
        for actor in self.actors:
            body = footprints.get(actor.id)
            if body is None:
                rows.append({'actor_id': actor.id, 'missing_from_snapshot': True})
                continue
            profile = self.profiles[actor.id]
            state = self._states.setdefault(actor.id, {'lane': (body.road_id, body.lane_id),
                'last_lane_change_time_s': -1e9, 'desired_kmh': _desired_cruise(actor, profile)})
            lane = body.road_id, body.lane_id
            if lane != state['lane']:
                state.update(lane=lane, last_lane_change_time_s=now)
            c, s = math.cos(math.radians(body.yaw)), math.sin(math.radians(body.yaw))
            desired = _desired_cruise(actor, profile)
            gap_target = profile['standstill_gap_m'] + body.speed_mps*profile['time_headway_s']
            nearest = None
            for other in footprints.values():
                if other is None or other.actor_id == actor.id or abs(other.z-body.z) >= self.config.elevation_separation_m:
                    continue
                dx, dy = other.x-body.x, other.y-body.y
                along, lateral = dx*c+dy*s, abs(-dx*s+dy*c)
                relative_yaw = math.radians(other.yaw-body.yaw)
                other_front_extent = abs(math.cos(relative_yaw))*other.half_length + abs(math.sin(relative_yaw))*other.half_width
                other_side_extent = abs(math.sin(relative_yaw))*other.half_length + abs(math.cos(relative_yaw))*other.half_width
                if along <= 0 or along > 90 or lateral > body.half_width+other_side_extent+.25:
                    continue
                gap = along-body.half_length-other_front_extent
                if nearest is None or gap < nearest['body_gap_m']:
                    other_speed = max(0., other.speed_mps*math.cos(relative_yaw))
                    nearest = {'actor_id': other.actor_id, 'body_gap_m': gap,
                               'projected_speed_mps': other_speed}
            if nearest is not None and nearest['body_gap_m'] < gap_target:
                desired = min(desired, max(0., nearest['projected_speed_mps'] +
                    (nearest['body_gap_m']-gap_target)/profile['time_headway_s'])*3.6)
            desired = min(desired, state['desired_kmh'] + self.config.speed_recovery_acceleration_mps2*dt*3.6)
            pose = snapshot.find(actor.id).get_transform()
            wp = self.road_map.get_waypoint(pose.location)
            permission = str(wp.lane_change).split('.')[-1] if wp else 'None'
            protected = ego_body is not None and abs(ego_body.z-body.z) < self.config.elevation_separation_m and math.hypot(
                body.x-ego_body.x, body.y-ego_body.y) < self.config.ego_lane_change_protection_m
            available = (actor.id not in self.native_paths and not protected and
                now-state['last_lane_change_time_s'] >= self.config.lane_change_cooldown_s and
                _continuous_lane_change_corridor(wp, max(35., body.speed_mps*3+8)))
            sides = {}
            for side in ('left', 'right'):
                neighbor = getattr(wp, 'get_'+side+'_lane')() if wp is not None else None
                same_direction = _driving(neighbor) and neighbor.lane_id*wp.lane_id > 0
                sides[side] = available and same_direction and permission in ('Both', side.title())
                if sides[side]:
                    for region in self.config.lane_change_exclude_xodr:
                        if (neighbor.road_id == int(region['road_id']) and
                            neighbor.lane_id in region['lane_ids'] and
                            float(region.get('s_start',0)) <= neighbor.s <= float(region['s_end'])):
                            sides[side]=False
                    anchor=neighbor.transform.location
                    heading=neighbor.transform.get_forward_vector()
                    for other in footprints.values():
                        if other is None or other.actor_id==actor.id or abs(other.z-body.z)>=self.config.elevation_separation_m:
                            continue
                        ox,oy=other.x-anchor.x,other.y-anchor.y
                        along=ox*heading.x+oy*heading.y
                        lateral=abs(-ox*heading.y+oy*heading.x)
                        if lateral > neighbor.lane_width/2+other.half_width+.5:
                            continue
                        gap=abs(along)-body.half_length-other.half_length
                        other_speed=max(0.,other.speed_mps*math.cos(math.radians(other.yaw-body.yaw)))
                        required=(6.+body.speed_mps*2.) if along>=0 else (6.+other_speed*2.+max(0.,other_speed-body.speed_mps)*4.)
                        if gap<required:sides[side]=False
            tm = self.tm
            tm.distance_to_leading_vehicle(actor, gap_target)
            # Native TM PID can return a NaN brake at an exactly zero desired
            # speed. Its collision planner remains responsible for actual stops.
            tm.set_desired_speed(actor, max(.5,desired))
            # Automatic obstacle-driven changes are enabled only when every
            # existing permitted side is safe, not just a random-side request.
            possible=[side for side in ('left','right') if _driving(getattr(wp,'get_'+side+'_lane')()) and
                getattr(wp,'get_'+side+'_lane')().lane_id*wp.lane_id>0 and permission in ('Both',side.title())] if wp else []
            tm.auto_lane_change(actor, bool(possible) and all(sides[side] for side in possible) and profile['lane_change_probability_pct']>0)
            tm.random_left_lanechange_percentage(actor, profile['lane_change_probability_pct'] if sides['left'] else 0.)
            tm.random_right_lanechange_percentage(actor, profile['lane_change_probability_pct'] if sides['right'] else 0.)
            state['desired_kmh'] = desired
            rows.append({'actor_id': actor.id, 'actual_speed_mps': body.speed_mps,
                'desired_speed_kmh': desired, 'front_to_back_gap_target_m': gap_target,
                'front_vehicle': nearest, 'lane_change_allowed': sides,
                'near_ego_protection': protected, 'actual_native_lane': list(lane),
                'native_route_id':self.native_paths.get(actor.id,{}).get('route_id'),
                'path_tracking_validated':False})
        return {'frame': int(snapshot.frame), 'timestamp_s': now, 'updated': True,
                'actors': rows, 'kind': 'TM_goal_diagnostics_not_action_acceptance'}

    def close(self):
        """Optional owner-invoked cleanup for module actors only; never steps world."""
        errors = []
        for actor in self.actors:
            try:
                if actor.is_alive:
                    actor.set_autopilot(False, self.tm.get_port())
                    actor.destroy()
            except Exception as error:
                errors.append({'actor_id': actor.id, 'error': str(error)})
        return errors


def setup_dynamic_traffic(world, tm, ego, config=None, *, carla_api=None,
                          register_actor=None) -> TrafficSession:
    """Initialize native traffic without stepping; retain session.actors for cleanup.

    Caller has already enabled synchronous world mode. Existing scenario actors
    are automatic spawn obstacles but are never reconfigured or adopted here.
    register_actor(actor), if given, lets capture own each actor immediately.
    """
    cfg = TrafficConfig.from_mapping(config)
    if not world.get_settings().synchronous_mode:
        raise RuntimeError('Owner must enable synchronous CARLA mode first')
    carla_api = carla_api or importlib.import_module('carla')
    road_map = world.get_map()
    session = TrafficSession(world, tm, ego, road_map, cfg)
    route_lookup = {}
    if cfg.native_routes:
        from native_background_routes import resolve_route
        for specification in cfg.native_routes:
            native, proof = resolve_route(road_map, specification)
            key = _native_key(native[0])
            if key in route_lookup:
                raise ValueError('Two native traffic routes share the same spawn')
            route_lookup[key] = (specification, native, proof)
    tm.set_synchronous_mode(True)
    tm.set_random_device_seed(cfg.seed)
    tm.set_hybrid_physics_mode(False)
    tm.set_respawn_dormant_vehicles(False)
    tm_port = int(tm.get_port())
    library = world.get_blueprint_library()
    available = {bp.id for bp in library.filter('vehicle.*')
        if bp.has_attribute('number_of_wheels') and int(bp.get_attribute('number_of_wheels')) == 4}
    blueprints = sorted(set(cfg.blueprints) & available)
    if cfg.count and not blueprints:
        raise RuntimeError('None of the configured native vehicle blueprints exists')
    candidates = build_spawn_candidates(road_map, cfg, carla_api, ego.get_location())
    rng = random.Random(cfg.seed)
    rng.shuffle(candidates)
    occupied = [body for actor in world.get_actors().filter('vehicle.*')
                if (body := _actor_footprint(actor, road_map)) is not None]
    rejects = {}
    records = []
    try:
        for wp, pose in candidates:
            if len(session.actors) >= cfg.count:
                break
            profile = sample_driver_profile(rng, cfg)
            route = route_lookup.get(_native_key(wp))
            if cfg.native_routes and route is None:
                raise RuntimeError('Native route spawn lost its path; random fallback prohibited')
            if route is not None:
                profile['cruise_cap_kmh'] = min(profile['cruise_cap_kmh'],float(route[0]['speed_cap_kmh']))
                profile['lane_change_probability_pct'] = 0.
            probe = Footprint(pose.location.x, pose.location.y, pose.location.z,
                pose.rotation.yaw, cfg.max_half_length_m, cfg.max_half_width_m,
                wp.road_id, wp.lane_id, wp.s, profile['cruise_cap_kmh']/3.6)
            reason = spawn_rejection(probe, occupied, cfg)
            if reason:
                rejects[reason] = rejects.get(reason, 0)+1
                continue
            blueprint = library.find(rng.choice(blueprints))
            blueprint.set_attribute('role_name', f'{cfg.role_prefix}_{len(session.actors):03d}')
            if blueprint.has_attribute('color'):
                colors = sorted(blueprint.get_attribute('color').recommended_values)
                if colors:
                    blueprint.set_attribute('color', rng.choice(colors))
            actor = world.try_spawn_actor(blueprint, pose)
            if actor is None:
                rejects['native_spawn_occupied'] = rejects.get('native_spawn_occupied', 0)+1
                continue
            session.actors.append(actor)
            # CARLA caches a just-created actor transform until the owner ticks.
            # Validate the requested native spawn pose with the real asset bbox;
            # recorded post-tick snapshots provide independent actual positions.
            actual = _actor_footprint(actor, road_map, spawn_pose=pose)
            if actual is None or actual.half_length > cfg.max_half_length_m or actual.half_width > cfg.max_half_width_m:
                raise RuntimeError(f'Native vehicle {actor.type_id} exceeds configured spawn envelope')
            # Measure real bbox as well: envelope center is not a guarantee when
            # a native asset has a nonzero local bbox offset.
            reason = spawn_rejection(actual, occupied, cfg)
            if reason:
                actor.destroy()
                session.actors.remove(actor)
                rejects['actual_bbox_'+reason] = rejects.get('actual_bbox_'+reason, 0)+1
                continue
            if register_actor is not None:
                register_actor(actor)
            session.profiles[actor.id] = profile
            configure_actor(tm, actor, profile, tm_port)
            if route is not None:
                specification, native, proof = route
                tm.set_path(actor, [carla_api.Location(x=w.transform.location.x,
                    y=w.transform.location.y,z=w.transform.location.z) for w in native])
                session.native_paths[actor.id] = {'route_id':specification['route_id'],
                    'offline_native_successor_proof':proof}
            occupied.append(replace(actual, speed_mps=profile['cruise_cap_kmh']/3.6))
            records.append({'id': actor.id, 'blueprint': actor.type_id,
                'role': actor.attributes.get('role_name'), 'native_spawn_key': list(_native_key(wp)),
                'spawn_location_m': [pose.location.x, pose.location.y, pose.location.z],
                'spawn_bbox_from_native_asset': asdict(actual),
                'spawn_pose_basis':'requested native spawn transform before first physics tick',
                'native_route_intention':session.native_paths.get(actor.id),
                'driver_profile': profile})
        session.evidence = {'schema': 'native_dynamic_traffic_v1', 'seed': cfg.seed,
            'requested_count': cfg.count, 'spawned_count': len(session.actors),
            'candidate_count': len(candidates), 'rejections': rejects,
            'config': asdict(cfg), 'actors': records, 'tm_port': tm_port,
            'physics': 'native_full_physics', 'single_tick_owner': 'caller',
            'maneuvers_validated': False,
            'limitations': ('Explicit native TM path intentions disable random lane changes and span their declared horizon; actual route/clearance still require a recorded native audit.' if cfg.native_routes else 'TM background has random junction choices and native TM junction priority; exact conflict opponents require a separate native route controller.')}
        if cfg.require_count and len(session.actors) < cfg.count:
            raise RuntimeError(f'Only {len(session.actors)}/{cfg.count} native traffic spawns fit; inspect spacing/reservations')
        # Apply actual marking + ego protections even before the first tick.
        session.update(world.get_snapshot())
        return session
    except Exception:
        if register_actor is None:
            session.close()
        raise
