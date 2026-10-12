"""Persistent native traffic roles; controls only, never Client/tick/spawn/poses.

The caller owns actors and synchronous physics. Routes are explicit native
references. This minimal version keeps each actor on its original route/lane.
"""
from __future__ import annotations

import math
import bisect
from native_controller import NativeRuleController


def _speed(v):
    return math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)


def _role(actor):
    return actor.attributes.get('role_name','')


def _vehicles(world):
    return list(world.get_actors().filter('vehicle.*'))


class NativeRouteMetric:
    """Measured native route arc, including lane reversals in OpenDRIVE s.

    A projected actor must occupy an actual road/lane of this route. References
    come from carla.Map and never move actors. Repeated route keys are rejected
    rather than selecting an arbitrary lap for a physical follower.
    """
    def __init__(self, road_map, references):
        self.parts = {}
        arc = 0.
        previous = None
        for road, lane, s in references:
            waypoint = road_map.get_waypoint_xodr(int(road), int(lane), float(s))
            if waypoint is None:
                raise ValueError('Unresolved native traffic route reference')
            location = waypoint.transform.location
            point = (location.x, location.y, location.z)
            if previous is not None:
                step = math.dist(point, previous)
                if step > 8.:
                    raise ValueError('Discontinuous native traffic route')
                arc += step
            self.parts.setdefault((int(road), int(lane)), []).append((float(s), arc))
            previous = point
        for key, points in self.parts.items():
            direction = 1 if points[-1][0] >= points[0][0] else -1
            if any(direction * (b[0]-a[0]) < -1e-5 for a,b in zip(points,points[1:])):
                raise ValueError('Repeated/reversed road segment is ambiguous')
            ordered = sorted(points)
            self.parts[key] = ([p[0] for p in ordered], [p[1] for p in ordered])

    def project(self, waypoint, *, corresponding_lane=None):
        if waypoint is None:
            return None
        key = (int(waypoint.road_id), int(waypoint.lane_id if corresponding_lane is None else corresponding_lane))
        part = self.parts.get(key)
        if part is None:
            return None
        positions, arcs = part
        value = float(waypoint.s)
        # A partial first/last segment cannot be extrapolated into other roads.
        if value < positions[0]-.5 or value > positions[-1]+.5:
            return None
        i = max(0, min(len(positions)-2, bisect.bisect_right(positions,value)-1))
        if len(positions) == 1:
            return arcs[0]
        span = positions[i+1]-positions[i]
        return arcs[i] if abs(span)<1e-9 else arcs[i]+(arcs[i+1]-arcs[i])*(value-positions[i])/span


class V3TrafficController:
    def __init__(self, world, actor, role_spec, *, carla_api):
        self.world, self.ego, self.spec, self.carla = world, actor, dict(role_spec), carla_api
        self.role = self.spec.get('name', _role(actor))
        self.lead_role = self.spec.get('lead_role', self.spec.get('leader_role'))
        self.nominal_speed_kmh = float(self.spec.get('nominal_speed_kmh', self.spec.get('desired_speed_kmh', 70)))
        self.headway_s = float(self.spec.get('desired_time_headway_s', 1.5))
        self.min_body_gap_m = float(self.spec.get('min_body_gap_m', 5.0))
        if self.nominal_speed_kmh <= 0 or self.headway_s < 1 or self.min_body_gap_m < 3:
            raise ValueError('Positive speed, headway>=1s and body gap>=3m required')
        if self.spec.get('lane_follow_ego', False):
            raise ValueError('Minimal v3 roles keep fixed native lanes; lane_follow_ego is unsupported')
        stages = self.spec.get('stages')
        if not stages:
            path = self.spec.get('xodr_path')
            if not path:
                raise ValueError('A v3 role needs dense xodr_path or native stages')
            stages = [{'name':'persistent_native_traffic_route','kind':'follow_path',
                'xodr_path':path,'speed_kmh':self.nominal_speed_kmh,'complete_radius_m':1.5}]
        following_policy=dict(self.spec.get('following_policy',{}))
        following_policy.setdefault('min_body_gap_m',self.min_body_gap_m)
        following_policy.setdefault('time_headway_s',self.headway_s)
        self.native_lane_corridor=following_policy.get('native_lane_corridor') is True
        self.base = NativeRuleController(world, actor, {'controller':'native_rule',
            'desired_speed_kmh':self.nominal_speed_kmh,
            'following_policy':following_policy,
            'stages':[dict(s) for s in stages]}, carla_api)
        self.events = self.base.events
        self.diagnostics = {}
        self._pace_released = False
        self.stage_speed_caps = [float(stage.get('speed_kmh',self.nominal_speed_kmh)) for stage in stages]
        # Native lane-change stages have physical paths constructed at runtime.
        # Never join the references either side of a lane change across a hole.
        self.route_metrics = {i:NativeRouteMetric(world.get_map(),stage['xodr_path'])
            for i,stage in enumerate(stages) if stage.get('xodr_path')}
        self.route_metric = self.route_metrics.get(0)

    def _native_separation(self, own_wp, other_wp, *, parallel_lane=False):
        if self.route_metric is None:
            return None
        own_arc = self.route_metric.project(own_wp)
        other_arc = self.route_metric.project(other_wp)
        if other_arc is None and parallel_lane and own_wp is not None and other_wp is not None:
            # Pacing across parallel lanes is intentional speed control only.
            # Require same-sign lanes, same actual road and near lane indices.
            if (own_wp.road_id == other_wp.road_id and own_wp.lane_id*other_wp.lane_id>0
                    and abs(own_wp.lane_id-other_wp.lane_id)<=2):
                other_arc = self.route_metric.project(other_wp, corresponding_lane=own_wp.lane_id)
        return None if own_arc is None or other_arc is None else other_arc-own_arc

    @property
    def finished(self):
        return self.base.index >= len(self.base.stages)

    def step(self, snapshot):
        self.route_metric = self.route_metrics.get(self.base.index)
        cap=self.stage_speed_caps[min(self.base.index,len(self.stage_speed_caps)-1)]
        desired = min(self.nominal_speed_kmh,cap) / 3.6
        own = self.ego.get_transform(); forward = own.get_forward_vector()
        own_speed = _speed(self.ego.get_velocity()); leader_detail = None
        own_wp = self.world.get_map().get_waypoint(own.location)
        pace_active = False
        pace = self.spec.get('pace_until_xodr')
        if (pace or self.spec.get('pace_role')) and not self._pace_released:
            for other in _vehicles(self.world):
                if _role(other) != self.spec.get('pace_role','hero'):
                    continue
                waypoint=self.world.get_map().get_waypoint(other.get_location())
                self._pace_released=bool(pace and (waypoint.road_id!=int(pace['road_id']) or
                    waypoint.s>=float(pace['s_m'])))
                if not self._pace_released:
                    delta=other.get_location()-own.location
                    along=self._native_separation(own_wp,waypoint,parallel_lane=True)
                    if along is None:
                        # No fabricated route connection if the ego has left
                        # this actor's native corridor. Resume nominal speed.
                        continue
                    velocity=other.get_velocity()
                    desired=min(desired,max(0.,_speed(velocity)+
                        .6*(along-float(self.spec.get('pace_gap_m',25.)))))
                    pace_active=True
                break
        if self.lead_role:
            for other in _vehicles(self.world):
                if other.id == self.ego.id or _role(other) != self.lead_role:
                    continue
                location = other.get_location(); delta = location-own.location
                along = delta.x*forward.x+delta.y*forward.y
                lateral = abs(-delta.x*forward.y+delta.y*forward.x)
                other_wp=self.world.get_map().get_waypoint(location)
                same_native_lane=(own_wp.road_id,own_wp.lane_id)==(other_wp.road_id,other_wp.lane_id)
                distinct_parallel_lane=(own_wp.road_id==other_wp.road_id
                    and own_wp.lane_id*other_wp.lane_id>0 and own_wp.lane_id!=other_wp.lane_id)
                native_along=self._native_separation(own_wp,other_wp)
                if native_along is not None:
                    along=native_along
                heading=other.get_transform().get_forward_vector()
                same_heading=heading.x*forward.x+heading.y*forward.y>=math.cos(math.radians(30))
                # The leader is only followed while physically in this road
                # corridor. Changing ego lanes cannot teleport the follower.
                if (0 < along < float(self.spec.get('max_leader_distance_m',120.))
                        and (native_along is not None or
                             ((not self.native_lane_corridor or not distinct_parallel_lane)
                              and lateral < 2.3 and same_heading))
                        and abs(delta.z) < 3):
                    velocity=other.get_velocity()
                    leader_speed=_speed(velocity) if native_along is not None else max(0.,velocity.x*forward.x+velocity.y*forward.y)
                    gap=along-self.ego.bounding_box.extent.x-other.bounding_box.extent.x
                    required=self.min_body_gap_m+own_speed*self.headway_s
                    desired=min(desired,max(0.,leader_speed+.35*(gap-required)))
                    leader_detail={'actor_id':other.id,'role':self.lead_role,
                        'actual_body_gap_m':gap,'required_body_gap_m':required,
                        'actual_leader_speed_mps':leader_speed,
                        'separation_method':'native_route_arc' if native_along is not None else 'local_tangent',
                        'actual_native_root_separation_m':native_along}
                break
        for i,stage in enumerate(self.base.stages):
            stage['speed_kmh']=min(desired*3.6,self.stage_speed_caps[i])
        control=self.base.step(snapshot)
        self.diagnostics={**self.base.diagnostics,'role':self.role,
            'v3_nominal_speed_kmh':self.nominal_speed_kmh,
            'v3_requested_speed_mps':desired,'v3_leader':leader_detail,
            'v3_pace_active':pace_active,'v3_pace_released':self._pace_released,
            'fixed_native_route':True,'state_overrides':False}
        return control


def traffic_ready(snapshot, ego, controllers, config=None):
    """One actual-state observation; caller owns the consecutive-time latch.

    Uses ego-local geometry, not actor totals or camera visibility. The default
    rear radius is60m; do not shrink physical time gaps to force rear40 counts.
    """
    config=dict(config or {})
    values=list(controllers.values()) if isinstance(controllers,dict) else list(controllers)
    actors=[controller.ego for controller in values]
    own=ego.get_transform(); forward=own.get_forward_vector()
    front=[];rear=[];near=[];moving=[]
    radius=float(config.get('radius_m',60.))
    corridor=float(config.get('corridor_half_width_m',7.))
    for actor in actors:
        pose=actor.get_transform();delta=pose.location-own.location
        heading=pose.get_forward_vector()
        along=delta.x*forward.x+delta.y*forward.y
        lateral=-delta.x*forward.y+delta.y*forward.x
        distance=math.hypot(delta.x,delta.y)
        name=_role(actor);speed=_speed(actor.get_velocity())
        if speed >= float(config.get('actor_moving_min_mps',1.)):
            moving.append(name)
        if (distance <= radius and abs(lateral)<=corridor and abs(delta.z)<3
                and heading.x*forward.x+heading.y*forward.y>=math.cos(math.radians(30))):
            item={'id':actor.id,'role':name,'x_m':along,'y_m':lateral,
                'distance_m':distance,'actual_speed_mps':speed}
            near.append(item);(front if along>=0 else rear).append(item)
    required_roles=list(config.get('required_roles',[]))
    present_roles=[_role(a) for a in actors]
    missing=[r for r in required_roles if r not in present_roles]
    not_moving=[r for r in config.get('required_moving_roles',required_roles) if r not in moving]
    speed=_speed(ego.get_velocity())
    checks={'ego_actual_speed':speed>=float(config.get('ego_min_speed_mps',4.)),
        'actual_front_coverage':len(front)>=int(config.get('minimum_front_count',1)),
        'actual_rear_coverage':len(rear)>=int(config.get('minimum_rear_count',1)),
        'required_roles_present':not missing,'required_roles_moving':not not_moving}
    return {'ready':all(checks.values()),'checks':checks,'frame':snapshot.frame,
        'timestamp_s':snapshot.timestamp.elapsed_seconds,'ego_actual_speed_mps':speed,
        'actual_front_count':len(front),'actual_rear_count':len(rear),
        'actual_neighbors':near,'missing_roles':missing,'not_moving_roles':not_moving,
        'latch_owned_by_caller':True,'camera_visibility_claimed':False}
