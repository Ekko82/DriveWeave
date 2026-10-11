"""Offline native TM routes that avoid a scenario's complete ego corridor.

No Client, world step, actor relocation or collision overrides. This produces
native Driving waypoints and explicit TM path intentions. It cannot certify TM
tracking, visibility, physical clearance or completion before a real capture.
"""
from __future__ import annotations

import bisect
from collections import deque
import hashlib
import math
from pathlib import Path
import random
import sys

import numpy as np


def xyz(wp):
    p = wp.transform.location
    return [p.x, p.y, p.z]


def ref(wp):
    return [int(wp.road_id), int(wp.lane_id), float(wp.s)]


def lane(wp):
    return int(wp.road_id), int(wp.section_id), int(wp.lane_id)


def resolve_route(road_map, specification):
    references = specification.get('xodr_path', [])
    if len(references) < 3:
        raise ValueError('A native background route needs at least three waypoints')
    points = [road_map.get_waypoint_xodr(int(r), int(l), float(s)) for r,l,s in references]
    if any(w is None or str(w.lane_type).split('.')[-1] != 'Driving' for w in points):
        raise ValueError('Background route contains a missing/non-Driving native waypoint')
    distances = [math.dist(xyz(a), xyz(b)) for a,b in zip(points,points[1:])]
    if min(distances) < .001 or max(distances) > 4.1:
        raise ValueError('Background route has duplicate or disconnected sparse waypoints')
    for a,b,d in zip(points,points[1:],distances):
        # Same native lane must advance in its real driving direction; joins
        # require a real next() successor, never world-space proximity alone.
        if lane(a)==lane(b):
            f=a.transform.get_forward_vector()
            delta=np.array(xyz(b))-np.array(xyz(a))
            if float(delta@np.array([f.x,f.y,f.z])) <= 0:
                raise ValueError('Native traffic route travels backwards on a lane')
        successors = a.next(max(.05,d))
        if not any(lane(c)==lane(b) and math.dist(xyz(c),xyz(b)) <= .30 for c in successors):
            successors = a.next(2.0)
            if not any(lane(c)==lane(b) and math.dist(xyz(c),xyz(b)) <= 2.05 for c in successors):
                raise ValueError(f'Background route lacks native successor {ref(a)} -> {ref(b)}')
    cap = float(specification['speed_cap_kmh'])
    horizon = float(specification['coverage_seconds'])
    # TM speed is a target, not a hard physical velocity limit. Keep 20% plus
    # 30m reserve rather than silently equating desired and actual speed.
    required = cap/3.6*horizon*1.2+30.0
    if cap <= 0 or horizon <= 0 or sum(distances)+1e-6 < required:
        raise ValueError('Background route may exhaust before its declared horizon')
    first = references[0]
    spawn = specification['spawn_xodr']
    if [int(spawn['road_id']),int(spawn['lane_id'])] != [int(first[0]),int(first[1])] or abs(float(spawn['s_m'])-float(first[2])) > .01:
        raise ValueError('Route must begin at its own native spawn waypoint')
    return points, {'native_length_m':sum(distances),'required_length_m':required,
        'maximum_reference_spacing_m':max(distances),'native_successors_checked':len(distances),
        'coverage_seconds':horizon,'speed_cap_kmh':cap,'runtime_tracking_validated':False}


class CorridorGuard:
    """Conservative oriented reference envelopes, rather than lane ID aliases."""
    def __init__(self, ego_points, minimum_reference_gap_m=3.25):
        from dynamic_traffic import Footprint
        # Native Tesla half dimensions from actual source asset, padded offsets.
        self.references = [Footprint(*xyz(w),w.transform.rotation.yaw,2.50,1.15)
                           for w in ego_points]
        self.positions=np.array([[v.x,v.y,v.z] for v in self.references])
        self.minimum_reference_gap_m=float(minimum_reference_gap_m)
        self.max_radius=math.hypot(3.2,1.45)+math.hypot(2.50,1.15)

    def clearance(self, wp):
        from dynamic_traffic import Footprint,horizontal_body_clearance
        p=np.array(xyz(wp)); distances=np.linalg.norm(self.positions[:,:2]-p[:2],axis=1)
        near=np.flatnonzero((distances <= self.max_radius+self.minimum_reference_gap_m)
                            & (abs(self.positions[:,2]-p[2])<4.))
        if not len(near):
            return float(distances.min()-self.max_radius)
        body=Footprint(*p,wp.transform.rotation.yaw,3.2,1.45)
        return min(horizontal_body_clearance(body,self.references[int(i)]) for i in near)

    def safe(self, wp):
        return self.clearance(wp) >= self.minimum_reference_gap_m


def _strong_components(graph):
    sys.setrecursionlimit(max(sys.getrecursionlimit(),len(graph)*3+100))
    indices={};low={};stack=[];on=set();result=[]
    def visit(v):
        indices[v]=low[v]=len(indices);stack.append(v);on.add(v)
        for w in graph[v]:
            if w not in indices:visit(w);low[v]=min(low[v],low[w])
            elif w in on:low[v]=min(low[v],indices[w])
        if low[v]==indices[v]:
            component=[]
            while True:
                w=stack.pop();on.remove(w);component.append(w)
                if w==v:break
            result.append(component)
    for v in graph:
        if v not in indices:visit(v)
    return result


def _cycle_from(start, graph, points, rng):
    history=[];seen={};current=start
    for _ in range(len(graph)+1):
        if current in seen:
            prefix=history[:seen[current]];loop=history[seen[current]:]
            if len(loop)<4:return None
            length=sum(math.dist(xyz(points[a]),xyz(points[b])) for a,b in zip(loop,loop[1:]+loop[:1]))
            return (prefix,loop) if length >= 45 else None
        seen[current]=len(history);history.append(current)
        choices=graph[current]
        if not choices:return None
        # Native junction successors are seeded; all are already safe.
        current=choices[rng.randrange(len(choices))]
    return None


def build_isolated_traffic(road_map, plan, *, count=20, seed=121203,
                           focus_radius_m=120., coverage_seconds=75.,
                           minimum_reference_gap_m=3.25):
    """Use existing lane-successor cycles outside the FULL planned ego path.

    Route length bounds full recording + worst-case physical preroll at each
    per-vehicle speed cap. TM random lane changes are disabled for these actors.
    Dense urban traffic stays on other existing native lanes, including nearby
    parallel lanes only when oriented body envelopes leave sufficient space.
    """
    from dynamic_traffic import Footprint,horizontal_body_clearance
    ego=[road_map.get_waypoint_xodr(*q) for s in plan['stages'] for q in s['xodr_path']]
    if any(w is None for w in ego):raise ValueError('Invalid native ego path')
    guard=CorridorGuard(ego,minimum_reference_gap_m)
    points=sorted(road_map.generate_waypoints(2.),key=lambda w:(lane(w),w.s))
    indexed={};lane_positions={}
    for i,w in enumerate(points):indexed.setdefault(lane(w),[]).append(i)
    for k,v in indexed.items():lane_positions[k]=[points[i].s for i in v]
    safe={i for i,w in enumerate(points) if guard.safe(w)}
    graph={}
    for i in sorted(safe):
        w=points[i];edges=[]
        for n in w.next(2.):
            k=lane(n)
            if k not in indexed:continue
            position=bisect.bisect_left(lane_positions[k],n.s)
            candidates=indexed[k][max(0,position-1):position+1]
            if not candidates:continue
            j=min(candidates,key=lambda j:abs(points[j].s-n.s))
            if j==i or j not in safe or math.dist(xyz(n),xyz(points[j]))>2.05:continue
            # Check the actual native midpoint as well as both route endpoints.
            mids=w.next(1.)
            matching=[p for p in mids if lane(p)==lane(w) or lane(p)==lane(n)]
            if matching and not any(guard.safe(p) for p in matching):continue
            edges.append(j)
        graph[i]=sorted(set(edges))
    components=_strong_components(graph)
    cyclic={i for c in components if len(c)>=4 for i in c}
    # Remove choices that lead outside a component containing a native cycle.
    reachable=set(cyclic); reverse={i:[] for i in graph}
    for a,targets in graph.items():
        for z in targets:reverse[z].append(a)
    queue=deque(sorted(cyclic))
    while queue:
        for p in reverse[queue.popleft()]:
            if p not in reachable:reachable.add(p);queue.append(p)
    usable={i:[j for j in graph[i] if j in reachable] for i in sorted(reachable)}
    rng=random.Random(seed)
    starts=[i for i in sorted(reachable) if not points[i].is_junction and math.dist(xyz(points[i]),[0,0,0] if plan['map']=='Town03' else [-50.69,-92.60,0]) <= focus_radius_m]
    rng.shuffle(starts)
    selected=[];routes=[]
    protected_spawns=[Footprint(*xyz(ego[0]),ego[0].transform.rotation.yaw,2.50,1.15)]
    for t in plan['traffic']:
        w=road_map.get_waypoint_xodr(t['spawn_xodr']['road_id'],t['spawn_xodr']['lane_id'],t['spawn_xodr']['s_m'])
        protected_spawns.append(Footprint(*xyz(w),w.transform.rotation.yaw,3.2,1.45))
    for start in starts:
        wp=points[start];body=Footprint(*xyz(wp),wp.transform.rotation.yaw,3.2,1.45)
        if any(horizontal_body_clearance(body,o)<14. for o in selected+protected_spawns):continue
        chain=_cycle_from(start,usable,points,rng)
        if chain is None:continue
        prefix,loop=chain
        ids=prefix+loop;length=sum(math.dist(xyz(points[a]),xyz(points[z])) for a,z in zip(ids,ids[1:]))
        cap=rng.uniform(22.,32.)
        needed=cap/3.6*coverage_seconds*1.2+35.
        while length<needed:
            for j in loop:
                length+=math.dist(xyz(points[ids[-1]]),xyz(points[j]));ids.append(j)
                if length>=needed:break
        route={'route_id':f"isolated_native_{len(routes):03d}",
            'spawn_xodr':{'road_id':int(wp.road_id),'lane_id':int(wp.lane_id),'s_m':float(wp.s)},
            'xodr_path':[ref(points[i]) for i in ids], 'speed_cap_kmh':cap,
            'coverage_seconds':coverage_seconds,'minimum_reference_body_gap_m':minimum_reference_gap_m,
            'runtime_tracking_validated':False}
        _,proof=resolve_route(road_map,route)
        proof['minimum_sampled_reference_body_gap_m']=min(guard.clearance(points[i]) for i in set(ids))
        proof['path_length_overspeed_reserve_fraction']=.2
        proof['path_length_reserve_m']=30.
        route['offline_native_successor_proof']=proof
        routes.append(route);selected.append(body)
        if len(routes)==count:break
    if len(routes)<count:raise RuntimeError(f'Only {len(routes)}/{count} safe native city route spawns; no fallback to random TM junctions')
    config={'count':count,'seed':seed,'native_routes':routes,'require_count':True,
        'focus_radius_m':focus_radius_m,'blueprints':['vehicle.audi.a2','vehicle.audi.tt','vehicle.tesla.model3','vehicle.nissan.patrol','vehicle.lincoln.mkz_2020','vehicle.mercedes.coupe_2020'],
        'speed_range_kmh':[22,32],'lane_change_probability_range_pct':[0,0],
        'min_spawn_body_gap_m':5,'spawn_time_headway_s':1.0}
    evidence={'schema':'native_background_route_isolation_v1','map':plan['map'],'count':count,'seed':seed,
        'ego_reference_points':len(ego),'protected_entire_ego_reference':True,
        'native_graph_nodes':len(points),'safe_native_graph_nodes':len(safe),'cycle_reachable_nodes':len(reachable),
        'minimum_reference_body_gap_m':minimum_reference_gap_m,
        'body_envelopes_m':{'ego_half_length':2.50,'ego_half_width':1.15,'background_half_length':3.2,'background_half_width':1.45},
        'coverage_seconds_including_preroll':coverage_seconds,'route_counts':len(routes),
        'spawn_minimum_reference_body_gap_m':14.,'random_lane_change_disabled':True,
        'official_api':'TrafficManager.set_path and auto_lane_change(False); native get_waypoint_xodr/next()',
        'helper_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'physical_runtime_acceptance':False,'server_connected_or_ticked':False,
        'limits':['Route references and length are proven offline. TM path obedience is an intention until actual native snapshots are audited.','Reference gap includes conservative geometric/track margin; actual all-frame ego body clearance still must reach1.5m.','Lights and physical queues can reduce movement/visibility; opening density and activity require actual camera/sensor auditing.']}
    return config,evidence
