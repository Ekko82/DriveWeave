"""Transparent rule controller using native waypoints and VehicleControl.

Reference paths steer native CARLA physics. They do not prescribe or teleport
actor poses. This is a demonstration controller, not a trained driving model.
"""
import math


def norm(vector):
    return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def inside(point, polygon):
    hit = False
    for i, a in enumerate(polygon):
        b = polygon[i - 1]
        if (a[1] > point[1]) != (b[1] > point[1]) and point[0] < (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0]:
            hit = not hit
    return hit


class LongitudinalGovernor:
    """Comfort-limited speed tracking; emergency braking bypasses the limits.

    The acceleration target is a control request, never a claimed measurement.
    The capture log separately records actual physics velocity/acceleration.
    """
    def __init__(self, dt, acceleration=2.2, deceleration=2.8, jerk=2.5):
        self.dt = float(dt)
        self.acceleration, self.deceleration, self.jerk = acceleration, deceleration, jerk
        self.requested_acceleration = self.throttle = self.brake = 0.0
        self.previous_speed = None
        self.measured_acceleration = 0.0

    def step(self, desired, speed, emergency=False, coast_stop=False):
        if self.previous_speed is not None:
            measured = (speed-self.previous_speed)/self.dt
            self.measured_acceleration = .7*self.measured_acceleration+.3*measured
        self.previous_speed = speed
        if emergency:
            self.requested_acceleration = -self.deceleration
            self.throttle, self.brake = 0.0, 1.0
        elif coast_stop:
            # Opt-in junction physics policy: any positive brake at low speed
            # can cause a native wheel-lock spike. The zero is exact. This is a
            # control request, never a replacement velocity or acceleration.
            self.requested_acceleration = 0.0
            self.throttle, self.brake = 0.0, .35 if speed <= .1 else 0.0
        else:
            target = max(-self.deceleration, min(self.acceleration, .7*(desired-speed)))
            step = self.jerk*self.dt
            self.requested_acceleration = max(self.requested_acceleration-step,
                min(self.requested_acceleration+step, target))
            effort = self.requested_acceleration+.35*(self.requested_acceleration-self.measured_acceleration)
            if desired < .05 and speed < .25:
                target_throttle, target_brake = 0.0, .35
            elif effort > -.1:
                target_throttle, target_brake = max(0.,min(.65,.14+.2*effort)), 0.0
            else:
                target_throttle, target_brake = 0.0,max(0.,min(.8,-.25*effort))
            self.throttle = max(self.throttle-.8*self.dt,min(self.throttle+.6*self.dt,target_throttle))
            self.brake = max(self.brake-1.0*self.dt,min(self.brake+1.0*self.dt,target_brake))
            # Actuators never fight each other during a transition.
            if target_brake > 0:
                self.throttle = 0.0
            elif self.brake > .01:
                self.throttle = 0.0
        return self.throttle, self.brake


def junction_coast_stop_policy():
    """Actual 0512 Tesla coast samples plus explicit planning margins.

    Distances below are remaining *actual pose arc* while already coasting,
    not distances asserted for every scene. The measured 0.911497m release
    transient and one upcoming physics tick are added before initiating coast.
    Full scene motion, stop pose, conflict and clearance audits remain required.
    """
    return {'enabled': True, 'mode': 'coast_only',
        'blueprint': 'vehicle.tesla.model3', 'maximum_approach_speed_mps': 6.0,
        'actual_stop_speed_mps': .1, 'actual_stopped_confirmation_seconds': .5,
        'release_transient_margin_m': .9114971194266213,
        'calibration_source': 'verification/brake-probe-20261012-0512/coast_only/frames.jsonl',
        'calibration_frames_sha256': '69366135ca704b54c0e966603e451680b8018485b95457a5f13861cd9b5c7e0a',
        'coasting_speed_remaining_distance_m': [
            [.09710806933138047, 0.], [.2468581986538692, .34626388836353683],
            [.5021088060265628, .9729244031957239], [.7521614963146751, 1.5598391979844626],
            [1.0045546866142412, 2.167351162106249], [1.4857801443600218, 3.332336413331056],
            [1.9905121461985658, 4.539784884858598], [2.509745609133081, 5.769381454691748],
            [3.0216964324018925, 7.007837631099087], [3.4978004945483465, 8.144597722353737],
            [3.9641022479343038, 9.259330824773468], [4.4948371783367636, 10.524014085815809],
            [4.976704168978188, 11.95022905335819], [5.567550272468093, 13.261016763499251],
            [5.9787192689150945, 14.413224537760419], [6.02685458885608, 14.713156380546377]],
        'scene_runtime_acceptance': False}


def remaining_path_arc(position, path, nearest):
    """Project onto actual reference segments; return remaining native arc."""
    candidates = []
    for i in range(max(0, nearest-1), min(len(path)-1, nearest+1)):
        a, b = path[i], path[i+1]
        dx, dy = b[0]-a[0], b[1]-a[1]
        squared = dx*dx+dy*dy
        if squared <= 1e-12:
            continue
        u = max(0., min(1., ((position[0]-a[0])*dx+(position[1]-a[1])*dy)/squared))
        point = [a[0]+u*dx, a[1]+u*dy]
        remaining = (1.-u)*math.sqrt(squared)+sum(distance(x,y) for x,y in zip(path[i+1:],path[i+2:]))
        candidates.append((distance(position, point), remaining))
    if not candidates:
        return distance(position, path[-1])
    return min(candidates)[1]


class JunctionCoastStop:
    def __init__(self, specification, dt):
        self.enabled = specification.get('enabled') is True
        self.specification, self.dt = specification, float(dt)
        self.latched_stage = None
        self.stopped_since = None
        if self.enabled:
            if specification.get('mode') != 'coast_only':
                raise ValueError('Only the actually calibrated coast_only junction policy is supported')
            self.table = specification['coasting_speed_remaining_distance_m']
            if len(self.table)<2 or any(a[0]>=b[0] or a[1]>b[1] for a,b in zip(self.table,self.table[1:])):
                raise ValueError('Coast calibration must be ordered actual speed/distance samples')
            if specification['actual_stop_speed_mps'] != .1 or specification['actual_stopped_confirmation_seconds'] < .5:
                raise ValueError('Junction stop requires actual <=0.1m/s and >=0.5s confirmation')

    def observe_actual_stop(self, speed, timestamp):
        if speed <= .1:
            if self.stopped_since is None:
                self.stopped_since = timestamp
        else:
            self.stopped_since = None

    def stopped_seconds(self, timestamp):
        return 0. if self.stopped_since is None else max(0.,timestamp-self.stopped_since)

    def trigger_distance(self, speed):
        if speed <= self.table[0][0]:
            base = 0.
        elif speed >= self.table[-1][0]:
            # Explicit planning extrapolation only; final actual trajectory is
            # audited. The approach target is capped at measured nominal6m/s.
            a,b = self.table[-2:]
            base = b[1]+(speed-b[0])*(b[1]-a[1])/(b[0]-a[0])
        else:
            for a,b in zip(self.table,self.table[1:]):
                if a[0] <= speed <= b[0]:
                    base = a[1]+(speed-a[0])*(b[1]-a[1])/(b[0]-a[0])
                    break
        return base+float(self.specification['release_transient_margin_m'])+speed*self.dt

    def approach(self, stage_index, remaining_arc_m, speed):
        trigger = self.trigger_distance(speed)
        if remaining_arc_m <= trigger:
            self.latched_stage = stage_index
        return self.latched_stage == stage_index, trigger


def spawn_from_xodr(road_map, specification, carla):
    wp = road_map.get_waypoint_xodr(int(specification['road_id']), int(specification['lane_id']), float(specification['s_m']))
    if wp is None or wp.lane_type != carla.LaneType.Driving:
        raise RuntimeError(f'No native driving waypoint at {specification}')
    pose = wp.transform
    pose.location.z += float(specification.get('z_offset_m', .3))
    return pose


class NativeRuleController:
    def __init__(self, world, ego, plan, carla):
        self.world, self.ego, self.plan, self.carla = world, ego, plan, carla
        self.road_map = world.get_map()
        self.stages = plan['stages']
        if not self.stages:
            raise RuntimeError('Native rule controller needs explicit stages.')
        self.index = 0
        self.path, self.progress = None, 0
        self.events, self.diagnostics = [], {}
        self.wait_reason, self.gate_released = None, False
        self.gate_histories = {}
        self.target_lane, self.target_road = None, None
        self.previous_steer = 0
        self.dt = world.get_settings().fixed_delta_seconds
        self.longitudinal = LongitudinalGovernor(self.dt)
        self.coast_stop = JunctionCoastStop(plan.get('longitudinal_stop_policy', {}), self.dt)
        if self.coast_stop.enabled and ego.type_id != self.coast_stop.specification['blueprint']:
            raise ValueError('Junction coast policy is calibrated only for the native Tesla model3')
        front_angles = [wheel.max_steer_angle for wheel in ego.get_physics_control().wheels if wheel.max_steer_angle > 1]
        self.max_steer_rad = math.radians(max(front_angles) if front_angles else 70)
        self.wheelbase_m = float(plan.get('wheelbase_m', ego.bounding_box.extent.x * 1.24))
        ego.set_autopilot(False)
        for stage in self.stages:
            if stage['kind'] not in ['follow_path', 'lane_change']:
                raise RuntimeError('Stage kind must be follow_path or lane_change.')
            if stage['kind'] == 'follow_path' and not (stage.get('path') or stage.get('xodr_path')):
                raise RuntimeError('Follow stage must contain a real native-map path.')

    def event(self, name, snapshot, **detail):
        self.events.append({'event': name, 'frame': snapshot.frame, 'timestamp': snapshot.timestamp.elapsed_seconds,
            'stage_index': self.index, 'stage': self.stages[self.index]['name'] if self.index < len(self.stages) else 'finished', **detail})

    def wait(self, reason, snapshot, **detail):
        if self.wait_reason != reason:
            self.event('controller_wait', snapshot, reason=reason, **detail)
        self.wait_reason = reason
        self.diagnostics = {'stage_index': self.index, 'waiting': reason, **detail}
        throttle, brake = self.longitudinal.step(0.0, norm(self.ego.get_velocity()),
            coast_stop=self.coast_stop.enabled)
        self.diagnostics.update({'actual_speed_mps':norm(self.ego.get_velocity()),
            'desired_speed_mps':0.0,'requested_acceleration_mps2':self.longitudinal.requested_acceleration,
            'throttle':throttle,'brake':brake,'steer':self.previous_steer})
        return self.carla.VehicleControl(throttle=throttle, brake=brake, steer=self.previous_steer)

    def followers(self):
        return [v for v in self.world.get_actors().filter('vehicle.*') if v.id != self.ego.id and v.is_alive]

    def observe_future_gates(self, vehicles):
        # History begins during the recorded approach, not only after the ego
        # reaches its stop point. This prevents waiting forever for an actor
        # that already passed while the ego was still approaching.
        for index in range(self.index,len(self.stages)):
            gate=self.stages[index].get('yield_gate',{})
            required=set(gate.get('require_observed_pass_roles',[]))
            if not required:
                continue
            observed=self.gate_histories.setdefault(index,set())
            for vehicle in vehicles:
                role=vehicle.attributes.get('role_name')
                if role not in required:
                    continue
                p=vehicle.get_location()
                if abs(p.z-self.ego.get_location().z)<=3 and inside((p.x,p.y),gate['polygon_xy']):
                    observed.add(role)

    def gate_conflicts(self, gate, vehicles):
        polygon = gate['polygon_xy']
        horizon = float(gate.get('prediction_s', 6))
        conflicts = []
        observed=self.gate_histories.setdefault(self.index,set())
        eligible_roles=set()
        for vehicle in vehicles:
            if gate.get('actor_roles') and vehicle.attributes.get('role_name') not in gate['actor_roles']:
                continue
            pose, velocity = vehicle.get_location(), vehicle.get_velocity()
            if abs(pose.z - self.ego.get_location().z) > 3:
                continue
            native_wp = self.road_map.get_waypoint(pose)
            observed_speed = norm(velocity)
            role = vehicle.attributes.get('role_name')
            eligible_roles.add(role)
            occupied = inside((pose.x,pose.y),polygon)
            if occupied:
                observed.add(role)
            # A stopped conflicting actor upstream is not evidence that it
            # yielded permanently. Explicit demonstration traffic must really
            # traverse the conflict region before the ego can be released.
            if role in gate.get('require_observed_pass_roles', []):
                if role not in observed or occupied:
                    conflicts.append({'actor_id':vehicle.id,'role':role,
                        'predicted_conflict_s':0 if occupied else None,
                        'reason':'await_actual_conflict_vehicle_passage'})
                    continue
            # Predictions follow real map connectivity where available, so a
            # curved circulating lane is not replaced by a straight tangent.
            # They are conservative predictions, never imposed actor motion.
            for step in range(int(horizon * 2) + 1):
                dt = step / 2
                candidates = [pose]
                if dt > 0 and observed_speed > .3:
                    next_points = native_wp.next(observed_speed*dt) if native_wp else []
                    candidates = [wp.transform.location for wp in next_points] or [
                        self.carla.Location(x=pose.x+velocity.x*dt,y=pose.y+velocity.y*dt,z=pose.z)]
                if any(inside((point.x,point.y),polygon) for point in candidates):
                    conflicts.append({'actor_id': vehicle.id, 'predicted_conflict_s': dt,
                        'prediction': 'observed speed along native possible waypoint successors'})
                    break
        for role in set(gate.get('require_observed_pass_roles',[]))-eligible_roles:
            conflicts.append({'actor_id':None,'role':role,'predicted_conflict_s':None,
                'reason':'required_conflict_actor_unavailable'})
        return conflicts

    def lane_gap(self, target, vehicles, speed):
        anchor = target.transform.location
        heading = target.transform.get_forward_vector()
        nearest_front, nearest_rear, risk = None, None, []
        for vehicle in vehicles:
            p, velocity = vehicle.get_location(), vehicle.get_velocity()
            dx, dy = p.x - anchor.x, p.y - anchor.y
            lateral = -dx * heading.y + dy * heading.x
            if abs(p.z - anchor.z) > 3 or abs(lateral) > target.lane_width / 2 + vehicle.bounding_box.extent.y + .25:
                continue
            along = dx * heading.x + dy * heading.y
            bumper_gap = abs(along) - vehicle.bounding_box.extent.x - self.ego.bounding_box.extent.x
            other_speed = max(0, velocity.x * heading.x + velocity.y * heading.y)
            required = max(9, speed * 1.3) if along >= 0 else max(12, other_speed * 1.4 + max(0, other_speed - speed) * 3)
            item = {'actor_id': vehicle.id, 'gap_m': bumper_gap, 'required_m': required}
            if along >= 0:
                if nearest_front is None or bumper_gap < nearest_front['gap_m']:
                    nearest_front = item
            elif nearest_rear is None or bumper_gap < nearest_rear['gap_m']:
                nearest_rear = item
            if bumper_gap < required:
                risk.append(item)
        return risk, {'front': nearest_front, 'rear': nearest_rear}

    def advance_native(self, wp, step):
        candidates = [candidate for candidate in wp.next(step) if candidate.lane_type == self.carla.LaneType.Driving and not candidate.is_junction]
        same = [candidate for candidate in candidates if candidate.road_id == wp.road_id and candidate.lane_id == wp.lane_id]
        if same:
            return same[0]
        # Lane change reference is restricted to one native road before a junction.
        return None

    def build_lane_change(self, stage, snapshot, vehicles, speed):
        wp = self.road_map.get_waypoint(self.ego.get_location())
        direction = stage['direction']
        if direction not in ['left', 'right']:
            raise RuntimeError('Lane change direction must be left or right.')
        target = wp.get_left_lane() if direction == 'left' else wp.get_right_lane()
        if wp.is_junction or target is None or target.lane_type != self.carla.LaneType.Driving or target.lane_id * wp.lane_id <= 0:
            return None, 'no_native_same_direction_neighbor', {}
        permission = str(wp.lane_change).split('.')[-1]
        if permission not in ['Both', direction.capitalize()]:
            return None, 'native_marking_prohibits_lane_change', {'permission': permission}
        risk, gap = self.lane_gap(target, vehicles, speed)
        if risk:
            return None, 'target_lane_gap', {'risks': risk, 'gap': gap}
        length = max(35, float(stage.get('distance_m', 45)))
        source, destination, path = wp, target, []
        for i in range(math.ceil((length + 12) / 1.5) + 1):
            travelled = i * 1.5
            u = min(1, travelled / length)
            blend = u**3 * (10 - 15 * u + 6 * u*u)
            a, b = source.transform.location, destination.transform.location
            path.append([a.x * (1-blend) + b.x * blend, a.y * (1-blend) + b.y * blend, a.z * (1-blend) + b.z * blend])
            if travelled >= length + 12:
                break
            source = self.advance_native(source, 1.5)
            destination = self.advance_native(destination, 1.5)
            if source is None or destination is None:
                return None, 'insufficient_native_lane_change_corridor', {}
        self.target_lane, self.target_road = target.lane_id, target.road_id
        self.event('lane_change_started', snapshot, direction=direction, from_road=wp.road_id,
            from_lane=wp.lane_id, target_lane=target.lane_id, reference_distance_m=length, gap=gap)
        return path, None, {}

    def load_path(self, stage):
        if stage.get('path'):
            result = [[float(v) for v in point] for point in stage['path']]
        else:
            result = []
            for point in stage['xodr_path']:
                wp = self.road_map.get_waypoint_xodr(int(point[0]), int(point[1]), float(point[2]))
                if wp is None:
                    raise RuntimeError(f'No native waypoint for {point}')
                pose = wp.transform.location
                result.append([pose.x, pose.y, pose.z])
        if len(result) < 2 or any(distance(a,b) > 8 for a,b in zip(result,result[1:])):
            raise RuntimeError('Rule path requires dense native connected points, spacing <=8m.')
        return result

    def step(self, snapshot):
        if self.index >= len(self.stages):
            return self.wait('plan_finished', snapshot, finished=True)
        stage = self.stages[self.index]
        vehicles = self.followers()
        self.observe_future_gates(vehicles)
        speed = norm(self.ego.get_velocity())
        self.coast_stop.observe_actual_stop(speed, snapshot.timestamp.elapsed_seconds)
        transform = self.ego.get_transform()
        position = [transform.location.x, transform.location.y, transform.location.z]
        if self.ego.is_at_traffic_light() and self.ego.get_traffic_light_state() in [self.carla.TrafficLightState.Red, self.carla.TrafficLightState.Yellow]:
            return self.wait('native_traffic_light_stop', snapshot, state=str(self.ego.get_traffic_light_state()))
        if stage.get('yield_gate') and not self.gate_released:
            conflicts = self.gate_conflicts(stage['yield_gate'], vehicles)
            if conflicts:
                return self.wait('native_conflict_traffic', snapshot, conflicts=conflicts)
            stop_limit = .1 if self.coast_stop.enabled else .5
            stopped_duration = self.coast_stop.stopped_seconds(snapshot.timestamp.elapsed_seconds)
            if speed > stop_limit or self.coast_stop.enabled and stopped_duration < .5:
                return self.wait('settle_before_yield_release', snapshot)
            self.gate_released = True
            self.event('yield_gate_released', snapshot, conflict_actor_count=0)
        if self.path is None:
            gap_waiting = None
            if stage['kind'] == 'lane_change':
                self.path, reason, details = self.build_lane_change(stage, snapshot, vehicles, speed)
                if self.path is None:
                    if reason != 'target_lane_gap':
                        return self.wait(reason, snapshot, **details)
                    # Remain on the current real lane while seeking a gap.
                    # This transient reference is regenerated on the next step;
                    # it cannot complete the lane-change stage.
                    wp = self.road_map.get_waypoint(self.ego.get_location())
                    self.path = []
                    current = wp
                    for _ in range(28):
                        if current is None:
                            break
                        p=current.transform.location
                        self.path.append([p.x,p.y,p.z])
                        current=self.advance_native(current,1.5)
                    if len(self.path)<3:
                        self.path=None
                        return self.wait('insufficient_gap_waiting_corridor',snapshot)
                    gap_waiting = details
                    if self.wait_reason != reason:
                        self.event('controller_gap_wait',snapshot,reason=reason,**details)
                    self.wait_reason=reason
            else:
                self.path = self.load_path(stage)
                self.event('path_follow_started', snapshot, kind=stage['kind'], reference_points=len(self.path))
            self.progress = 0
            if gap_waiting is None:
                self.wait_reason = None
        else:
            gap_waiting = None
        window = range(max(0, self.progress-2), min(len(self.path), self.progress+35))
        nearest = min(window, key=lambda i: distance(position, self.path[i]))
        reference_error = distance(position, self.path[nearest])
        if reference_error > 3.5:
            return self.wait('excessive_reference_error', snapshot, error_m=reference_error)
        self.progress = max(self.progress, nearest)
        lookahead_base = float(stage.get('lookahead_base_m',4))
        lookahead = max(lookahead_base,min(float(stage.get('lookahead_max_m',14)),
            lookahead_base+speed*float(stage.get('lookahead_speed_s',.65))))
        target_index, arc = self.progress, 0
        while target_index + 1 < len(self.path) and arc < lookahead:
            arc += distance(self.path[target_index], self.path[target_index+1])
            target_index += 1
        target = self.path[target_index]
        dx, dy = target[0] - position[0], target[1] - position[1]
        alpha = (math.atan2(dy, dx) - math.radians(transform.rotation.yaw) + math.pi) % (2*math.pi) - math.pi
        steering_angle = math.atan2(2 * self.wheelbase_m * math.sin(alpha), max(2, math.hypot(dx,dy)))
        steer = max(-.75, min(.75, steering_angle / self.max_steer_rad))
        steer = max(self.previous_steer-self.dt*1.2, min(self.previous_steer+self.dt*1.2, steer))
        self.previous_steer = steer
        desired = float(stage.get('speed_kmh', self.plan.get('desired_speed_kmh', 40))) / 3.6
        desired = min(desired, max(2, math.sqrt(2.5 / max(.001, abs(2*math.sin(alpha)/max(2,math.hypot(dx,dy)))))))
        active_lane_gap = None
        if stage['kind'] == 'lane_change' and gap_waiting is None:
            current_native = self.road_map.get_waypoint(transform.location)
            target_native = self.road_map.get_waypoint_xodr(self.target_road, self.target_lane, current_native.s)
            if target_native is not None:
                risks, active_lane_gap = self.lane_gap(target_native, vehicles, speed)
                if risks:
                    # Keep the planned physical steering; limit forward speed on
                    # emerging conflicts rather than reassigning an actor pose.
                    front = active_lane_gap['front']
                    if front and front['gap_m'] < front['required_m']:
                        desired = min(desired, max(0, (front['gap_m']-4)/1.5))
                    if any(item['gap_m'] < 3 for item in risks):
                        desired = 0
        remaining = distance(position, self.path[-1])
        coast_requested, coast_trigger, remaining_arc = False, None, None
        if stage.get('stop_at_end'):
            if self.coast_stop.enabled:
                desired = min(desired, self.coast_stop.specification['maximum_approach_speed_mps'])
                remaining_arc = remaining_path_arc(position,self.path,nearest)
                coast_requested,coast_trigger = self.coast_stop.approach(self.index,remaining_arc,speed)
                if coast_requested:
                    desired = 0.
            else:
                desired = min(desired, math.sqrt(max(0, 2 * 1.8 * (remaining-.7))))
        blocking = []
        emergency = False
        forward = transform.get_forward_vector()
        for vehicle in vehicles:
            other, velocity = vehicle.get_location(), vehicle.get_velocity()
            ox, oy = other.x - position[0], other.y - position[1]
            along = ox * forward.x + oy * forward.y
            lateral = abs(-ox * forward.y + oy * forward.x)
            gap = along - self.ego.bounding_box.extent.x - vehicle.bounding_box.extent.x
            if abs(other.z-position[2]) < 3 and along > 0 and lateral < self.ego.bounding_box.extent.y + vehicle.bounding_box.extent.y + .35:
                safe = max(4, speed*1.5)
                if gap < safe:
                    other_speed = max(0, velocity.x*forward.x + velocity.y*forward.y)
                    desired = min(desired, max(0, other_speed + (gap-safe)*.35))
                    blocking.append({'actor_id': vehicle.id, 'gap_m': gap})
                    closing = max(0.,speed-other_speed)
                    emergency |= gap < 1.5 or closing > .1 and gap/closing < .8
        throttle, brake = self.longitudinal.step(desired, speed, emergency=emergency,
            coast_stop=coast_requested)
        current_wp = self.road_map.get_waypoint(transform.location)
        if gap_waiting is None and remaining < float(stage.get('complete_radius_m', 2.2)) and self.progress >= len(self.path)-4:
            stop_limit = .1 if self.coast_stop.enabled else .5
            completed = not stage.get('stop_at_end') or speed <= stop_limit
            if stage['kind'] == 'lane_change':
                lateral_error = current_wp.transform.location.distance(transform.location)
                completed = completed and current_wp.road_id == self.target_road and current_wp.lane_id == self.target_lane and lateral_error < .75
            if completed:
                self.event('stage_completed', snapshot, actual_road=current_wp.road_id, actual_lane=current_wp.lane_id,
                    location_m=position, speed_mps=speed)
                self.gate_histories.pop(self.index,None)
                self.index += 1
                self.path, self.progress, self.gate_released, self.wait_reason = None, 0, False, None
        self.diagnostics = {'stage_index': self.index, 'stage': stage['name'], 'reference_progress': self.progress,
            'desired_speed_mps': desired, 'actual_speed_mps': speed, 'blocking': blocking,
            'active_target_lane_gap': active_lane_gap,
            'gap_waiting':gap_waiting,'emergency_braking':emergency,
            'requested_acceleration_mps2':self.longitudinal.requested_acceleration,
            'junction_coast_stop': {'enabled':self.coast_stop.enabled,'requested':coast_requested,
                'remaining_native_arc_m':remaining_arc,'trigger_distance_m':coast_trigger,
                'actual_stopped_seconds':self.coast_stop.stopped_seconds(snapshot.timestamp.elapsed_seconds),
                'actual_stop_speed_limit_mps':.1 if self.coast_stop.enabled else .5,
                'emergency_override':bool(emergency)},
            'steer': steer, 'throttle': throttle, 'brake': brake}
        if gap_waiting is not None:
            self.path, self.progress = None, 0
        return self.carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)
