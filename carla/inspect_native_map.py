"""Inventory a running CARLA native map before choosing real driving routes."""
from pathlib import Path
import argparse
import json
import math


def transform_record(transform):
    return {'location_m': [transform.location.x, transform.location.y, transform.location.z],
            'rotation_deg': [transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll]}


def waypoint_record(wp):
    return {'road_id': wp.road_id, 'section_id': wp.section_id, 'lane_id': wp.lane_id,
        's_m': wp.s, 'lane_width_m': wp.lane_width, 'is_junction': wp.is_junction,
        'junction_id': wp.get_junction().id if wp.is_junction else None,
        'lane_change': str(wp.lane_change), 'pose': transform_record(wp.transform)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--map', help='Omit to inspect current world; supplied map loads a new world.')
    p.add_argument('--out', type=Path, default=Path(__file__).resolve().parent / 'map-inventory')
    args = p.parse_args()
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(60)
    available = client.get_available_maps()
    world = client.load_world(args.map) if args.map else client.get_world()
    road_map = world.get_map()
    destination = args.out / road_map.name.rsplit('/', 1)[-1]
    destination.mkdir(parents=True, exist_ok=True)
    lights = world.get_actors().filter('traffic.traffic_light*')
    junctions, road_counts, spawn_records = {}, {}, []
    for wp in road_map.generate_waypoints(5):
        road_counts.setdefault(wp.road_id, set()).add(wp.lane_id)
        if wp.is_junction:
            junctions[wp.get_junction().id] = wp.get_junction()
    for index, transform in enumerate(road_map.get_spawn_points()):
        wp = road_map.get_waypoint(transform.location)
        lane_neighbors = []
        for side, neighbor in [('left', wp.get_left_lane()), ('right', wp.get_right_lane())]:
            if neighbor is not None and neighbor.lane_type == carla.LaneType.Driving:
                lane_neighbors.append({'side': side, 'same_direction': wp.lane_id * neighbor.lane_id > 0,
                    'waypoint': waypoint_record(neighbor)})
        spawn_records.append({'index': index, 'pose': transform_record(transform),
            'waypoint': waypoint_record(wp), 'neighbors': lane_neighbors})
    junction_records = []
    for ident, junction in junctions.items():
        bbox = junction.bounding_box
        paths = []
        for entry, exit_wp in junction.get_waypoints(carla.LaneType.Driving):
            delta = (exit_wp.transform.rotation.yaw - entry.transform.rotation.yaw + 180) % 360 - 180
            paths.append({'entry': waypoint_record(entry), 'exit': waypoint_record(exit_wp),
                'heading_change_deg': delta, 'left_turn_candidate': -145 < delta < -35})
        nearby = [light.id for light in lights if light.get_location().distance(bbox.location)
                  < math.hypot(bbox.extent.x, bbox.extent.y) + 20]
        junction_records.append({'id': ident, 'bbox': {'center_m': [bbox.location.x, bbox.location.y, bbox.location.z],
            'extent_m': [bbox.extent.x, bbox.extent.y, bbox.extent.z]}, 'paths': paths,
            'nearby_traffic_light_actor_ids': nearby,
            'note': 'Candidate classification only; verify actual signal groups/conflict lanes in runtime.'})
    data = {'server_version': client.get_server_version(), 'client_version': client.get_client_version(),
        'map': road_map.name, 'available_maps': available, 'spawn_points': spawn_records,
        'roads': [{'road_id': ident, 'driving_lane_ids': sorted(lanes)} for ident, lanes in sorted(road_counts.items())],
        'topology': [{'entry': waypoint_record(a), 'exit': waypoint_record(b)} for a, b in road_map.get_topology()],
        'junctions': junction_records,
        'traffic_lights': [{'id': light.id, 'pose': transform_record(light.get_transform()),
            'state': str(light.get_state())} for light in lights],
        'tesla_blueprint_available': bool(world.get_blueprint_library().filter('vehicle.tesla.model3')),
        'simulation_ticks_performed': 0, 'actions_verified': False}
    (destination / 'inventory.json').write_text(json.dumps(data, indent=2), encoding='utf-8')
    (destination / 'map.xodr').write_text(road_map.to_opendrive(), encoding='utf-8')
    print(json.dumps({'directory': str(destination), 'map': data['map'], 'spawns': len(spawn_records),
                      'junctions': len(junction_records), 'tesla': data['tesla_blueprint_available']}))


if __name__ == '__main__':
    main()
