"""Minimal official CARLA 0.9.16/SUMO live bridge with raw state evidence.

Imported only by capture_live_cosim; never launches anything on import.
SUMO-controlled actors use the official kinematic mirror, while CARLA-owned
actors keep native physics and their existing VehicleControl/TrafficManager.
"""
from pathlib import Path
import hashlib
import json
import math
import os
import sys
import xml.etree.ElementTree as ET


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class LiveCosimBridge:
    def __init__(self, args, run, world):
        self.args, self.run, self.world = args, run, world
        self.sync = self.sumo = self.carla = None
        self.log = None
        self.count = 0
        self.autonomous_positions = {}
        self.closed = False
        if args.tm_port != 8000:
            raise ValueError('Official bridge configures TrafficManager 8000; use --tm-port 8000.')
        if not args.sumo_cfg.is_file():
            raise FileNotFoundError(args.sumo_cfg)
        os.environ['SUMO_HOME'] = str(args.sumo_home.resolve())
        os.environ['PATH'] = str(args.sumo_home.resolve() / 'bin') + os.pathsep + os.environ['PATH']
        sys.path.insert(0, str(args.carla_sumo.resolve()))
        sys.path.insert(0, str(args.sumo_home.resolve() / 'tools'))
        from sumo_integration.sumo_simulation import SumoSimulation
        from sumo_integration.carla_simulation import CarlaSimulation
        from run_synchronization import SimulationSynchronization
        import traci
        self.traci = traci
        cfg = ET.parse(args.sumo_cfg).getroot()
        net_value = cfg.find('.//net-file').get('value')
        network = Path(net_value)
        if not network.is_absolute():
            network = args.sumo_cfg.parent / network
        self.metadata = {'mode': 'live_official_bidirectional_actor_synchronization',
            'sumo_cfg': str(args.sumo_cfg.resolve()), 'sumo_cfg_sha256': sha(args.sumo_cfg),
            'network': str(network.resolve()), 'network_sha256': sha(network),
            'official_sync_source': str(args.carla_sumo / 'run_synchronization.py'),
            'official_sync_sha256': sha(args.carla_sumo / 'run_synchronization.py'),
            'official_bridge_sha256': sha(args.carla_sumo / 'sumo_integration/bridge_helper.py'),
            'carla_owned_control': 'native physical VehicleControl / TrafficManager',
            'sumo_owned_control': 'autonomous SUMO traffic; official CARLA mirror has physics disabled and receives set_transform',
            'gui_capture': False, 'postcapture_replay': False,
            'synchronization_phase_order': 'SUMO step; SUMO-to-CARLA mirror; CARLA physical tick; CARLA-to-SUMO mirror',
            'tls_manager': 'carla', 'step_length_s': 1 / args.fps}
        try:
            self.sumo = SumoSimulation(str(args.sumo_cfg.resolve()), 1 / args.fps, sumo_gui=False)
            self.carla = CarlaSimulation(args.host, args.port, 1 / args.fps)
            self.carla.client.set_timeout(args.sensor_timeout)
            self.sync = SimulationSynchronization(self.sumo, self.carla, tls_manager='carla',
                                                 sync_vehicle_color=True, sync_vehicle_lights=False)
            self.metadata['sumo_version'] = list(traci.getVersion())
            self.metadata['network_offset_m'] = list(self.sumo.get_net_offset())
            self.log = (run / 'sumo-sync.jsonl').open('w', encoding='utf-8')
        except BaseException:
            self.close()
            raise

    def tick(self):
        # The official synchronizer is the only simulation ticker.
        before = self.world.get_snapshot().frame
        self.sync.tick()
        frame = self.world.get_snapshot().frame
        if frame != before + 1:
            raise RuntimeError(f'Unexpected live world frame gap {before} -> {frame}')
        return frame

    def record(self, sample, snapshot):
        sumo_ids = set(self.traci.vehicle.getIDList())
        rows = []
        for direction, mapping in [('sumo_to_carla', self.sync.sumo2carla_ids),
                                   ('carla_to_sumo', self.sync.carla2sumo_ids)]:
            for source, target in sorted(mapping.items(), key=lambda item: str(item[0])):
                sid, cid = (source, target) if direction == 'sumo_to_carla' else (target, source)
                if sid not in sumo_ids:
                    raise RuntimeError(f'Mapped SUMO actor is missing: {sid}')
                actor_snapshot = snapshot.find(cid)
                actor = self.world.get_actor(cid)
                if actor_snapshot is None or actor is None:
                    raise RuntimeError(f'Mapped CARLA actor absent from actual world snapshot: {cid}')
                xy = list(self.traci.vehicle.getPosition(sid))
                pose = actor_snapshot.get_transform()
                row = {'direction': direction, 'sumo_id': sid, 'carla_id': cid,
                    'sumo_front_bumper_xy_m': xy, 'sumo_navigation_angle_deg': self.traci.vehicle.getAngle(sid),
                    'sumo_speed_mps': self.traci.vehicle.getSpeed(sid),
                    'sumo_lane_id': self.traci.vehicle.getLaneID(sid),
                    'carla_world_center_m': [pose.location.x, pose.location.y, pose.location.z],
                    'carla_rotation_pitch_yaw_roll_deg': [pose.rotation.pitch, pose.rotation.yaw, pose.rotation.roll],
                    'carla_world_from_local': pose.get_matrix(),
                    'carla_bbox_extent_m': [actor.bounding_box.extent.x, actor.bounding_box.extent.y, actor.bounding_box.extent.z],
                    'carla_role': actor.attributes.get('role_name'), 'carla_blueprint': actor.type_id,
                    'source_authority': 'SUMO ordinary autonomous simulation' if direction == 'sumo_to_carla' else 'CARLA native physical controller'}
                rows.append(row)
                if direction == 'sumo_to_carla':
                    history = self.autonomous_positions.setdefault(sid, {'initial_xy': xy, 'max_displacement_m': 0., 'max_speed_mps': 0., 'samples': 0})
                    history['max_displacement_m'] = max(history['max_displacement_m'], math.dist(xy, history['initial_xy']))
                    history['max_speed_mps'] = max(history['max_speed_mps'], row['sumo_speed_mps'])
                    history['samples'] += 1
        record = {'source_sample': sample, 'source_frame': snapshot.frame,
            'source_timestamp_s': snapshot.timestamp.elapsed_seconds,
            'sumo_time_s': self.traci.simulation.getTime(),
            'phase': 'after complete official sync.tick; no additional SUMO or CARLA step',
            'sumo_to_carla_ids': self.sync.sumo2carla_ids,
            'carla_to_sumo_ids': self.sync.carla2sumo_ids, 'actors': rows,
            'sumo_colliding_vehicle_ids': list(self.traci.simulation.getCollidingVehiclesIDList())}
        self.log.write(json.dumps(record, separators=(',', ':')) + '\n')
        self.log.flush()
        self.count += 1
        return {'sumo_time_s': record['sumo_time_s'], 'sumo_to_carla_ids': dict(self.sync.sumo2carla_ids),
                'carla_to_sumo_ids': dict(self.sync.carla2sumo_ids), 'recorded_in': 'sumo-sync.jsonl'}

    def summary(self):
        return {'recorded_synchronization_frames': self.count, 'sumo_autonomous_actor_observations': self.autonomous_positions,
            'actual_bidirectional_actor_maps_observed': bool(self.sync.sumo2carla_ids and self.sync.carla2sumo_ids),
            'sumo_autonomous_motion_observed': any(v['max_displacement_m'] > 5 and v['max_speed_mps'] > 1 for v in self.autonomous_positions.values()),
            'sumo_sync_jsonl_sha256': sha(self.run / 'sumo-sync.jsonl')}

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.sync is not None:
                # Official close destroys only SUMO-origin CARLA proxies and
                # CARLA-origin SUMO mirrors. Original owned CARLA actors remain
                # for capture_native's single owned-actor cleanup batch.
                self.sync.close()
            else:
                if self.carla is not None:
                    self.carla.close()
                if self.sumo is not None:
                    self.sumo.close()
        finally:
            if self.log is not None:
                self.log.flush()
                os.fsync(self.log.fileno())
                self.log.close()
            self.sync = self.sumo = self.carla = self.world = None
