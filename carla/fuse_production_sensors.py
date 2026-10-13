"""Process newly captured production-style camera/front-LiDAR sensor recordings.

Actual YOLOX-S RGB inference runs independently on four cameras. Only the front
camera receives geometric associations from the native forward 120-degree LiDAR.
Rear/side observations remain camera-only, with independent short-lived IDs.
"""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import sys
import time

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'carla'))
sys.path.insert(0, str(ROOT / 'verification'))
from detect_rgb_onnx import Detector
from fuse_camera_lidar import AssociationTracker, associate, sha
from process_carla_sensors import matrix, project, fit_ground, voxel_representatives
from render_production_fusion import render_views

CAMERAS = {'front': 'rgb', 'rear': 'rear', 'side_left': 'side_left', 'side_right': 'side_right'}
OFFSETS = {'front': 0, 'rear': 10000, 'side_left': 20000, 'side_right': 30000}
THRESHOLDS = {'front': .75, 'rear': .5, 'side_left': .5, 'side_right': .5}
MODULES = ['carla/fuse_production_sensors.py', 'carla/detect_rgb_onnx.py',
           'carla/fuse_camera_lidar.py', 'carla/render_production_fusion.py',
           'verification/process_carla_sensors.py']
FOV_POLICY = {
    'kind': 'engineering_numeric_tolerance',
    'native_horizontal_fov_deg': 120., 'native_upper_fov_deg': 12.9, 'native_lower_fov_deg': -12.5,
    'maximum_boundary_perpendicular_excess_m': .005, 'maximum_angle_excess_deg': .02,
    'minimum_x_m': 0., 'all_limits_required_simultaneously': True,
    'horizontal_ray_spacing_deg': .2, 'vertical_channel_spacing_deg': .2,
    'angular_budget_fraction_of_ray_spacing': .1,
    'rationale': 'Fixed 5 mm engineering boundary budget and one tenth of native ray spacing; '
                 'not a proven PhysX error bound and not fitted to observed extrema.',
    'validation_domain': 'All unmodified native raw points before range, ego or voxel filtering',
    'post_capture_fov_crop': False,
    'native_implementation': 'CARLA 0.9.16 native ImpactPoint is inverse-transformed into sensor '
                             'coordinates and serialized as float32; cumulative ray/hit/transform '
                             'numerics can produce small boundary residuals. No sole error cause is asserted.',
}
FOV_COUNT_FIELDS = ('strict_horizontal_outside_points', 'strict_vertical_outside_points',
                    'engineering_tolerance_exceeded_points')
FOV_MAX_FIELDS = ('max_horizontal_angle_excess_deg', 'max_vertical_angle_excess_deg',
                  'max_horizontal_boundary_excess_m', 'max_vertical_boundary_excess_m')


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate_capture_profile(run, manifest):
    """Accept the new native profile, not an old omnidirectional recording."""
    require(manifest.get('status') == 'complete' and manifest.get('synthetic_sensor_fallback') is False,
            'A completed native capture is required')
    profile = manifest.get('sensor_profile', {})
    require(profile.get('native_mounts_and_blueprint_fov') is True and profile.get('post_capture_fov_crop') is False,
            'Capture must record native sensor-profile mounting/FOV, without a post-capture FOV crop')
    profile_path = run / 'sensor-profile.json'
    require(profile_path.is_file() and sha(profile_path) == profile.get('sha256'), 'Native profile bytes/SHA mismatch')
    require(json.loads(profile_path.read_text(encoding='utf-8-sig')) == profile.get('definition'),
            'Native profile definition differs from capture-side copy')
    for sensor_name in CAMERAS.values():
        sensor = manifest.get('sensors', {}).get(sensor_name, {})
        require(sensor.get('blueprint') == 'sensor.camera.rgb' and sensor.get('role') == 'perception',
                'Missing native perception camera: ' + sensor_name)
        require(sensor_name in profile.get('resolved_cameras', {}), 'Profile has no mount for ' + sensor_name)
    sensor = manifest['sensors'].get('lidar', {})
    require(sensor.get('blueprint') == 'sensor.lidar.ray_cast' and sensor.get('role') == 'perception',
            'Native ray-cast perception LiDAR is required')
    for attributes in (sensor.get('attributes', {}), sensor.get('native_actor_attributes', {}),
                       profile.get('resolved_lidar', {}).get('attributes', {})):
        expected = {'horizontal_fov': 120., 'upper_fov': 12.9, 'lower_fov': -12.5,
                    'channels': 128, 'points_per_second': 1536000, 'rotation_frequency': 20., 'range': 120.}
        for name, value in expected.items():
            require(np.isclose(float(attributes.get(name, -999)), value, atol=1e-6, rtol=0),
                    'All native LiDAR settings must explicitly record '+name+'='+str(value))
        require(np.isclose(float(attributes.get('rotation_frequency', -1)), float(manifest['fps']), atol=1e-6),
                'Native LiDAR must complete one forward FOV sweep per capture frame')
    require(np.isclose(float(sensor['relative_transform']['rotation_deg'][1]), 0., atol=1e-6),
            'Production profile requires forward-facing LiDAR yaw=0')
    return profile_path


def image_and_intrinsic(path, sensor, manifest, sensor_name):
    rgb = np.asarray(Image.open(path).convert('RGB'))
    height, width = rgb.shape[:2]
    require((width, height) == (sensor['width'], sensor['height']), 'Native camera dimensions differ')
    require(rgb.shape == (720, 1280, 3), 'Production renderer requires native 1280x720 cameras')
    bgra = np.concatenate((rgb[:, :, ::-1], np.full((height, width, 1), 255, np.uint8)), axis=2)
    require(hashlib.sha256(bgra.tobytes()).hexdigest() == sensor['raw_sha256'],
            'Native BGRA callback hash differs for ' + sensor_name)
    fov = float(sensor['fov'])
    require(np.isfinite(fov) and 0 < fov < 180, 'Invalid native camera field of view')
    focal = width / (2 * np.tan(np.deg2rad(fov) / 2))
    intrinsic = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1.]])
    for declared in (sensor.get('intrinsic'), manifest['sensors'][sensor_name].get('intrinsic'),
                     manifest.get('camera_intrinsics', {}).get(sensor_name)):
        require(declared is not None and np.asarray(declared).shape == (3, 3)
                and np.allclose(intrinsic, declared, atol=1e-5, rtol=1e-6),
                'Per-camera native intrinsic mismatch for ' + sensor_name)
    if sensor_name == 'rgb':
        require(np.allclose(intrinsic, manifest['camera_intrinsic']), 'Legacy front intrinsic differs')
    return rgb, intrinsic


def validate_native_fov(raw):
    """Check raw returns with a declared metric AND angular engineering budget.

    Plane residuals measure perpendicular distance beyond each nominal boundary.
    This is a boundary acceptance policy, not an error theorem for ray tracing.
    All statistics describe unchanged raw points, including strictly outside rays.
    """
    require(raw.ndim == 2 and raw.shape[1] == 4 and raw.dtype == np.dtype('float32') and len(raw) > 0,
            'LiDAR must be a nonempty native float32 Nx4 array')
    require(np.isfinite(raw).all(), 'Native LiDAR contains nonfinite values')
    x, y, z = raw[:, :3].astype(np.float64).T
    horizontal_range = np.hypot(x, y)
    azimuth = np.degrees(np.arctan2(y, x))
    elevation = np.degrees(np.arctan2(z, horizontal_range))
    half, upper, lower = np.deg2rad([60., 12.9, -12.5])
    horizontal_excess = np.maximum(np.abs(y)*np.cos(half)-x*np.sin(half), 0.)
    upper_excess = np.maximum(z*np.cos(upper)-horizontal_range*np.sin(upper), 0.)
    lower_excess = np.maximum(horizontal_range*np.sin(lower)-z*np.cos(lower), 0.)
    vertical_excess = np.maximum(upper_excess, lower_excess)
    horizontal_angle = np.maximum(np.abs(azimuth)-60., 0.)
    vertical_angle = np.maximum(np.maximum(elevation-12.9, -12.5-elevation), 0.)
    invalid = ((x < 0) | (horizontal_excess > FOV_POLICY['maximum_boundary_perpendicular_excess_m'])
               | (vertical_excess > FOV_POLICY['maximum_boundary_perpendicular_excess_m'])
               | (horizontal_angle > FOV_POLICY['maximum_angle_excess_deg'])
               | (vertical_angle > FOV_POLICY['maximum_angle_excess_deg']))
    metrics = {
        'native_azimuth_min_deg': float(azimuth.min()), 'native_azimuth_max_deg': float(azimuth.max()),
        'native_elevation_min_deg': float(elevation.min()), 'native_elevation_max_deg': float(elevation.max()),
        'native_min_x_m': float(x.min()),
        'strict_horizontal_outside_points': int(np.count_nonzero(horizontal_angle > 0)),
        'strict_vertical_outside_points': int(np.count_nonzero(vertical_angle > 0)),
        'max_horizontal_angle_excess_deg': float(horizontal_angle.max()),
        'max_vertical_angle_excess_deg': float(vertical_angle.max()),
        'max_horizontal_boundary_excess_m': float(horizontal_excess.max()),
        'max_vertical_boundary_excess_m': float(vertical_excess.max()),
        'engineering_tolerance_exceeded_points': int(np.count_nonzero(invalid)),
    }
    require(not np.any(invalid), 'Raw native LiDAR outside forward native FOV engineering limits; '
            'every ray requires x>=0, boundary excess<=0.005m AND angle excess<=0.02deg; '
            'no crop fallback: '+json.dumps(metrics, separators=(',', ':')))
    return metrics


def preprocess_points(raw, row):
    """Index actual rays, using simulator ego geometry only to remove self hits."""
    fov_metrics = validate_native_fov(raw)
    distance = np.linalg.norm(raw[:, :3].astype(float), axis=1)
    keep = (distance >= .5) & (distance <= 120.)
    ego = next((vehicle for vehicle in row['vehicles'] if vehicle['ego']), None)
    require(ego is not None, 'Measured ego pose/bounding box is required for self filtering')
    ego_from_lidar = np.linalg.inv(np.asarray(ego['transform']['world_from_local'], dtype=float)) @ matrix(row['sensors']['lidar'])
    local = (ego_from_lidar @ np.column_stack((raw[:, :3], np.ones(len(raw)))).T).T[:, :3]
    box = ego['bounding_box']
    require(not any(abs(float(value)) > 1e-6 for value in box.get('rotation_rpy_deg', [0, 0, 0])),
            'Rotated ego bounding box requires an explicit box-frame transform')
    inside = np.all(np.abs(local-np.asarray(box['center_local_m'])) <= np.asarray(box['extent_m'])+.15, axis=1)
    kept = np.flatnonzero(keep & ~inside)
    require(len(kept) >= 40, 'Too few native points remain after range and ego filtering')
    sampled = voxel_representatives(raw[kept], kept, .18)
    xyz = raw[sampled, :3]
    normal, offset, ground = fit_ground(xyz, seed=row['frame'])
    metrics = {'raw_points': len(raw), 'kept_points': len(kept), 'sampled_points': len(sampled),
               'ground_points': int(ground.sum()), **fov_metrics}
    return kept, sampled, xyz, ground, np.r_[normal, offset], metrics


def camera_only(detection, camera):
    return dict(copy.deepcopy(detection), camera=camera, fusion_status='camera_only',
                sampled_point_indices=[], raw_point_indices=[], range_m=None,
                observed_center_lidar_m=None, world_observed_center_m=None)


def process(args):
    run, out = args.run.resolve(), args.out.resolve()
    require(out != run and run not in out.parents and out not in run.parents,
            'Output must be separate from preserved native input')
    require(not out.exists(), 'Output directory already exists; preserved results are never overwritten')
    require(args.start_sample >= 0 and (args.end_sample is None or args.end_sample > args.start_sample),
            'Invalid sample selection')
    require(args.id_base >= 0 and args.id_base % 40000 == 0, 'id_base must be a nonnegative multiple of 40000')
    manifest = read(run / 'manifest.json')
    profile_path = validate_capture_profile(run, manifest)
    bindings = {path: sha(path) for path in (run / 'manifest.json', run / 'frames.jsonl', profile_path)}
    modules = {name: sha(ROOT / name) for name in MODULES}
    rows = [json.loads(line) for line in (run / 'frames.jsonl').read_text(encoding='utf-8').splitlines()]
    require(len(rows) == manifest['frames_captured'], 'Capture frame count mismatch')
    require(args.end_sample is None or args.end_sample <= len(rows), 'end_sample exceeds capture length')
    selected = rows[args.start_sample:args.end_sample]
    require(bool(selected), 'No selected native frames')
    for index, row in enumerate(selected):
        require(row['sample'] == args.start_sample+index, 'Native sample sequence differs')
        require(np.isfinite(row['timestamp']), 'Nonfinite native timestamp')
        for name in (*CAMERAS.values(), 'lidar'):
            sensor = row['sensors'][name]
            require(sensor['frame'] == row['frame'] and abs(sensor['timestamp']-row['timestamp']) <= 1e-6,
                    'Native sensor synchronization mismatch for ' + name)
        if index:
            prior = selected[index-1]
            require(row['frame'] == prior['frame']+1 and
                    abs(row['timestamp']-prior['timestamp']-1/manifest['fps']) <= 2e-6,
                    'Selected native frames must be consecutive')
    detector = Detector(args.model, threads=args.threads, confidence=.3)
    require(detector.provenance['matches_known_official_model_sha256'], 'Hash-bound official YOLOX-S model required')
    trackers = {camera: AssociationTracker() for camera in CAMERAS}
    out.mkdir(parents=True, exist_ok=False)
    (out / 'frames').mkdir()
    (out / 'data').mkdir()
    records, geometry_statistics, begin = [], [], time.perf_counter()
    with (out / 'associations.jsonl').open('w', encoding='utf-8') as stream:
        for row in selected:
            frame, timestamp, sensors = row['frame'], row['timestamp'], row['sensors']
            token = f'{frame:08d}'
            images, intrinsics, native_hashes = {}, {}, {}
            for camera, sensor_name in CAMERAS.items():
                path = run / sensor_name / f'{token}.png'
                images[camera], intrinsics[camera] = image_and_intrinsic(path, sensors[sensor_name], manifest, sensor_name)
                key = 'rgb_file_sha256' if camera == 'front' else camera+'_rgb_file_sha256'
                native_hashes[key] = sha(path)
            raw = np.load(run / 'lidar' / f'{token}.npy', allow_pickle=False)
            raw_sha = hashlib.sha256(raw.astype('<f4', copy=False).tobytes()).hexdigest()
            require(raw_sha == sensors['lidar']['raw_sha256'], 'Native LiDAR callback hash differs')
            kept, sampled, xyz, ground, plane, metrics = preprocess_points(raw, row)
            projected, uv, depth = project(xyz, matrix(sensors['lidar']), matrix(sensors['rgb']),
                                           intrinsics['front'], images['front'].shape[1], images['front'].shape[0])
            data_path = out / 'data' / f'{token}.npz'
            np.savez_compressed(data_path, kept_raw_indices=kept, sampled_raw_indices=sampled,
                                ground_mask=ground, ground_plane_abcd=plane,
                                projection_sample_indices=projected, projection_uv=uv, projection_depth_m=depth)
            projections = {camera: (np.empty(0, dtype=int), np.empty((0, 2))) for camera in CAMERAS}
            projections['front'] = (projected, uv)
            detections, inference_ms, raw_detections = [], {}, {}
            for camera in CAMERAS:
                tick = time.perf_counter()
                raw_detections[camera] = detector.detect(images[camera])
                inference_ms[camera] = (time.perf_counter()-tick)*1000
                accepted = [item for item in raw_detections[camera] if item['score'] >= THRESHOLDS[camera]]
                if camera == 'front':
                    current = associate(accepted, xyz, sampled, ground, projected, uv, depth)
                    for item in current:
                        item['camera'] = camera
                else:
                    current = [camera_only(item, camera) for item in accepted]
                trackers[camera].update(current, timestamp, matrix(sensors['lidar']))
                for item in current:
                    local_id = item['display_track_id']
                    require(local_id < 10000, 'Camera ID namespace exhausted')
                    item['track_id_local'] = local_id
                    item['display_track_id'] = args.id_base+OFFSETS[camera]+local_id
                    item['tracking_scope'] = 'same_camera_only'
                detections.extend(current)
            owned = [index for item in detections for index in item['raw_point_indices']]
            require(len(owned) == len(set(owned)), 'Measured LiDAR point has multiple displayed owners')
            pictures = render_views(images, xyz, ground, projections, detections)
            require(set(pictures) == {'camera', 'rear', 'side_left', 'side_right', 'lidar'}, 'Renderer output differs')
            rendered_hashes = {}
            for name, picture in pictures.items():
                path = out / 'frames' / f'{token}-{name}.png'
                picture.save(path, compress_level=3)
                rendered_hashes[name+'_sha256'] = sha(path)
            native_sensor_bindings = {name: {'frame': sensors[name]['frame'], 'timestamp': sensors[name]['timestamp'],
                'raw_sha256': sensors[name]['raw_sha256']} for name in (*CAMERAS.values(), 'lidar')}
            record = {'sample': row['sample'], 'frame': frame, 'timestamp': timestamp,
                      **native_hashes, 'native_lidar_raw_sha256': raw_sha,
                      'geometric_points_npz_sha256': sha(data_path), **rendered_hashes,
                      'native_sensor_bindings': native_sensor_bindings,
                      'camera_intrinsics': {name: value.tolist() for name, value in intrinsics.items()},
                      'inference_ms': inference_ms, 'raw_detections': raw_detections,
                      'detections': detections, 'geometry_statistics': metrics}
            stream.write(json.dumps(record, separators=(',', ':'))+'\n')
            stream.flush()
            records.append(record)
            geometry_statistics.append(metrics)
            if len(records) % 50 == 0 or len(records) == len(selected):
                print(json.dumps({'processed': len(records), 'total': len(selected),
                                  'elapsed_s': round(time.perf_counter()-begin, 1)}), flush=True)
    require(all(sha(path) == digest for path, digest in bindings.items()), 'Bound native input changed during processing')
    require(all(sha(ROOT/path) == digest for path, digest in modules.items()), 'Source module changed during processing')
    source_fields = {'source_run': str(run), 'source_manifest_sha256': bindings[run/'manifest.json'],
                     'source_frames_sha256': bindings[run/'frames.jsonl'],
                     'source_sensor_profile_sha256': bindings[profile_path],
                     'native_frame_ids': [row['frame'] for row in records]}
    processing = {'status': 'processed_actual_capture_audit_pending', **source_fields,
                  'source_frames_jsonl_sha256': bindings[run/'frames.jsonl'], 'script_sha256': modules[MODULES[0]],
                  'frames': len(records), 'voxel_m': .18, 'range_interval_m': [.5, 120.],
                  'ground_threshold_m': .18, 'ego_skin_margin_m': .15,
                  'ground_method': 'Seeded measured-point RANSAC and SVD refinement; no fallback',
                  'source_modules_sha256': modules, 'point_processing_uses_native_lidar': True,
                  'image_processing_uses_native_ground_truth': False, 'generated_points': False,
                  'native_lidar_horizontal_fov_deg': 120., 'post_capture_fov_crop': False,
                  'native_fov_validation_policy': FOV_POLICY,
                  'ego_geometry_used_only_for_self_filter': True, 'other_vehicle_ground_truth_used': False,
                  'native_azimuth_min_deg': min(row['native_azimuth_min_deg'] for row in geometry_statistics),
                  'native_azimuth_max_deg': max(row['native_azimuth_max_deg'] for row in geometry_statistics),
                  'native_elevation_min_deg': min(row['native_elevation_min_deg'] for row in geometry_statistics),
                  'native_elevation_max_deg': max(row['native_elevation_max_deg'] for row in geometry_statistics),
                  **{key: sum(row[key] for row in geometry_statistics) for key in FOV_COUNT_FIELDS},
                  **{key: max(row[key] for row in geometry_statistics) for key in FOV_MAX_FIELDS},
                  'native_min_x_m': min(row['native_min_x_m'] for row in geometry_statistics)}
    (out/'processing-manifest.json').write_text(json.dumps(processing, indent=2)+'\n', encoding='utf-8')
    statistics = {}
    for camera in CAMERAS:
        frames = [[item for item in row['detections'] if item['camera'] == camera] for row in records]
        statistics[camera] = {'detected_boxes': sum(map(len, frames)),
            'associated_boxes': sum(item['fusion_status'] == 'associated' for items in frames for item in items),
            'frames_with_detection': sum(bool(items) for items in frames),
            'frames_with_association': sum(any(item['fusion_status'] == 'associated' for item in items) for items in frames),
            'median_inference_ms': float(np.median([row['inference_ms'][camera] for row in records]))}
    summary = {'status': 'processed_pending_independent_review', **source_fields,
        'source_processed': str(out), 'source_processing_manifest_sha256': sha(out/'processing-manifest.json'),
        'frames': len(records), 'associations_sha256': sha(out/'associations.jsonl'),
        'source_modules_sha256': modules, 'detector_provenance': detector.provenance,
        'model_sha256': sha(args.model), 'inference_confidence': .3, 'display_confidence': THRESHOLDS,
        'derived_png_compression': 3, 'derived_png_encoding_lossless': True,
        'native_fov_validation_policy': FOV_POLICY,
        'statistics': statistics, 'display_track_id_base': args.id_base, 'camera_id_offsets': OFFSETS,
        'tracking_scope': 'Independent per-camera short-term observation association; no cross-camera ReID',
        'cross_camera_reidentification': False, 'trained_fusion_model': False,
        'trained_image_detector_inference': True, 'uses_simulator_labels_for_detection_or_association': False,
        'ego_geometry_used_only_for_self_filter': True, 'offline_processing': True,
        'processing_controls_vehicle': False, 'camera_and_lidar_visible_every_frame': True,
        'sensor_layout_claim': 'Generic production-style reference; not an exact OEM sensor replica',
        'limits': ['New native captures from the declared physical mounts and forward 120-degree LiDAR FOV.',
                  'Only the front camera receives geometric measured-point association and surface range.',
                  'Rear and side camera detections have no LiDAR points or range; no rear depth is invented.',
                  'Each camera has independent short-term IDs; no cross-camera or cross-scene ReID is claimed.',
                  'Pretrained image detector and geometric late fusion, not a jointly trained fusion model.',
                  'Offline processing; native 20 fps playback does not imply real-time inference.',
                  'Front 0.75 and rear/side 0.50 confidence thresholds are development settings, not accuracy validation.',
                  'Generic production-style mounting coordinates are not an exact OEM installation specification.',
                  'Surface distances and flat-road ground estimation have occlusion and sparse-return limitations.']}
    (out/'fusion-manifest.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'status': summary['status'], 'frames': len(records), 'statistics': statistics}), flush=True)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    for name in ('run', 'out', 'model'):
        result.add_argument('--'+name, type=Path, required=True)
    result.add_argument('--threads', type=int, default=3)
    result.add_argument('--start-sample', type=int, default=0)
    result.add_argument('--end-sample', type=int)
    result.add_argument('--id-base', type=int, default=0,
                        help='Optional per-scene namespace, in multiples of 40000; camera offsets remain fixed')
    return result


if __name__ == '__main__':
    process(parser().parse_args())
