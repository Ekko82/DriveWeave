"""Independent provenance/geometry audit for four cameras and native front LiDAR.

No processor, renderer, detector, tracker, or earlier audit is imported. Native
camera matrices are solved directly. Only ego geometry is read for checking the
documented self filter; other vehicle labels and semantic masks are never read.
Passing this audit does not establish detection accuracy or visual quality.
"""
from pathlib import Path
import argparse
import collections
import hashlib
import json
import math

import numpy as np
from PIL import Image

CAMERAS = {'front': 'rgb', 'rear': 'rear', 'side_left': 'side_left', 'side_right': 'side_right'}
OFFSETS = {'front': 0, 'rear': 10000, 'side_left': 20000, 'side_right': 30000}
MODEL_SHA256 = 'c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063'
REQUIRED_MODULES = {'carla/fuse_production_sensors.py', 'carla/detect_rgb_onnx.py',
                    'carla/fuse_camera_lidar.py', 'carla/render_production_fusion.py',
                    'verification/process_carla_sensors.py'}
COCO_CLASSES = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}
FOV_BOUNDARY_BUDGET_M = .005
FOV_ANGLE_BUDGET_DEG = .02


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def read_rows(path):
    with Path(path).open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


class Checks:
    def __init__(self):
        self.counts = {'assertions': 0, 'passed_assertions': 0, 'failed_assertions': 0}
        self.failures = []

    def __call__(self, condition, message):
        self.counts['assertions'] += 1
        if bool(condition):
            self.counts['passed_assertions'] += 1
        else:
            self.counts['failed_assertions'] += 1
            if len(self.failures) < 100:
                self.failures.append(message)
        return bool(condition)


def close(a, b, atol=1e-6, rtol=1e-7):
    try:
        aa, bb = np.asarray(a, float), np.asarray(b, float)
        return aa.shape == bb.shape and np.isfinite(aa).all() and np.isfinite(bb).all() and np.allclose(aa, bb, atol=atol, rtol=rtol)
    except (TypeError, ValueError):
        return False


def matrix_from_pose(location, pitch_yaw_roll):
    """CARLA x-forward/y-right/z-up transform from its recorded pose fields."""
    pitch, yaw, roll = np.radians(np.asarray(pitch_yaw_roll, float))
    cp, sp, cy, sy, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll)
    transform = np.eye(4)
    transform[:3, :3] = [[cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
                         [cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
                         [sp, -cp * sr, cp * cr]]
    transform[:3, 3] = location
    return transform


def intrinsic(width, height, fov):
    focal = width / (2 * math.tan(math.radians(float(fov)) / 2))
    return np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1.]])


def independent_projection(xyz, world_from_lidar, world_from_camera, k, width, height):
    homogeneous = np.column_stack((np.asarray(xyz, float), np.ones(len(xyz))))
    local = np.linalg.solve(np.asarray(world_from_camera, float),
                            np.asarray(world_from_lidar, float) @ homogeneous.T).T[:, :3]
    optical = np.column_stack((local[:, 1], -local[:, 2], local[:, 0]))
    depth = optical[:, 2]
    projected = optical @ np.asarray(k, float).T
    uv = np.full((len(xyz), 2), np.nan)
    positive = depth > .1
    uv[positive] = projected[positive, :2] / depth[positive, None]
    visible = positive & np.isfinite(uv).all(axis=1)
    visible &= (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)
    return visible, uv, depth


def check_native_sector(xyz, lower_deg, upper_deg, checks, label):
    """Fixed development budget for returned hits, not a floating-point theorem.

    The producer uses signed boundary-plane dot products. Here an independent
    polar reconstruction computes perpendicular excess as radius*sin(angle
    excess). Both the 5mm AND 0.02-degree budgets must pass. The latter is one
    tenth of the profile's nominal 0.2-degree ray spacing. No point is cropped,
    moved, or added; strict nominal-boundary deviations remain reported.
    """
    xyz = np.asarray(xyz, float)
    horizontal_radius = np.hypot(xyz[:, 0], xyz[:, 1])
    radius = np.linalg.norm(xyz, axis=1)
    azimuth = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
    elevation = np.degrees(np.arctan2(xyz[:, 2], horizontal_radius))
    horizontal_angle_excess = np.maximum(abs(azimuth) - 60., 0.)
    vertical_angle_excess = np.maximum.reduce([np.zeros(len(xyz)),
                                               elevation - upper_deg, lower_deg - elevation])
    horizontal_excess_m = horizontal_radius * np.sin(np.radians(horizontal_angle_excess))
    vertical_excess_m = radius * np.sin(np.radians(vertical_angle_excess))
    invalid = ((xyz[:, 0] < 0) | (horizontal_angle_excess > FOV_ANGLE_BUDGET_DEG) |
               (vertical_angle_excess > FOV_ANGLE_BUDGET_DEG) |
               (horizontal_excess_m > FOV_BOUNDARY_BUDGET_M) | (vertical_excess_m > FOV_BOUNDARY_BUDGET_M))
    checks((xyz[:, 0] >= 0).all(), label + ': native negative-x ray is forbidden')
    checks((horizontal_angle_excess <= FOV_ANGLE_BUDGET_DEG).all() and
           (horizontal_excess_m <= FOV_BOUNDARY_BUDGET_M).all(),
           label + ': native horizontal FOV engineering budget exceeded')
    checks((vertical_angle_excess <= FOV_ANGLE_BUDGET_DEG).all() and
           (vertical_excess_m <= FOV_BOUNDARY_BUDGET_M).all(),
           label + ': native vertical FOV engineering budget exceeded')
    return {'native_azimuth_min_deg': float(azimuth.min()), 'native_azimuth_max_deg': float(azimuth.max()),
            'native_elevation_min_deg': float(elevation.min()), 'native_elevation_max_deg': float(elevation.max()),
            'native_min_x_m': float(xyz[:, 0].min()),
            'engineering_tolerance_exceeded_points': int(invalid.sum()),
            'strict_horizontal_outside_points': int((horizontal_angle_excess > 0).sum()),
            'strict_vertical_outside_points': int((vertical_angle_excess > 0).sum()),
            'max_horizontal_angle_excess_deg': float(horizontal_angle_excess.max()),
            'max_vertical_angle_excess_deg': float(vertical_angle_excess.max()),
            'max_horizontal_boundary_excess_m': float(horizontal_excess_m.max()),
            'max_vertical_boundary_excess_m': float(vertical_excess_m.max())}


def check_fov_policy(policy, lidar_attributes, fps, checks):
    """Verify declared budgets against this audit's independent fixed policy."""
    checks(policy.get('kind') == 'engineering_numeric_tolerance', 'Native FOV policy kind differs')
    horizontal_step = 120. / (float(lidar_attributes['points_per_second']) / fps /
                              int(lidar_attributes['channels']))
    vertical_step = ((float(lidar_attributes['upper_fov']) - float(lidar_attributes['lower_fov'])) /
                     (int(lidar_attributes['channels']) - 1))
    for key, value in {'native_horizontal_fov_deg': 120.,
                       'native_upper_fov_deg': float(lidar_attributes['upper_fov']),
                       'native_lower_fov_deg': float(lidar_attributes['lower_fov']),
                       'maximum_boundary_perpendicular_excess_m': FOV_BOUNDARY_BUDGET_M,
                       'maximum_angle_excess_deg': FOV_ANGLE_BUDGET_DEG, 'minimum_x_m': 0.,
                       'horizontal_ray_spacing_deg': horizontal_step, 'vertical_channel_spacing_deg': vertical_step,
                       'angular_budget_fraction_of_ray_spacing': .1}.items():
        checks(close(policy.get(key), value, atol=1e-12), 'Native FOV policy differs: ' + key)
    checks(policy.get('all_limits_required_simultaneously') is True and
           policy.get('post_capture_fov_crop') is False, 'Native FOV policy must require all limits without cropping')


def check_transform(transform, checks, label):
    value = np.asarray(transform, float)
    if not checks(value.shape == (4, 4) and np.isfinite(value).all(), label + ': invalid transform'):
        raise ValueError('Invalid transform shape/values')
    checks(close(value[3], [0, 0, 0, 1]), label + ': invalid homogeneous row')
    rotation = value[:3, :3]
    checks(close(rotation.T @ rotation, np.eye(3), atol=2e-5) and abs(np.linalg.det(rotation) - 1) < 2e-5,
           label + ': rotation is not rigid')
    return value


def check_profile(run, manifest, checks):
    profile = manifest.get('sensor_profile', {})
    path = run / 'sensor-profile.json'
    checks(profile.get('native_mounts_and_blueprint_fov') is True and
           profile.get('post_capture_fov_crop') is False, 'Native profile declaration differs')
    checks(sha(path) == profile.get('sha256'), 'Capture-side profile SHA differs')
    definition = json.loads(path.read_text(encoding='utf-8-sig'))
    checks(definition == profile.get('definition'), 'Profile bytes/declared definition differ')
    checks(definition.get('schema_version') == 1, 'Unknown sensor profile schema')
    checks(set(definition['cameras']) == set(CAMERAS.values()), 'Profile requires exactly four perception cameras')
    checks(set(profile.get('resolved_cameras', {})) == set(CAMERAS.values()), 'Resolved profile camera set differs')
    relative = {}
    for name in (*CAMERAS.values(), 'lidar'):
        sensor = manifest['sensors'][name]
        declared = definition['lidar'] if name == 'lidar' else definition['cameras'][name]
        resolved = profile['resolved_lidar'] if name == 'lidar' else profile['resolved_cameras'][name]
        checks(sensor.get('role') == 'perception', name + ': native perception role missing')
        checks(sensor.get('blueprint') == ('sensor.lidar.ray_cast' if name == 'lidar' else 'sensor.camera.rgb'),
               name + ': native blueprint differs')
        expected = matrix_from_pose(declared['location_m'], declared['rotation_deg'])
        native = sensor['relative_transform']
        checks(close(declared['location_m'], resolved['location_m']) and
               close(declared['rotation_deg'], resolved['rotation_deg']), name + ': resolved profile mounting differs')
        checks(close(native['location_m'], declared['location_m']) and
               close(matrix_from_pose(native['location_m'], native['rotation_deg']), expected, atol=2e-5),
               name + ': native relative pose differs from profile')
        relative[name] = check_transform(native['world_from_local'], checks, name + ' relative')
        checks(close(relative[name], expected, atol=2e-5), name + ': native relative matrix differs from profile')
        if name != 'lidar':
            width = int(declared.get('width', manifest['settings']['width']))
            height = int(declared.get('height', manifest['settings']['height']))
            fov = float(declared['fov'])
            checks((width, height) == (1280, 720) and 0 < fov < 180, name + ': profile camera dimensions/FOV invalid')
            checks(resolved['width'] == width and resolved['height'] == height and resolved['fov'] == fov,
                   name + ': resolved camera settings differ')
            for attributes in (sensor['attributes'], sensor['native_actor_attributes']):
                for key, value in [('image_size_x', width), ('image_size_y', height), ('fov', fov),
                                   ('sensor_tick', 0), ('lens_k', 0), ('lens_kcube', 0)]:
                    checks(close(float(attributes.get(key, float('nan'))), value), name + ': native attribute differs ' + key)
            k = intrinsic(width, height, fov)
            checks(close(sensor.get('intrinsic'), k, atol=1e-5) and
                   close(manifest.get('camera_intrinsics', {}).get(name), k, atol=1e-5),
                   name + ': manifest intrinsic differs from native FOV')
            if name == 'rgb':
                checks(close(manifest.get('camera_intrinsic'), k, atol=1e-5), 'Legacy front K differs')
    attributes = definition['lidar']['attributes']
    checks(close(attributes['horizontal_fov'], 120), 'Requested LiDAR FOV is not 120 degrees')
    checks(close(definition['lidar']['rotation_deg'], [0, 0, 0]), 'LiDAR is not forward mounted')
    checks(manifest['fps'] == 20 and close(manifest['fixed_delta_seconds'], .05), 'Native capture is not fixed 20 fps')
    for values in (manifest['sensors']['lidar']['attributes'],
                   manifest['sensors']['lidar']['native_actor_attributes'], profile['resolved_lidar']['attributes']):
        for name, value in attributes.items():
            checks(close(float(values.get(name, float('nan'))), value, atol=1e-5), 'LiDAR native attribute differs: ' + name)
        checks(close(float(values.get('sensor_tick', float('nan'))), 0), 'LiDAR must tick every native frame')
        checks(close(float(values.get('rotation_frequency', float('nan'))) * manifest['fixed_delta_seconds'], 1),
               'LiDAR does not request exactly one FOV sweep per frame')
    return relative, attributes


def check_sensor_image(run, name, row, manifest, relative, ego_world, checks):
    frame, sensor = row['frame'], row['sensors'][name]
    label = f'{name} {frame}'
    path = run / name / f'{frame:08d}.png'
    with Image.open(path) as image:
        checks(image.mode == 'RGB', label + ': source PNG is not RGB')
        rgb = np.asarray(image.convert('RGB'))
    height, width = rgb.shape[:2]
    checks((width, height) == (sensor['width'], sensor['height']) == (1280, 720), label + ': image dimensions differ')
    bgra = np.empty((height, width, 4), np.uint8)
    bgra[:, :, :3], bgra[:, :, 3] = rgb[:, :, ::-1], 255
    checks(hashlib.sha256(bgra.tobytes()).hexdigest() == sensor['raw_sha256'], label + ': raw BGRA callback SHA differs')
    fov = float(sensor['fov'])
    checks(close(fov, float(manifest['sensors'][name]['attributes']['fov'])), label + ': frame FOV differs')
    k = intrinsic(width, height, fov)
    checks(close(sensor.get('intrinsic'), k, atol=1e-5), label + ': frame intrinsic differs')
    world = check_transform(sensor['transform']['world_from_local'], checks, label)
    checks(close(world[:3, 3], sensor['transform']['location_m'], atol=1e-5), label + ': location/matrix differ')
    checks(close(world, matrix_from_pose(sensor['transform']['location_m'], sensor['transform']['rotation_deg']), atol=2e-5),
           label + ': native pose/matrix differ')
    attached = np.linalg.solve(ego_world, world)
    checks(close(attached[:3, :3], relative[name][:3, :3], atol=2e-5) and
           close(attached[:3, 3], relative[name][:3, 3], atol=.002, rtol=0),
           label + ': per-frame rigid mounting differs')
    return sha(path), k, world


def check_points(raw, row, manifest, data, lidar_attributes, checks):
    """Inspect every raw ray before applying any derived selection."""
    frame = row['frame']
    if not checks(raw.ndim == 2 and raw.shape[1] == 4 and raw.dtype == np.dtype('float32') and len(raw) > 0,
                  f'Invalid native float32 Nx4 LiDAR {frame}'):
        raise ValueError('Invalid native LiDAR shape/type')
    checks(np.isfinite(raw).all(), f'Nonfinite native LiDAR {frame}')
    xyz_all = raw[:, :3]
    sector = check_native_sector(xyz_all, float(lidar_attributes['lower_fov']),
                                 float(lidar_attributes['upper_fov']), checks, f'LiDAR {frame}')
    per_channel_budget = math.floor(float(lidar_attributes['points_per_second']) /
                                    manifest['fps'] / int(lidar_attributes['channels']) + .5)
    sensor = row['sensors']['lidar']
    checks(sensor['points'] == len(raw), f'Native LiDAR point count differs {frame}')
    checks(sensor['channels'] == int(lidar_attributes['channels']), f'Native LiDAR channel count differs {frame}')
    checks(len(sensor['counts_per_channel']) == sensor['channels'] and
           sum(sensor['counts_per_channel']) == len(raw) and
           all(type(n) is int and 0 <= n <= per_channel_budget for n in sensor['counts_per_channel']),
           f'Native channel point counts differ from single-sweep budget {frame}')
    indices, kept = data['sampled_raw_indices'], data['kept_raw_indices']
    for name, values in [('sampled', indices), ('kept', kept)]:
        valid = values.ndim == 1 and np.issubdtype(values.dtype, np.integer)
        valid = valid and np.all(values >= 0) and np.all(values < len(raw))
        if not checks(valid and len(np.unique(values)) == len(values), f'Invalid {name} raw indices {frame}'):
            raise ValueError('Invalid point index domain')
    ego = next(v for v in row['vehicles'] if v.get('ego') is True)
    shape = ego['bounding_box']
    checks(close(shape.get('rotation_rpy_deg', [0, 0, 0]), [0, 0, 0]), f'Unsupported rotated ego box {frame}')
    ego_world = np.asarray(ego['transform']['world_from_local'], float)
    lidar_world = np.asarray(sensor['transform']['world_from_local'], float)
    world = lidar_world @ np.column_stack((xyz_all, np.ones(len(raw)))).T
    ego_points = np.linalg.solve(ego_world, world).T[:, :3]
    inside = (abs(ego_points - np.asarray(shape['center_local_m'])) <= np.asarray(shape['extent_m']) + .15).all(axis=1)
    ranges = np.linalg.norm(xyz_all.astype(float), axis=1)
    expected_kept = np.flatnonzero((ranges >= .5) & (ranges <= 120) & ~inside)
    checks(np.array_equal(kept, expected_kept), f'Range/ego self-filter selection differs {frame}')
    checks(np.isin(indices, kept).all(), f'Sampled points absent from kept native rows {frame}')
    # Independently inspect first occurrences of the declared float32 0.18m voxels.
    seen, representatives = set(), []
    for raw_index, cell in zip(kept, np.floor(xyz_all[kept] / np.float32(.18)).astype(np.int64)):
        key = tuple(map(int, cell))
        if key not in seen:
            seen.add(key)
            representatives.append(int(raw_index))
    checks(np.array_equal(indices, representatives), f'Native voxel representatives differ {frame}')
    xyz, ground, plane = xyz_all[indices], data['ground_mask'], data['ground_plane_abcd']
    checks(plane.shape == (4,) and np.isfinite(plane).all() and abs(np.linalg.norm(plane[:3]) - 1) < 1e-6 and
           plane[2] >= math.cos(math.radians(15)), f'Invalid measured ground plane {frame}')
    checks(ground.dtype == np.bool_ and ground.shape == (len(xyz),), f'Invalid ground mask {frame}')
    checks(np.array_equal(abs(xyz.astype(float) @ plane[:3] + plane[3]) < .18, ground),
           f'Ground residual mask differs {frame}')
    metrics = {'raw_points': len(raw), 'kept_points': len(kept), 'sampled_points': len(indices),
               'ground_points': int(ground.sum()), **sector}
    return xyz, ground, indices, metrics


def core(item):
    return {key: item[key] for key in ('bbox_xyxy', 'class_id', 'class_name', 'score')}


def check_detections(record, summary, xyz, ground, indices, projection, lidar_world, checks):
    frame, used, visible_ids = record['frame'], set(), set()
    visible, uv, depth = projection
    counters = {camera: collections.Counter() for camera in CAMERAS}
    for camera in CAMERAS:
        accepted = [core(item) for item in record['raw_detections'][camera]
                    if item['score'] >= summary['display_confidence'][camera]]
        actual = [core(item) for item in record['detections'] if item['camera'] == camera]
        canonical = lambda items: sorted(json.dumps(item, sort_keys=True) for item in items)
        checks(canonical(accepted) == canonical(actual), f'Display detections differ from thresholded RGB results {camera} {frame}')
        for item in record['raw_detections'][camera]:
            box = np.asarray(item['bbox_xyxy'], float)
            checks(box.shape == (4,) and np.isfinite(box).all() and
                   0 <= box[0] < box[2] <= 1280 and 0 <= box[1] < box[3] <= 720,
                   f'Invalid original detector box {camera} {frame}')
            checks(np.isfinite(item['score']) and summary['inference_confidence'] <= item['score'] <= 1,
                   f'Invalid original detector score {camera} {frame}')
            checks(item['class_id'] in COCO_CLASSES and COCO_CLASSES.get(item['class_id']) == item['class_name'],
                   f'Invalid detector class {camera} {frame}')
    for item in record['detections']:
        camera, identifier = item.get('camera'), item.get('display_track_id')
        if not checks(camera in CAMERAS, f'Unknown camera detection {frame}'):
            continue
        counters[camera]['detected_boxes'] += 1
        counters[camera][str(item.get('fusion_status')) + '_boxes'] += 1
        local = item.get('track_id_local')
        checks(type(local) is int and 0 < local < 10000 and type(identifier) is int and
               identifier == summary['display_track_id_base'] + OFFSETS[camera] + local,
               f'Independent camera ID namespace differs {camera} {frame}')
        checks(identifier not in visible_ids, f'Repeated visible display ID {frame}')
        visible_ids.add(identifier)
        checks(item.get('tracking_scope') == 'same_camera_only', f'Unexpected tracking scope {camera} {frame}')
        ids, raw_ids = np.asarray(item['sampled_point_indices'], int), np.asarray(item['raw_point_indices'], int)
        checks(all(type(i) is int for i in item['sampled_point_indices'] + item['raw_point_indices']),
               f'Noninteger associated point index {camera} {frame}')
        if camera != 'front':
            checks(item.get('fusion_status') == 'camera_only', f'Non-front camera claims LiDAR association {camera} {frame}')
        else:
            checks(item.get('fusion_status') in ('associated', 'camera_only', 'ambiguous_depth'),
                   f'Unknown front fusion status {frame}')
        if item.get('fusion_status') != 'associated':
            checks(not len(ids) and not len(raw_ids) and all(item.get(key) is None for key in
                   ('range_m', 'observed_center_lidar_m', 'world_observed_center_m',
                    'observed_bounds_min_m', 'observed_bounds_max_m', 'association_score')) and
                   item.get('association_support_points', 0) in (None, 0),
                   f'Camera-only/ambiguous target contains fabricated geometry {camera} {frame}')
            continue
        checks(camera == 'front', f'Only front may own measured points {frame}')
        if not checks(len(ids) >= 6 and np.all(ids >= 0) and np.all(ids < len(xyz)) and
                      len(np.unique(ids)) == len(ids), f'Invalid associated sampled indices {camera} {frame}'):
            continue
        checks(np.array_equal(indices[ids], raw_ids), f'Associated native point mapping differs {frame}')
        checks(not used.intersection(raw_ids.tolist()), f'Native point has multiple owners {frame}')
        used.update(raw_ids.tolist())
        checks(not ground[ids].any() and visible[ids].all() and (depth[ids] > .1).all(),
               f'Ground, invisible or behind-camera point associated {frame}')
        box, pixels = item['bbox_xyxy'], uv[ids]
        checks(((pixels[:, 0] >= box[0]) & (pixels[:, 0] <= box[2]) &
                (pixels[:, 1] >= box[1]) & (pixels[:, 1] <= box[3])).all(),
               f'Associated point is outside the front detection box {frame}')
        points, center = xyz[ids], np.median(xyz[ids], axis=0)
        checks(close(item.get('range_m'), np.median(np.linalg.norm(points, axis=1)), atol=1e-5),
               f'Measured surface range differs {frame}')
        checks(close(item.get('observed_center_lidar_m'), center), f'Measured center differs {frame}')
        checks(close(item.get('observed_bounds_min_m'), points.min(axis=0)) and
               close(item.get('observed_bounds_max_m'), points.max(axis=0)), f'Measured bounds differ {frame}')
        checks(close(item.get('world_observed_center_m'), (lidar_world @ np.r_[center, 1])[:3]),
               f'Measured world center differs {frame}')
        checks(item.get('association_support_points') == len(ids), f'Measured point support count differs {frame}')
        counters[camera]['associated_native_points'] += len(ids)
    return counters


def audit(run, fusion, stride=1):
    if stride < 1:
        raise ValueError('stride must be a positive integer')
    run, fusion = Path(run).resolve(), Path(fusion).resolve()
    auditor_sha = sha(__file__)
    checks = Checks()
    manifest, summary = read_json(run / 'manifest.json'), read_json(fusion / 'fusion-manifest.json')
    processing = read_json(fusion / 'processing-manifest.json')
    rows, records = read_rows(run / 'frames.jsonl'), read_rows(fusion / 'associations.jsonl')
    paths = {'capture_manifest': run / 'manifest.json', 'capture_frames': run / 'frames.jsonl',
             'sensor_profile': run / 'sensor-profile.json', 'processing_manifest': fusion / 'processing-manifest.json',
             'fusion_manifest': fusion / 'fusion-manifest.json', 'associations': fusion / 'associations.jsonl'}
    hashes = {key: sha(path) for key, path in paths.items()}
    checks(manifest.get('status') == 'complete' and manifest.get('synthetic_sensor_fallback') is False,
           'Completed native capture required')
    checks(len(rows) == manifest['frames_captured'], 'Native capture row count differs')
    checks(len({row['frame'] for row in rows}) == len(rows), 'Native capture contains repeated frame IDs')
    relative, lidar_attributes = check_profile(run, manifest, checks)
    check_fov_policy(processing.get('native_fov_validation_policy', {}), lidar_attributes, manifest['fps'], checks)
    checks(summary.get('native_fov_validation_policy') == processing.get('native_fov_validation_policy'),
           'Processing/fusion native FOV policy differs')
    for key, expected in [('source_manifest_sha256', hashes['capture_manifest']),
                          ('source_frames_sha256', hashes['capture_frames']),
                          ('source_sensor_profile_sha256', hashes['sensor_profile'])]:
        checks(summary.get(key) == processing.get(key) == expected, 'Source provenance differs: ' + key)
    checks(summary.get('source_processing_manifest_sha256') == hashes['processing_manifest'], 'Processing manifest SHA differs')
    checks(processing.get('source_frames_jsonl_sha256') == hashes['capture_frames'], 'Processing native-frame SHA differs')
    checks(summary.get('associations_sha256') == hashes['associations'], 'Associations SHA differs')
    declarations = {'trained_image_detector_inference': True, 'trained_fusion_model': False,
                    'uses_simulator_labels_for_detection_or_association': False,
                    'ego_geometry_used_only_for_self_filter': True, 'offline_processing': True,
                    'processing_controls_vehicle': False, 'cross_camera_reidentification': False}
    for key, expected in declarations.items():
        checks(summary.get(key) is expected, 'Fusion scope declaration differs: ' + key)
    for key, expected in {'point_processing_uses_native_lidar': True, 'image_processing_uses_native_ground_truth': False,
                          'generated_points': False, 'post_capture_fov_crop': False,
                          'ego_geometry_used_only_for_self_filter': True, 'other_vehicle_ground_truth_used': False}.items():
        checks(processing.get(key) is expected, 'Processing scope declaration differs: ' + key)
    for key, expected in [('voxel_m', .18), ('range_interval_m', [.5, 120.]),
                          ('ground_threshold_m', .18), ('ego_skin_margin_m', .15),
                          ('native_lidar_horizontal_fov_deg', 120.)]:
        checks(close(processing.get(key), expected), 'Processing parameter differs: ' + key)
    provenance = summary.get('detector_provenance', {})
    checks(summary.get('model_sha256') == provenance.get('model_sha256') == MODEL_SHA256 and
           provenance.get('matches_known_official_model_sha256') is True, 'Official YOLOX model provenance differs')
    checks(provenance.get('ground_truth_used_for_inference') is False and provenance.get('lidar_used_for_detection') is False,
           'RGB-only image inference declaration differs')
    checks(close(summary.get('inference_confidence'), .3), 'Image inference confidence differs')
    checks(summary.get('display_confidence') == {'front': .75, 'rear': .5, 'side_left': .5, 'side_right': .5},
           'Declared development display thresholds differ')
    checks(summary.get('camera_id_offsets') == OFFSETS, 'Camera ID namespace offsets differ')
    checks(summary.get('derived_png_compression') == 3 and summary.get('derived_png_encoding_lossless') is True,
           'Derived PNG lossless encoding declaration differs')
    base = summary.get('display_track_id_base')
    checks(type(base) is int and base >= 0 and base % 40000 == 0, 'Invalid scene ID base')
    root = Path(__file__).resolve().parents[1]
    modules = summary.get('source_modules_sha256', {})
    checks(isinstance(modules, dict) and REQUIRED_MODULES <= set(modules), 'Required source module SHA mapping missing')
    checks(processing.get('source_modules_sha256') == modules, 'Processing/fusion source hash mapping differs')
    checks(processing.get('script_sha256') == modules.get('carla/fuse_production_sensors.py'),
           'Processing entry-point SHA differs')
    source_files = {}
    for name, digest in modules.items():
        path = (root / name).resolve()
        if checks(path.is_relative_to(root) and path.is_file(), 'Invalid bound source module: ' + name):
            checks(sha(path) == digest, 'Source module changed: ' + name)
            source_files[path] = digest
    checks(bool(records) and len(records) == summary.get('frames') == processing.get('frames'), 'Derived frame count differs')
    frame_list = [record['frame'] for record in records]
    checks(frame_list == summary.get('native_frame_ids') == processing.get('native_frame_ids') and
           len(set(frame_list)) == len(frame_list), 'Derived native frame sequence differs')
    row_lookup = {row['frame']: row for row in rows}
    selected = set(range(0, len(records), stride))
    if records:
        selected.add(len(records) - 1)
    counters = {camera: collections.Counter() for camera in CAMERAS}
    total = collections.Counter()
    previous, observed_ids, azimuth_min, azimuth_max, elevation_min, elevation_max, min_x = None, {}, [], [], [], [], []
    fov_deviations = collections.Counter()
    for index, record in enumerate(records):
        frame = record['frame']
        if not checks(frame in row_lookup, f'Derived frame absent from native capture {frame}'):
            continue
        row, sensors = row_lookup[frame], row_lookup[frame]['sensors']
        checks(record['sample'] == row['sample'] and record['timestamp'] == row['timestamp'], f'Native record identity differs {frame}')
        if previous:
            checks(frame == previous['frame'] + 1 and record['sample'] == previous['sample'] + 1 and
                   abs(record['timestamp'] - previous['timestamp'] - .05) < 2e-6, f'Derived cadence differs {frame}')
        previous = record
        names = list(CAMERAS.values()) + ['lidar'] + (['chase'] if 'chase' in sensors else [])
        for name in names:
            sensor = sensors[name]
            checks(sensor['frame'] == frame and abs(sensor['timestamp'] - row['timestamp']) <= 1e-6,
                   f'Native sensor synchronization differs {name} {frame}')
            if name != 'chase':
                checks(record['native_sensor_bindings'].get(name) == {key: sensor[key] for key in ('frame', 'timestamp', 'raw_sha256')},
                       f'Derived native sensor binding differs {name} {frame}')
        for camera in CAMERAS:
            items = [d for d in record['detections'] if d.get('camera') == camera]
            counters[camera]['frames_with_detection'] += bool(items)
            counters[camera]['frames_with_association'] += any(d['fusion_status'] == 'associated' for d in items)
            counters[camera]['detected_boxes_all_records'] += len(items)
            counters[camera]['associated_boxes_all_records'] += sum(d['fusion_status'] == 'associated' for d in items)
        visible = set()
        for item in record['detections']:
            identifier, camera = item['display_track_id'], item['camera']
            checks(identifier not in visible, f'Repeated visible ID {frame}')
            visible.add(identifier)
            last = observed_ids.get(identifier)
            if last:
                checks(last['camera'] == camera and last['class_id'] == item['class_id'] and
                       record['timestamp'] - last['timestamp'] <= .500001,
                       f'ID changed camera/class or reappeared after expiry {identifier} {frame}')
            observed_ids[identifier] = {'camera': camera, 'class_id': item['class_id'], 'timestamp': record['timestamp']}
        if index not in selected:
            continue
        total['geometry_audited_frames'] += 1
        try:
            ego = next(vehicle for vehicle in row['vehicles'] if vehicle.get('ego') is True)
            ego_world = check_transform(ego['transform']['world_from_local'], checks, f'ego {frame}')
            lidar_world = check_transform(sensors['lidar']['transform']['world_from_local'], checks, f'lidar {frame}')
            lidar_pose = sensors['lidar']['transform']
            checks(close(lidar_world, matrix_from_pose(lidar_pose['location_m'], lidar_pose['rotation_deg']), atol=2e-5),
                   f'Native LiDAR pose/matrix differ {frame}')
            attached = np.linalg.solve(ego_world, lidar_world)
            checks(close(attached[:3, :3], relative['lidar'][:3, :3], atol=2e-5) and
                   close(attached[:3, 3], relative['lidar'][:3, 3], atol=.002, rtol=0),
                   f'Per-frame native LiDAR mounting differs {frame}')
            front_world = front_k = None
            for camera, name in CAMERAS.items():
                file_sha, k, world = check_sensor_image(run, name, row, manifest, relative, ego_world, checks)
                key = 'rgb_file_sha256' if camera == 'front' else camera + '_rgb_file_sha256'
                checks(file_sha == record[key], f'Native source image file SHA differs {camera} {frame}')
                checks(close(record['camera_intrinsics'].get(camera), k, atol=1e-5), f'Derived camera K differs {camera} {frame}')
                total['audited_native_camera_pngs'] += 1
                if camera == 'front':
                    front_world, front_k = world, k
            if 'chase' in sensors:
                chase_relative = manifest['sensors']['chase']['relative_transform']['world_from_local']
                check_sensor_image(run, 'chase', row, manifest, {'chase': np.asarray(chase_relative)}, ego_world, checks)
                checks(manifest['sensors']['chase'].get('role') == 'filming_only', 'Chase role differs')
                total['audited_native_filming_pngs'] += 1
            raw = np.load(run / 'lidar' / f'{frame:08d}.npy', allow_pickle=False)
            checks(hashlib.sha256(raw.astype('<f4', copy=False).tobytes()).hexdigest() ==
                   record['native_lidar_raw_sha256'] == sensors['lidar']['raw_sha256'], f'Native LiDAR byte SHA differs {frame}')
            data_path = fusion / 'data' / f'{frame:08d}.npz'
            checks(sha(data_path) == record['geometric_points_npz_sha256'], f'Derived NPZ SHA differs {frame}')
            with np.load(data_path, allow_pickle=False) as archive:
                data = {name: archive[name].copy() for name in archive.files}
            xyz, ground, indices, metrics = check_points(raw, row, manifest, data, lidar_attributes, checks)
            for name, value in metrics.items():
                checks(close(record['geometry_statistics'].get(name), value), f'Derived geometry metric differs {name} {frame}')
                if name.startswith('strict_') or name == 'engineering_tolerance_exceeded_points':
                    fov_deviations[name] += value
                elif name.startswith('max_'):
                    fov_deviations[name] = max(fov_deviations[name], value)
            azimuth_min.append(metrics['native_azimuth_min_deg'])
            azimuth_max.append(metrics['native_azimuth_max_deg'])
            elevation_min.append(metrics['native_elevation_min_deg'])
            elevation_max.append(metrics['native_elevation_max_deg'])
            min_x.append(metrics['native_min_x_m'])
            total['audited_native_raw_points'] += len(raw)
            projection = independent_projection(xyz, lidar_world, front_world, front_k, 1280, 720)
            projected, uv, depth = projection
            checks(np.array_equal(np.flatnonzero(projected), data['projection_sample_indices']), f'Independent front projection index differs {frame}')
            checks(close(uv[projected], data['projection_uv']) and close(depth[projected], data['projection_depth_m']),
                   f'Independent front projected pixels/depth differ {frame}')
            measured = check_detections(record, summary, xyz, ground, indices, projection, lidar_world, checks)
            for camera, values in measured.items():
                counters[camera].update({'audited_' + key: int(value) for key, value in values.items()})
            for view in ('camera', 'rear', 'side_left', 'side_right', 'lidar'):
                path = fusion / 'frames' / f'{frame:08d}-{view}.png'
                checks(sha(path) == record[view + '_sha256'], f'Derived PNG SHA differs {view} {frame}')
                with Image.open(path) as image:
                    checks(image.mode == 'RGB' and image.size == ((640, 1080) if view == 'lidar' else (1280, 720)),
                           f'Derived PNG type/dimensions differ {view} {frame}')
                    image.load()
                total['audited_derived_pngs'] += 1
        except (KeyError, TypeError, ValueError, IndexError, OSError, StopIteration, np.linalg.LinAlgError) as error:
            checks(False, f'Frame {frame} audit exception {type(error).__name__}: {error}')
    for camera, values in counters.items():
        reported = summary['statistics'][camera]
        for key, calculated in [('detected_boxes', values['detected_boxes_all_records']),
                                ('associated_boxes', values['associated_boxes_all_records']),
                                ('frames_with_detection', values['frames_with_detection']),
                                ('frames_with_association', values['frames_with_association'])]:
            checks(reported.get(key) == calculated, f'Summary count differs {camera} {key}')
    if stride == 1 and azimuth_min:
        for key, value in [('native_azimuth_min_deg', min(azimuth_min)),
                            ('native_azimuth_max_deg', max(azimuth_max)),
                            ('native_elevation_min_deg', min(elevation_min)),
                            ('native_elevation_max_deg', max(elevation_max)), ('native_min_x_m', min(min_x)),
                            *fov_deviations.items()]:
            checks(close(processing.get(key), value), 'Global raw LiDAR metric differs: ' + key)
    for key, path in paths.items():
        checks(sha(path) == hashes[key], 'Bound input changed during audit: ' + key)
    for path, digest in source_files.items():
        checks(sha(path) == digest, 'Bound source module changed during audit: ' + path.name)
    checks(sha(__file__) == auditor_sha, 'Auditor source changed during audit')
    total['frames'] = len(records)
    total['distinct_display_ids'] = len(observed_ids)
    return {'status': 'failed' if checks.counts['failed_assertions'] else 'provenance_and_geometry_passed_visual_review_required',
            'auditor_sha256': auditor_sha, 'input_sha256': hashes, 'source_modules_sha256': modules,
            'assertions': checks.counts, 'failures': checks.failures,
            'failures_truncated': checks.counts['failed_assertions'] > len(checks.failures),
            'stride': stride, 'all_output_frames_geometry_audited': total['geometry_audited_frames'] == len(records),
            'all_record_identity_and_timing_checked': True, 'accuracy_claimed': False,
            'other_vehicle_ground_truth_read': False, 'cross_camera_reidentification_claimed': False,
            'stats': {key: int(value) for key, value in total.items()},
            'by_camera': {camera: {key: int(value) for key, value in values.items()} for camera, values in counters.items()},
            'audited_raw_azimuth_deg': [min(azimuth_min), max(azimuth_max)] if azimuth_min else None,
            'native_fov_development_policy': {'nominal_horizontal_fov_deg': 120.,
                'boundary_budget_m': FOV_BOUNDARY_BUDGET_M, 'angle_budget_deg': FOV_ANGLE_BUDGET_DEG,
                'both_limits_required': True, 'negative_x_allowed': False,
                'basis': 'Fixed engineering budget; angle cap is one tenth of nominal 0.2-degree ray spacing.',
                'proven_theoretical_float_error_bound': False, 'points_cropped_or_corrected': False},
            'native_fov_strict_deviations': dict(fov_deviations),
            'limits': ['Byte provenance and geometry consistency do not establish detector/fusion accuracy.',
                       'Rendered PNG hashes bind image content but require separate visual inspection.',
                       'A camera-only result has no measured distance; no cross-camera identity is inferred.',
                       'The native ray-cast profile is a generic sensor approximation, not hardware validation.',
                       'A sampled stride greater than one does not validate geometry of skipped frames.',
                       'Ground checks validate stored-plane residuals, not an independent road-label benchmark.',
                       'Native capture trust relies on recorded callbacks and capture execution provenance.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('run', 'fusion', 'out'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--stride', type=int, default=1)
    args = parser.parse_args()
    report = audit(args.run, args.fusion, args.stride)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({key: report[key] for key in ('status', 'assertions', 'failures', 'stats', 'by_camera')}, indent=2))
    raise SystemExit(bool(report['assertions']['failed_assertions']))


if __name__ == '__main__':
    main()
