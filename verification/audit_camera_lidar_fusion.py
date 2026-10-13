"""Independent provenance and geometry audit of offline camera/LiDAR fusion.

Inference is never imported or rerun. Optional instance masks and actor boxes
are opened only after output hashes have been frozen, for held-out diagnostics.
Those labels cannot change detections, associations, thresholds, or media here.
"""
from pathlib import Path
import argparse
import collections
import hashlib
import json
import math

import numpy as np
from PIL import Image


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4*1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines() if line.strip()]


def independent_projection(xyz, lidar, camera, intrinsic, width, height):
    """Solve the native rigid transform directly; do not import renderer math."""
    homogeneous = np.concatenate((np.asarray(xyz, dtype=float), np.ones((len(xyz), 1))), axis=1)
    camera_xyz = np.linalg.solve(np.asarray(camera, float), np.asarray(lidar, float) @ homogeneous.T).T[:, :3]
    optical = camera_xyz[:, [1, 2, 0]].copy()
    optical[:, 1] *= -1
    camera_depth = optical[:, 2]
    perspective = optical @ np.asarray(intrinsic, float).T
    pixels = np.full((len(xyz), 2), np.nan)
    positive = camera_depth > .1
    pixels[positive] = perspective[positive, :2] / camera_depth[positive, None]
    keep = positive & np.isfinite(pixels).all(axis=1)
    keep &= (pixels[:, 0] >= 0) & (pixels[:, 0] < width)
    keep &= (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
    return keep, pixels, camera_depth


def overlap_iou(a, b):
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0., x1-x0)*max(0., y1-y0)
    area_a, area_b = max(0., a[2]-a[0])*max(0., a[3]-a[1]), max(0., b[2]-b[0])*max(0., b[3]-b[1])
    return intersection / max(area_a+area_b-intersection, 1e-12)


def gt_instance_boxes(instance):
    classes = instance[:, :, 0]
    identifiers = instance[:, :, 1].astype(np.uint16)*256+instance[:, :, 2]
    mapping = {12: 0, 19: 1, 14: 2, 18: 3, 16: 5, 15: 7}
    result = []
    for native, coco in mapping.items():
        for identifier in np.unique(identifiers[classes == native]):
            y, x = np.where((classes == native) & (identifiers == identifier))
            if len(x) >= 20:
                result.append({'class_id': coco, 'native_class_id': native,
                               'instance_code': int(identifier), 'pixels': len(x),
                               'bbox_xyxy': [int(x.min()), int(y.min()), int(x.max()), int(y.max())]})
    return result


def box_rotation(rpy):
    # CARLA local x forward/y right/z up convention, matching recorded transforms.
    roll, pitch, yaw = map(math.radians, rpy)
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cp*cy, cy*sp*sr-sy*cr, -cy*sp*cr-sy*sr],
                     [cp*sy, sy*sp*sr+cy*cr, -sy*sp*cr+cy*sr],
                     [sp, -cp*sr, cp*cr]])


def independent_gt_diagnostics(run, rows_by_frame, records, samples):
    """Read labels after predictions are frozen; descriptive sample, not a benchmark."""
    report = {'evaluation_only': True, 'inference_feedback': False,
              'selection': 'evenly spaced output records, including endpoints',
              'instance_minimum_visible_pixels': 20, 'detector_iou_threshold': .5,
              'point_box_tolerance_m': .15,
              'limits': ['Single prerecorded set with domain shift; no accuracy or robustness claim.',
                         'Native instance masks have independent rasterization and exclude heavy occlusion.',
                         'Vehicle box containment is a coarse diagnostic, not semantic LiDAR ground truth.',
                         'A display ID is not a native actor or benchmark track ID.'],
              'frames': [], 'counts': {}}
    totals = collections.Counter()
    if not samples or not records:
        report['status'] = 'not_requested'
        return report
    for index in np.unique(np.linspace(0, len(records)-1, min(samples, len(records)), dtype=int)):
        record = records[int(index)]
        row = rows_by_frame[record['frame']]
        frame = row['frame']
        path = run/'instance'/f'{frame:08d}.png'
        instance = np.asarray(Image.open(path).convert('RGB'))
        boxes = gt_instance_boxes(instance)
        detections = record['detections']
        matched_gt = set()
        matches = []
        for detection in sorted(detections, key=lambda item: -item['score']):
            candidates = [(overlap_iou(detection['bbox_xyxy'], box['bbox_xyxy']), i)
                          for i, box in enumerate(boxes)
                          if i not in matched_gt and box['class_id'] == detection['class_id']]
            best = max(candidates, default=(0., None))
            is_match = best[0] >= .5
            if is_match:
                matched_gt.add(best[1])
            matches.append({'display_id': detection['display_track_id'], 'class_id': detection['class_id'],
                            'score': detection['score'], 'best_same_class_iou': best[0],
                            'matched_visible_instance': bool(is_match)})
        raw = np.load(run/'lidar'/f'{frame:08d}.npy', allow_pickle=False)
        lidar_world = np.asarray(row['sensors']['lidar']['transform']['world_from_local'], float)
        vehicle_boxes = []
        for vehicle in row['vehicles']:
            if vehicle.get('ego'):
                continue
            shape = vehicle['bounding_box']
            world_from_box = np.asarray(vehicle['transform']['world_from_local'], float).copy()
            local_box = np.eye(4)
            local_box[:3, :3] = box_rotation(shape.get('rotation_rpy_deg', [0, 0, 0]))
            local_box[:3, 3] = shape['center_local_m']
            vehicle_boxes.append((vehicle['id'], np.linalg.inv(world_from_box @ local_box),
                                  np.asarray(shape['extent_m'])+.15))
        supports = []
        for detection in detections:
            if detection['fusion_status'] != 'associated':
                continue
            points = raw[detection['raw_point_indices'], :3]
            world = lidar_world @ np.column_stack((points, np.ones(len(points)))).T
            union = np.zeros(len(points), bool)
            actor_counts = []
            for actor_id, box_from_world, extent in vehicle_boxes:
                contained = (abs((box_from_world @ world)[:3].T) <= extent).all(axis=1)
                union |= contained
                if contained.any():
                    actor_counts.append((int(contained.sum()), actor_id))
            fraction = float(union.mean())
            totals['associated_points'] += len(points)
            totals['associated_points_inside_any_native_vehicle_box'] += int(union.sum())
            totals['associated_boxes'] += 1
            totals['associated_boxes_below_half_native_vehicle_box_support'] += fraction < .5
            supports.append({'display_id': detection['display_track_id'], 'points': len(points),
                             'inside_any_native_vehicle_box_fraction': fraction,
                             'dominant_native_actor': max(actor_counts, default=(0, None))[1]})
        totals['sampled_frames'] += 1
        totals['camera_detections'] += len(detections)
        totals['visible_native_instances'] += len(boxes)
        totals['matched_detections_iou_0_5'] += len(matched_gt)
        report['frames'].append({'frame': frame, 'sample': row['sample'],
                                 'instance_png_sha256': sha(path), 'gt_visible_boxes': boxes,
                                 'camera_matches': matches, 'point_support': supports})
    report['counts'] = dict(totals)
    report['status'] = 'diagnostic_review_required' if totals['associated_boxes_below_half_native_vehicle_box_support'] else 'diagnostics_completed'
    return report


def audit(run, processed, fusion, model=None, gt_samples=15):
    run, processed, fusion = map(lambda path: Path(path).resolve(), (run, processed, fusion))
    manifest = read_json(run/'manifest.json')
    processing = read_json(processed/'processing-manifest.json')
    summary = read_json(fusion/'fusion-manifest.json')
    rows = read_rows(run/'frames.jsonl')
    rows_by_frame = {row['frame']: row for row in rows}
    records = read_rows(fusion/'associations.jsonl')
    source_hashes = {'capture_manifest': sha(run/'manifest.json'), 'capture_frames': sha(run/'frames.jsonl'),
                     'processing_manifest': sha(processed/'processing-manifest.json'),
                     'fusion_manifest': sha(fusion/'fusion-manifest.json'), 'associations': sha(fusion/'associations.jsonl')}
    failures, counts = [], collections.Counter()
    def require(condition, message):
        counts['assertions'] += 1
        if not condition:
            failures.append(message)
        else:
            counts['passed_assertions'] += 1
    require(manifest.get('status') == 'complete' and manifest.get('synthetic_sensor_fallback') is False, 'Not a complete native capture')
    require(processing['source_frames_jsonl_sha256'] == source_hashes['capture_frames'], 'Processing capture frame hash differs')
    require(processing['source_manifest_sha256'] == source_hashes['capture_manifest'], 'Processing capture manifest hash differs')
    for key, actual in [('source_manifest_sha256', source_hashes['capture_manifest']),
                        ('source_frames_sha256', source_hashes['capture_frames']),
                        ('source_processing_manifest_sha256', source_hashes['processing_manifest']),
                        ('associations_sha256', source_hashes['associations'])]:
        require(summary.get(key) == actual, key+' differs')
    require(summary.get('uses_simulator_labels_for_detection_or_association') is False, 'No-label inference declaration missing')
    require(summary.get('trained_image_detector_inference') is True and summary.get('trained_fusion_model') is False, 'Model scope declaration differs')
    require(summary.get('offline_processing') is True and summary.get('processing_controls_vehicle') is False, 'Offline processing scope differs')
    provenance = summary.get('detector_provenance', {})
    require(provenance.get('matches_known_official_model_sha256') is True and
            provenance.get('model_sha256') == summary.get('model_sha256') ==
            'c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063',
            'Official detector model provenance differs')
    require(provenance.get('ground_truth_used_for_inference') is False and
            provenance.get('lidar_used_for_detection') is False,
            'RGB-only detector scope differs')
    root = Path(__file__).resolve().parents[1]
    for name, key in [('fuse_camera_lidar.py', 'script_sha256'), ('detect_rgb_onnx.py', 'detector_script_sha256')]:
        path = root/'carla'/name
        require(path.is_file() and sha(path) == summary.get(key), key+' differs from current source')
    if model:
        require(sha(model) == summary.get('model_sha256'), 'Detector model SHA differs')
    require(len(records) == summary.get('frames') and bool(records), 'Fusion frame count differs or is zero')
    require([r['frame'] for r in records] == summary.get('native_frame_ids'), 'Fusion frame list differs')
    require(len({r['frame'] for r in records}) == len(records), 'Repeated output frame')
    previous = None
    stats = collections.Counter()
    per_frame = []
    for record in records:
        frame = record['frame']
        row = rows_by_frame[frame]
        sensors = row['sensors']
        require(record['sample'] == row['sample'] and record['timestamp'] == row['timestamp'], f'Native record identity differs {frame}')
        if previous:
            require(frame == previous['frame']+1, f'Native frame gap {frame}')
            require(abs(record['timestamp']-previous['timestamp']-1/manifest['fps']) < 1e-5, f'Timestamp cadence differs {frame}')
        previous = record
        for name in ['rgb', 'lidar']:
            require(sensors[name]['frame'] == frame and abs(sensors[name]['timestamp']-record['timestamp']) < 1e-6, f'{name} native time differs {frame}')
        rgb_path = run/'rgb'/f'{frame:08d}.png'
        width, height = Image.open(rgb_path).size
        require(sha(rgb_path) == record['rgb_file_sha256'], f'RGB SHA differs {frame}')
        raw = np.load(run/'lidar'/f'{frame:08d}.npy', allow_pickle=False)
        native_sha = hashlib.sha256(raw.astype('<f4', copy=False).tobytes()).hexdigest()
        require(native_sha == record['native_lidar_raw_sha256'] == sensors['lidar']['raw_sha256'], f'Native LiDAR SHA differs {frame}')
        data_path = processed/'data'/f'{frame:08d}.npz'
        require(sha(data_path) == record['geometric_points_npz_sha256'], f'Geometric data SHA differs {frame}')
        with np.load(data_path, allow_pickle=False) as data:
            raw_indices = data['sampled_raw_indices'].copy()
            ground = data['ground_mask'].copy()
            plane = data['ground_plane_abcd'].copy()
            saved_indices = data['projection_sample_indices'].copy()
            saved_uv = data['projection_uv'].copy()
        require(np.issubdtype(raw_indices.dtype, np.integer) and np.all(raw_indices >= 0) and np.all(raw_indices < len(raw)), f'Native index domain differs {frame}')
        require(len(set(raw_indices.tolist())) == len(raw_indices), f'Repeated sampled native index {frame}')
        xyz = raw[raw_indices, :3]
        require(np.isfinite(xyz).all(), f'Nonfinite native point {frame}')
        require(np.array_equal(abs(xyz.astype(float) @ plane[:3]+plane[3]) < .18, ground), f'Ground mask differs from native plane residual {frame}')
        lidar_world = np.asarray(sensors['lidar']['transform']['world_from_local'], float)
        camera_world = np.asarray(sensors['rgb']['transform']['world_from_local'], float)
        visible, pixels, depths = independent_projection(xyz, lidar_world, camera_world, manifest['camera_intrinsic'], width, height)
        require(np.array_equal(np.flatnonzero(visible), saved_indices), f'Independent native visibility differs {frame}')
        require(np.allclose(pixels[saved_indices], saved_uv, atol=1e-6), f'Independent native calibration projection differs {frame}')
        used, track_ids = set(), set()
        frame_stats = collections.Counter()
        for item in record['detections']:
            frame_stats['camera_boxes'] += 1
            frame_stats[item['fusion_status']] += 1
            bbox = np.asarray(item['bbox_xyxy'], float)
            require(bbox.shape == (4,) and np.isfinite(bbox).all() and bbox[2] > bbox[0] and bbox[3] > bbox[1], f'Invalid camera bbox {frame}')
            require(0 <= bbox[0] < width and 0 < bbox[2] <= width and 0 <= bbox[1] < height and 0 < bbox[3] <= height, f'Camera bbox outside image {frame}')
            require(np.isfinite(item['score']) and summary['confidence_threshold'] <= item['score'] <= 1, f'Invalid camera confidence {frame}')
            require(item['display_track_id'] not in track_ids and item['display_track_id'] > 0, f'Repeated or invalid display ID {frame}')
            track_ids.add(item['display_track_id'])
            require(item['fusion_status'] in ['associated', 'camera_only', 'ambiguous_depth'], f'Unknown fusion status {frame}')
            ids = np.asarray(item['sampled_point_indices'], dtype=int)
            native_ids = np.asarray(item['raw_point_indices'], dtype=int)
            require(all(type(i) is int for i in item['sampled_point_indices']+item['raw_point_indices']), f'Noninteger point indices {frame}')
            if item['fusion_status'] != 'associated':
                require(not len(ids) and not len(native_ids) and item.get('range_m') is None and item.get('observed_center_lidar_m') is None and item.get('world_observed_center_m') is None, f'Unobserved box has fabricated geometry {frame}')
                continue
            require(len(ids) >= 6 and len(ids) == len(set(ids.tolist())), f'Insufficient or repeated point support {frame}')
            if not (len(ids) and np.all(ids >= 0) and np.all(ids < len(xyz))):
                require(False, f'Associated index out of bounds {frame}')
                continue
            require(np.array_equal(raw_indices[ids], native_ids), f'Associated native index mapping differs {frame}')
            require(not used.intersection(native_ids.tolist()), f'Native point belongs to multiple detections {frame}')
            used.update(native_ids.tolist())
            require(not ground[ids].any() and visible[ids].all() and (depths[ids] > .1).all(), f'Ground or invisible point associated {frame}')
            projected = pixels[ids]
            require(((projected[:, 0] >= bbox[0]) & (projected[:, 0] <= bbox[2]) & (projected[:, 1] >= bbox[1]) & (projected[:, 1] <= bbox[3])).all(), f'Associated point outside detection frustum {frame}')
            points = xyz[ids]
            require(abs(float(np.median(np.linalg.norm(points, axis=1)))-item['range_m']) < 1e-5, f'Observed median surface range differs {frame}')
            require(np.allclose(np.median(points, axis=0), item['observed_center_lidar_m'], atol=1e-6), f'Observed median point center differs {frame}')
            require(np.allclose(points.min(axis=0), item['observed_bounds_min_m'], atol=1e-6) and np.allclose(points.max(axis=0), item['observed_bounds_max_m'], atol=1e-6), f'Observed point bounds differ {frame}')
            expected_world = (lidar_world @ np.r_[np.median(points, axis=0), 1])[:3]
            require(np.allclose(expected_world, item['world_observed_center_m'], atol=1e-6), f'World observed center differs {frame}')
            require(item['association_support_points'] == len(ids), f'Point support count differs {frame}')
            stats['associated_native_points'] += len(ids)
        for view in ['camera', 'lidar']:
            path = fusion/'frames'/f'{frame:08d}-{view}.png'
            require(sha(path) == record[f'{view}_sha256'], f'{view} rendered PNG SHA differs {frame}')
            with Image.open(path) as output:
                require(output.size == ((width, height) if view == 'camera' else (640, 1080)), f'{view} output dimensions differ {frame}')
        stats.update(frame_stats)
        stats['frames_with_camera_detection'] += frame_stats['camera_boxes'] > 0
        stats['frames_with_association'] += frame_stats['associated'] > 0
        per_frame.append({'frame': frame, 'sample': row['sample'], **dict(frame_stats)})
    require(summary.get('detected_boxes') == stats['camera_boxes'], 'Summary detected count differs')
    require(summary.get('associated_boxes') == stats['associated'], 'Summary associated count differs')
    require(summary.get('frames_with_association') == stats['frames_with_association'], 'Summary association frame count differs')
    stats['frames'] = len(records)
    stats['camera_detection_frame_fraction'] = stats['frames_with_camera_detection']/max(1, len(records))
    stats['association_frame_fraction'] = stats['frames_with_association']/max(1, len(records))
    stats['association_box_fraction'] = stats['associated']/max(1, stats['camera_boxes'])
    # Only now open native labels. The already frozen predictions remain immutable.
    evaluation = independent_gt_diagnostics(run, rows_by_frame, records, gt_samples)
    require(sha(fusion/'associations.jsonl') == source_hashes['associations'], 'Predictions changed during held-out evaluation')
    require(sha(fusion/'fusion-manifest.json') == source_hashes['fusion_manifest'], 'Manifest changed during held-out evaluation')
    return {'status': 'failed' if failures else 'provenance_and_geometry_passed_visual_and_gt_review_needed',
            'inference_truth_accuracy_claimed': False, 'trained_fusion_algorithm_claimed': False,
            'all_output_frames_audited': True, 'run': str(run), 'processed': str(processed), 'fusion': str(fusion),
            'auditor_sha256': sha(__file__), 'input_sha256': source_hashes, 'assertions': dict(counts),
            'failures': failures, 'stats': dict(stats), 'per_frame': per_frame,
            'held_out_gt_diagnostics': evaluation,
            'limitations': ['Full-frame provenance/geometry checks do not establish detection or fusion accuracy.',
                            'PNG SHA checks bind rendered color outputs, but do not replace visual inspection.',
                            'Offline media playback rate is not model inference throughput.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['run', 'processed', 'fusion', 'out']:
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--gt-samples', type=int, default=15)
    args = parser.parse_args()
    report = audit(args.run, args.processed, args.fusion, args.model, args.gt_samples)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'status': report['status'], 'failures': report['failures'], 'stats': report['stats'],
                      'gt_counts': report['held_out_gt_diagnostics']['counts'], 'out': str(args.out)}, indent=2))
    raise SystemExit(bool(report['failures']))


if __name__ == '__main__':
    main()
