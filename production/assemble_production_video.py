"""Compose four native camera detections and forward LiDAR without image distortion."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess

import imageio_ffmpeg
import numpy as np
from PIL import Image


CAMERAS = {'camera': 'rgb', 'rear': 'rear', 'side_left': 'side_left', 'side_right': 'side_right'}
NATIVE_HASH_KEYS = {'camera': 'rgb_file_sha256', 'rear': 'rear_rgb_file_sha256',
                    'side_left': 'side_left_rgb_file_sha256', 'side_right': 'side_right_rgb_file_sha256'}
LAYOUT = {'camera': [0, 0, 960, 540], 'rear': [0, 540, 960, 540],
          'lidar': [960, 0, 640, 1080], 'side_left': [1600, 90, 320, 180],
          'side_right': [1600, 450, 320, 180], 'chase': [1600, 810, 320, 180]}
PANELS = dict(LAYOUT, side_left=[1600, 0, 320, 360], side_right=[1600, 360, 320, 360],
              chase=[1600, 720, 320, 360])


def compose_views(pictures):
    """Paste full native images with uniform scaling and centered letterboxing.

    LAYOUT specifies actual image content, PANELS includes black letterbox areas.
    No input image is cropped, stretched nonuniformly, or resynthesized.
    """
    canvas = Image.new('RGB', (1920, 1080), (0, 0, 0))
    for name, (x, y, width, height) in LAYOUT.items():
        picture = pictures[name]
        expected = (640, 1080) if name == 'lidar' else (1280, 720)
        require(picture.size == expected, f'Unexpected native render size for {name}')
        require(abs(width/picture.width-height/picture.height) < 1e-9,
                f'Nonuniform scale is forbidden: {name}')
        canvas.paste(picture.resize((width, height), Image.Resampling.LANCZOS), (x, y))
    return canvas


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def read_rows(path):
    with Path(path).open(encoding='utf-8-sig') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def inside(root, value):
    path = (root / value).resolve()
    require(path.is_relative_to(root), f'Input escapes project root: {value}')
    return path


def png(path, expected_size, expected_sha=None, native_raw_sha=None):
    raw = path.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    require(expected_sha is None or digest == expected_sha, f'PNG SHA mismatch: {path}')
    # Decode the exact bytes checked above, so a concurrent rewrite cannot change the image.
    from io import BytesIO
    with Image.open(BytesIO(raw)) as source:
        require(source.format == 'PNG' and source.size == expected_size,
                f'PNG format or dimensions mismatch: {path}')
        picture = source.convert('RGB')
    if native_raw_sha:
        pixels = np.asarray(picture.convert('RGBA'))[:, :, [2, 1, 0, 3]]
        require(hashlib.sha256(pixels.tobytes()).hexdigest() == native_raw_sha,
                f'Native BGRA reconstruction mismatch: {path}')
    return picture, digest


def prepare_scene(root, scene):
    run, fusion = [inside(root, scene[key]) for key in ('run', 'fusion_out')]
    # fuse_production_sensors writes its geometry and processing manifest in its
    # own output. An explicit identical processed path remains supported.
    processed = inside(root, scene.get('processed', scene['fusion_out']))
    require(processed == fusion, 'Production geometry and rendered output must share the processor output')
    bound = {path: sha(path) for path in (run/'manifest.json', run/'frames.jsonl',
        run/'sensor-profile.json', processed/'processing-manifest.json',
        fusion/'fusion-manifest.json', fusion/'associations.jsonl')}
    native, rendered = read_json(run/'manifest.json'), read_json(fusion/'fusion-manifest.json')
    geometry = read_json(processed/'processing-manifest.json')
    require(native['status'] == 'complete' and native['fps'] == 20, 'Source must be complete native 20 fps')
    require(native.get('synthetic_sensor_fallback') is False, 'Native sensor input is required')
    require(native['sensor_profile']['sha256'] == bound[run/'sensor-profile.json'] and
            native['sensor_profile']['native_mounts_and_blueprint_fov'] is True and
            native['sensor_profile']['post_capture_fov_crop'] is False,
            'Capture must bind the native mounted-sensor profile without a FOV crop')
    require(read_json(run/'sensor-profile.json') == native['sensor_profile']['definition'],
            'Native mounted-sensor profile definition differs')
    require(Path(rendered['source_run']).resolve() == run, 'Fusion source run mismatch')
    require(Path(rendered['source_processed']).resolve() == processed, 'Fusion processed source mismatch')
    for key, path in [('source_manifest_sha256', run/'manifest.json'),
                      ('source_frames_sha256', run/'frames.jsonl'),
                      ('source_sensor_profile_sha256', run/'sensor-profile.json'),
                      ('source_processing_manifest_sha256', processed/'processing-manifest.json'),
                      ('associations_sha256', fusion/'associations.jsonl')]:
        require(rendered[key] == bound[path], f'Fusion binding mismatch: {key}')
    for key, path in [('source_manifest_sha256', run/'manifest.json'),
                      ('source_frames_sha256', run/'frames.jsonl'),
                      ('source_frames_jsonl_sha256', run/'frames.jsonl'),
                      ('source_sensor_profile_sha256', run/'sensor-profile.json')]:
        require(geometry[key] == bound[path], f'Geometry binding mismatch: {key}')
    require(geometry['generated_points'] is False and geometry['post_capture_fov_crop'] is False and
            geometry['native_lidar_horizontal_fov_deg'] == 120., 'Native forward 120-degree LiDAR geometry required')
    require(rendered['cross_camera_reidentification'] is False and rendered['trained_fusion_model'] is False,
            'Current production baseline uses per-camera observation IDs and geometric front fusion')
    rows, associations = read_rows(run/'frames.jsonl'), read_rows(fusion/'associations.jsonl')
    require(len(rows) == native['frames_captured'], 'Native capture count mismatch')
    require(len(associations) == rendered['frames'], 'Fusion association count mismatch')
    require(geometry['frames'] == rendered['frames'] and geometry['native_frame_ids'] == rendered['native_frame_ids'],
            'Geometry and fusion frame selections differ')
    require([row['frame'] for row in associations] == rendered['native_frame_ids'], 'Fusion frame IDs mismatch')
    by_sample = {row['sample']: row for row in associations}
    require(len(by_sample) == len(associations), 'Duplicate fusion samples')
    start, end = scene['start_sample'], scene['end_sample_exclusive']
    require(isinstance(start, int) and isinstance(end, int) and 0 <= start < end <= len(rows),
            'Invalid source selection')
    selected = []
    for sample in range(start, end):
        row = rows[sample]
        require(row['sample'] == sample and sample in by_sample, 'Missing or inconsistent sample')
        association = by_sample[sample]
        require(association['frame'] == row['frame'] and
                abs(association['timestamp'] - row['timestamp']) < 1e-6, 'Association time mismatch')
        for detection in association['detections']:
            require(detection['camera'] in ('front', 'rear', 'side_left', 'side_right'),
                    'Unexpected camera in a four-camera fusion result')
            if detection['camera'] != 'front':
                require(detection['fusion_status'] == 'camera_only',
                        'The forward-only baseline cannot claim rear/side LiDAR fusion')
            if detection['fusion_status'] != 'associated':
                require(not detection['sampled_point_indices'] and detection.get('range_m') is None,
                        'Camera-only observations cannot contain a LiDAR range or target points')
        for name in ('rgb', 'lidar', 'chase', 'rear', 'side_left', 'side_right'):
            sensor = row['sensors'][name]
            require(sensor['frame'] == row['frame'] and abs(sensor['timestamp']-row['timestamp']) < 1e-6,
                    f'Native synchronization mismatch: {name}')
            if name != 'chase':
                binding = association['native_sensor_bindings'][name]
                require(binding['frame'] == sensor['frame'] and
                        abs(binding['timestamp']-sensor['timestamp']) < 1e-6 and
                        binding['raw_sha256'] == sensor['raw_sha256'],
                        f'Fusion/native sensor binding differs: {name}')
        if selected:
            previous = selected[-1][0]
            require(row['frame'] == previous['frame'] + 1 and
                    abs(row['timestamp']-previous['timestamp']-.05) < 2e-6, 'Skipped native frame')
        token = f"{row['frame']:08d}"
        paths = {'camera': fusion/'frames'/f'{token}-camera.png',
                 'lidar': fusion/'frames'/f'{token}-lidar.png',
                 'chase': run/'chase'/f'{token}.png',
                 'rear': fusion/'frames'/f'{token}-rear.png',
                 'side_left': fusion/'frames'/f'{token}-side_left.png',
                 'side_right': fusion/'frames'/f'{token}-side_right.png',
                 'native_rear': run/'rear'/f'{token}.png',
                 'native_side_left': run/'side_left'/f'{token}.png',
                 'native_side_right': run/'side_right'/f'{token}.png',
                 'rgb': run/'rgb'/f'{token}.png', 'raw_lidar': run/'lidar'/f'{token}.npy',
                 'geometry': processed/'data'/f'{token}.npz'}
        require(all(path.is_file() for path in paths.values()), f'Missing selected source frame: {token}')
        selected.append((row, association, paths))
    record = dict(scene, processed=processed.relative_to(root).as_posix(),
        source_manifest_sha256=bound[run/'manifest.json'],
        source_frames_sha256=bound[run/'frames.jsonl'],
        source_sensor_profile_sha256=bound[run/'sensor-profile.json'],
        source_processing_manifest_sha256=bound[processed/'processing-manifest.json'],
        fusion_manifest_sha256=bound[fusion/'fusion-manifest.json'],
        associations_sha256=bound[fusion/'associations.jsonl'],
        detector_model_sha256=rendered['model_sha256'], limitations=rendered['limits'],
        tracking_scope=rendered['tracking_scope'], cross_camera_reidentification=False,
        native_lidar_horizontal_fov_deg=120., derived_png_compression=rendered['derived_png_compression'])
    return selected, record, bound


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    root, out = args.root.resolve(), args.out.resolve()
    spec_sha = sha(args.spec)
    specification = read_json(args.spec)
    require(bool(specification['scenes']), 'No scenes supplied')
    scenes = [prepare_scene(root, scene) for scene in specification['scenes']]
    out.mkdir(parents=True, exist_ok=False)
    output, mapping = out/'DriveWeave-four-camera-forward-lidar.mp4', out/'frame-map.jsonl'
    assembler_sha = sha(__file__)
    writer = imageio_ffmpeg.write_frames(str(output), (1920, 1080), fps=20,
        codec='libx264', pix_fmt_in='rgb24', pix_fmt_out='yuv420p', macro_block_size=1,
        ffmpeg_log_level='error', output_params=['-preset', 'fast', '-crf', '18',
            '-threads', '4', '-g', '40', '-movflags', '+faststart'])
    writer.send(None)
    count, records = 0, []
    try:
        with mapping.open('w', encoding='utf-8') as stream:
            for selected, record, bound in scenes:
                record['output_start_sample'] = count
                for row, association, paths in selected:
                    native_hashes, rendered_hashes, pictures = {}, {}, {}
                    for output_name, sensor_name in CAMERAS.items():
                        native_path = paths['rgb' if output_name == 'camera' else 'native_'+output_name]
                        _, digest = png(native_path, (1280, 720), association[NATIVE_HASH_KEYS[output_name]],
                            native_raw_sha=row['sensors'][sensor_name]['raw_sha256'])
                        native_hashes['front' if output_name == 'camera' else output_name] = digest
                        pictures[output_name], rendered_hashes[output_name] = png(paths[output_name],
                            (1280, 720), association[output_name+'_sha256'])
                    require(sha(paths['geometry']) == association['geometric_points_npz_sha256'], 'Geometry file changed')
                    raw = np.load(paths['raw_lidar'], allow_pickle=False)
                    raw_sha = hashlib.sha256(raw.astype('<f4', copy=False).tobytes()).hexdigest()
                    require(raw_sha == association['native_lidar_raw_sha256'] == row['sensors']['lidar']['raw_sha256'],
                            'Native LiDAR changed')
                    pictures['lidar'], rendered_hashes['lidar'] = png(paths['lidar'], (640, 1080), association['lidar_sha256'])
                    pictures['chase'], rendered_hashes['chase'] = png(paths['chase'], (1280, 720),
                        native_raw_sha=row['sensors']['chase']['raw_sha256'])
                    canvas = compose_views(pictures)
                    writer.send(canvas.tobytes())
                    entry = {'output_sample': count, 'scene': record['id'], 'source_sample': row['sample'],
                        'native_frame': row['frame'], 'native_timestamp': row['timestamp'],
                        **{name+'_png_sha256': digest for name, digest in rendered_hashes.items()},
                        **{'native_'+name+'_png_sha256': digest for name, digest in native_hashes.items()},
                        'composite_rgb_sha256': hashlib.sha256(canvas.tobytes()).hexdigest()}
                    stream.write(json.dumps(entry, separators=(',', ':')) + '\n')
                    count += 1
                    if count % 100 == 0:
                        print(json.dumps({'encoded_frames': count, 'scene': record['id']}), flush=True)
                record['output_end_sample_exclusive'] = count
                records.append(record)
                require(all(sha(path) == digest for path, digest in bound.items()), 'Source metadata changed during encoding')
    finally:
        writer.close()
    require(sha(args.spec) == spec_sha, 'Specification changed during encoding')
    require(sha(__file__) == assembler_sha, 'Assembler changed during encoding')
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    result = subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-nostdin',
        '-i', str(output), '-map', '0:v:0', '-progress', 'pipe:1', '-fps_mode', 'passthrough',
        '-f', 'null', '-'], capture_output=True, text=True)
    require(result.returncode == 0 and not result.stderr.strip(), f'Full decode failed: {result.stderr}')
    decoded = int([line.split('=', 1)[1] for line in result.stdout.splitlines() if line.startswith('frame=')][-1])
    reader = imageio_ffmpeg.read_frames(str(output))
    metadata = next(reader)
    reader.close()
    require(decoded == count and metadata['size'] == (1920, 1080) and abs(metadata['fps']-20) < .001
            and abs(metadata['duration']-count/20) < .011, 'Output metadata differs from source mapping')
    report = {'status': 'encoded_and_full_decode_passed_visual_review_pending',
        'file': output.name, 'sha256': sha(output), 'bytes': output.stat().st_size,
        'frames': count, 'fps': 20, 'duration_s': count/20, 'size': [1920, 1080],
        'camera_and_lidar_visible_every_frame': True, 'playback_speed': 1,
        'front_and_rear_camera_detection_views_visible_every_frame': True,
        'four_camera_detection_views_visible_every_frame': True,
        'all_camera_views_are_rendered_detection_outputs': True,
        'all_four_native_camera_bgra_verified_every_frame': True,
        'camera_only_observations_have_no_invented_lidar_returns': True,
        'lidar_association_cameras': ['front'],
        'camera_only_detection_cameras': ['rear', 'side_left', 'side_right'],
        'cross_camera_reidentification': False,
        'interpolated_frames': False, 'synthetic_filler_frames': False, 'audio_added': False,
        'added_editorial_text': False, 'sensor_object_annotations_preserved': True,
        'offline_processing': True, 'trained_fusion_model': False,
        'full_decoded_frames': decoded, 'specification_sha256': spec_sha,
        'assembler_sha256': assembler_sha, 'frame_mapping_file': mapping.name,
        'frame_mapping_sha256': sha(mapping), 'scenes': records,
        'layout': LAYOUT, 'layout_panels': PANELS,
        'source_crops': {name: [0, 0, 640, 1080] if name == 'lidar' else [0, 0, 1280, 720]
                         for name in LAYOUT},
        'resize_method': 'uniform Lanczos scaling; full native images; side/chase centered black letterbox',
        'nonuniform_image_scaling': False,
        'published_to_bilibili': False}
    (out/'video-manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: value for key, value in report.items() if key not in ('scenes', 'layout')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
