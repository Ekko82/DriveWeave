"""Compose synchronized native recordings with camera and LiDAR visible throughout."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess

import imageio_ffmpeg
import numpy as np
from PIL import Image


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
    run, processed, fusion = [inside(root, scene[key]) for key in ('run', 'processed', 'fusion_out')]
    bound = {path: sha(path) for path in (run/'manifest.json', run/'frames.jsonl',
        processed/'processing-manifest.json', fusion/'fusion-manifest.json', fusion/'associations.jsonl')}
    native, rendered = read_json(run/'manifest.json'), read_json(fusion/'fusion-manifest.json')
    require(native['status'] == 'complete' and native['fps'] == 20, 'Source must be complete native 20 fps')
    require(Path(rendered['source_run']).resolve() == run, 'Fusion source run mismatch')
    require(Path(rendered['source_processed']).resolve() == processed, 'Fusion processed source mismatch')
    for key, path in [('source_manifest_sha256', run/'manifest.json'),
                      ('source_frames_sha256', run/'frames.jsonl'),
                      ('source_processing_manifest_sha256', processed/'processing-manifest.json'),
                      ('associations_sha256', fusion/'associations.jsonl')]:
        require(rendered[key] == bound[path], f'Fusion binding mismatch: {key}')
    rows, associations = read_rows(run/'frames.jsonl'), read_rows(fusion/'associations.jsonl')
    require(len(associations) == rendered['frames'], 'Fusion association count mismatch')
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
        for name in ('rgb', 'lidar', 'chase', 'rear'):
            sensor = row['sensors'][name]
            require(sensor['frame'] == row['frame'] and abs(sensor['timestamp']-row['timestamp']) < 1e-6,
                    f'Native synchronization mismatch: {name}')
        if selected:
            previous = selected[-1][0]
            require(row['frame'] == previous['frame'] + 1 and
                    abs(row['timestamp']-previous['timestamp']-.05) < 2e-6, 'Skipped native frame')
        token = f"{row['frame']:08d}"
        paths = {'camera': fusion/'frames'/f'{token}-camera.png',
                 'lidar': fusion/'frames'/f'{token}-lidar.png',
                 'chase': run/'chase'/f'{token}.png', 'rear': run/'rear'/f'{token}.png',
                 'rgb': run/'rgb'/f'{token}.png', 'raw_lidar': run/'lidar'/f'{token}.npy',
                 'geometry': processed/'data'/f'{token}.npz'}
        require(all(path.is_file() for path in paths.values()), f'Missing selected source frame: {token}')
        selected.append((row, association, paths))
    record = dict(scene, source_manifest_sha256=bound[run/'manifest.json'],
        source_frames_sha256=bound[run/'frames.jsonl'],
        source_processing_manifest_sha256=bound[processed/'processing-manifest.json'],
        fusion_manifest_sha256=bound[fusion/'fusion-manifest.json'],
        associations_sha256=bound[fusion/'associations.jsonl'],
        detector_model_sha256=rendered['model_sha256'], limitations=rendered['limits'])
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
    output, mapping = out/'DriveWeave-camera-lidar-fusion.mp4', out/'frame-map.jsonl'
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
                    require(sha(paths['rgb']) == association['rgb_file_sha256'], 'Native RGB file changed')
                    require(sha(paths['geometry']) == association['geometric_points_npz_sha256'], 'Geometry file changed')
                    raw = np.load(paths['raw_lidar'], allow_pickle=False)
                    raw_sha = hashlib.sha256(raw.astype('<f4', copy=False).tobytes()).hexdigest()
                    require(raw_sha == association['native_lidar_raw_sha256'] == row['sensors']['lidar']['raw_sha256'],
                            'Native LiDAR changed')
                    camera, camera_sha = png(paths['camera'], (1280, 720), association['camera_sha256'])
                    lidar, lidar_sha = png(paths['lidar'], (640, 1080), association['lidar_sha256'])
                    chase, chase_sha = png(paths['chase'], (1280, 720), native_raw_sha=row['sensors']['chase']['raw_sha256'])
                    rear, rear_sha = png(paths['rear'], (1280, 720), native_raw_sha=row['sensors']['rear']['raw_sha256'])
                    canvas = Image.new('RGB', (1920, 1080))
                    canvas.paste(camera, (0, 0))
                    canvas.paste(chase.resize((640, 360), Image.Resampling.LANCZOS), (0, 720))
                    canvas.paste(rear.resize((640, 360), Image.Resampling.LANCZOS), (640, 720))
                    canvas.paste(lidar, (1280, 0))
                    writer.send(canvas.tobytes())
                    entry = {'output_sample': count, 'scene': record['id'], 'source_sample': row['sample'],
                        'native_frame': row['frame'], 'native_timestamp': row['timestamp'],
                        'camera_png_sha256': camera_sha, 'lidar_png_sha256': lidar_sha,
                        'chase_png_sha256': chase_sha, 'rear_png_sha256': rear_sha,
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
        'interpolated_frames': False, 'synthetic_filler_frames': False, 'audio_added': False,
        'added_editorial_text': False, 'sensor_object_annotations_preserved': True,
        'offline_processing': True, 'trained_fusion_model': False,
        'full_decoded_frames': decoded, 'specification_sha256': spec_sha,
        'assembler_sha256': sha(__file__), 'frame_mapping_file': mapping.name,
        'frame_mapping_sha256': sha(mapping), 'scenes': records,
        'layout': {'camera': [0, 0, 1280, 720], 'chase': [0, 720, 640, 360],
                   'rear': [640, 720, 640, 360], 'lidar': [1280, 0, 640, 1080]},
        'published_to_bilibili': False}
    (out/'video-manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: value for key, value in report.items() if key not in ('scenes', 'layout')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
