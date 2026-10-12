"""Offline full decode and exact frame sample QA; never modifies capture/media inputs."""
from pathlib import Path
import argparse
import concurrent.futures
import datetime
import hashlib
import json
import re
import subprocess

from PIL import Image, ImageChops
import numpy as np
import imageio_ffmpeg


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    source = json.loads(args.manifest.read_text())
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    ids = source['native_frame_ids']
    indices = sorted(set([0, round((len(ids)-1)*.445), round((len(ids)-1)*.742), len(ids)-1]))
    run, processed = Path(source['source_run']), Path(source['processed_run'])
    folders = {'chase': run/'chase', 'front': run/'rgb', 'rear': run/'rear',
               'image-processing': processed/'image-processed', 'lidar-processing': processed/'lidar-3d'}

    def inspect(item):
        video = Path(item['file'])
        role = item['role']
        out = args.out/role
        out.mkdir()
        command = [ffmpeg, '-hide_banner', '-v', 'error', '-xerror', '-err_detect', 'explode',
                   '-i', str(video), '-map', '0:v:0', '-progress', 'pipe:1', '-nostats', '-f', 'null', '-']
        decoded = subprocess.run(command, capture_output=True, text=True)
        (out/'decode-progress.txt').write_text(decoded.stdout)
        (out/'decode-errors.txt').write_text(decoded.stderr)
        progress = {}
        for line in decoded.stdout.splitlines():
            if '=' in line:
                key, value = line.split('=', 1)
                progress[key] = value
        metadata = subprocess.run([ffmpeg, '-hide_banner', '-i', str(video)], capture_output=True, text=True)
        (out/'container-metadata.txt').write_text(metadata.stderr)
        stream = next((line for line in metadata.stderr.splitlines() if 'Video:' in line), '')
        fps = re.search(r'([0-9.]+) fps', stream)
        size = re.search(r'\b([0-9]{2,5})x([0-9]{2,5})\b', stream)
        duration = re.search(r'Duration: (\d+):(\d+):([0-9.]+)', metadata.stderr)
        actual_duration = sum(float(v)*s for v,s in zip(duration.groups(), [3600,60,1])) if duration else None
        selection = '+'.join(f'eq(n\\,{i})' for i in indices)
        extracted = subprocess.run([ffmpeg, '-hide_banner', '-v', 'error', '-i', str(video),
                                   '-vf', 'select='+selection, '-fps_mode', 'vfr', str(out/'decoded-%03d.png')],
                                  capture_output=True, text=True)
        (out/'extract-errors.txt').write_text(extracted.stderr)
        samples = []
        for ordinal, index in enumerate(indices, 1):
            decoded_path = out/f'decoded-{ordinal:03d}.png'
            with Image.open(decoded_path) as im:
                encoded = im.convert('RGB')
            if role == 'sensors':
                original = Image.new('RGB', (1920,1080))
                inputs = []
                for i, name in enumerate(['chase','front','image-processing','lidar-processing']):
                    path = folders[name]/f'{ids[index]:08d}.png'
                    inputs.append({'path': str(path.resolve()), 'sha256': sha(path)})
                    with Image.open(path) as im:
                        original.paste(im.convert('RGB').resize((960,540), Image.Resampling.LANCZOS),
                                       ((i%2)*960,(i//2)*540))
                original_path = out/f'reference-quad-{ordinal:03d}.png'
                original.save(original_path)
            else:
                original_path = folders[role]/f'{ids[index]:08d}.png'
                inputs = [{'path': str(original_path.resolve()), 'sha256': sha(original_path)}]
                with Image.open(original_path) as im:
                    original = im.convert('RGB')
            error = np.asarray(ImageChops.difference(original, encoded), dtype=np.float64)
            mse = float(np.mean(error*error))
            samples.append({'index': index, 'native_frame': ids[index], 'time_s': index/item['fps'],
                            'source_inputs': inputs, 'reference_image': str(original_path.resolve()),
                            'decoded_image': str(decoded_path.resolve()), 'decoded_sha256': sha(decoded_path),
                            'source_encoded_psnr_db': float(10*np.log10(255*255/mse)) if mse else None,
                            'encoded_nonzero_fraction': float(np.count_nonzero(np.asarray(encoded))/np.asarray(encoded).size)})
        actual_frames = int(progress.get('frame', -1))
        actual_fps = float(fps.group(1)) if fps else None
        actual_size = [int(x) for x in size.groups()] if size else None
        actual_sha = sha(video)
        checks = {'full_decode_exit_zero': decoded.returncode == 0,
                  'no_error_level_decoder_messages': not decoded.stderr.strip(),
                  'decode_completed': progress.get('progress') == 'end',
                  'frame_count_matches': actual_frames == len(ids) == item['frames'],
                  'fps_matches': actual_fps == item['fps'],
                  'duration_matches': actual_duration is not None and abs(actual_duration-item['duration_s']) < .011,
                  'resolution_matches': actual_size == item['size'],
                  'sha_matches_manifest': actual_sha == item['sha256'],
                  'size_matches_manifest': video.stat().st_size == item['bytes'],
                  'sample_extraction_exit_zero': extracted.returncode == 0 and not extracted.stderr.strip()}
        result = {'role': role, 'video': str(video.resolve()), 'sha256': actual_sha,
                  'decode_command': command, 'actual_frames': actual_frames, 'actual_fps': actual_fps,
                  'actual_duration_s': actual_duration, 'actual_size': actual_size,
                  'audio_stream_present': 'Audio:' in metadata.stderr,
                  'checks': checks, 'automated_passed': all(checks.values()), 'samples': samples}
        print(json.dumps({'role': role, 'automated_passed': result['automated_passed'],
                          'frames': actual_frames, 'duration': actual_duration, 'fps': actual_fps}), flush=True)
        return result

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(inspect, source['outputs']))
    bindings = {'capture_manifest': (run/'manifest.json', source['capture_manifest_sha256']),
                'source_frames': (run/'frames.jsonl', source['source_frames_sha256']),
                'processing_manifest': (processed/'processing-manifest.json', source['processing_manifest_sha256'])}
    checks = {name: sha(path) == expected for name,(path,expected) in bindings.items()}
    report = {'reviewed_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'input_manifest': str(args.manifest.resolve()), 'input_manifest_sha256': sha(args.manifest),
              'review_script_sha256': sha(__file__), 'source_binding_checks': checks,
              'automated_passed': len(results)==6 and all(checks.values()) and all(r['automated_passed'] for r in results),
              'visual_inspection_status': 'pending_actual_view_image', 'outputs': results,
              'limitations': ['All video frames decoded; visual inspection is sampled and recorded separately.',
                             'Original camera PNGs and native point clouds remain unchanged.',
                             'Image masks are simulator ground truth, not a trained perception model.',
                             'LiDAR processing uses geometric ground/obstacle separation, not object identity tracking.',
                             'This media review does not replace action, motion, PNG integrity or sensor processing audits.']}
    (args.out/'media-review.json').write_text(json.dumps(report,indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
