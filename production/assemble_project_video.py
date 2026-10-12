"""Edit hash-bound native clips at 20 fps; never synthesize simulator frames."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import imageio_ffmpeg


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    specification = json.loads(args.spec.read_text(encoding='utf-8'))
    args.out.mkdir(parents=True, exist_ok=False)
    parts = args.out / 'parts'
    parts.mkdir()
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    records, cursor = [], 0
    for index, clip in enumerate(specification['clips']):
        source = Path(clip['file']).resolve()
        if sha(source) != clip['sha256']:
            raise ValueError(f'Source SHA mismatch: {source.name}')
        start, end = clip['start_sample'], clip['end_sample_exclusive']
        if not 0 <= start < end <= clip['source_frames']:
            raise ValueError('Selection outside complete source')
        metadata_reader = imageio_ffmpeg.read_frames(str(source))
        metadata = next(metadata_reader)
        metadata_reader.close()
        if abs(metadata['fps'] - 20) > .001:
            raise ValueError('Only native 20 fps sources are supported')
        target = parts / f'{index:02d}.mp4'
        filters = [f'trim=start_frame={start}:end_frame={end}', 'setpts=N/(20*TB)']
        filters += ['scale=1920:1080:flags=lanczos', 'setsar=1', 'format=yuv420p']
        subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-nostdin',
            '-i', str(source), '-an', '-vf', ','.join(filters), '-c:v', 'libx264',
            '-preset', 'fast', '-crf', '18', '-threads', '4', '-r', '20',
            '-fps_mode', 'cfr', '-enc_time_base', '1:20', '-movflags', '+faststart',
            str(target)], check=True)
        record = dict(clip, part_sha256=sha(target), output_start_sample=cursor,
            output_end_sample_exclusive=cursor + end - start)
        records.append(record)
        cursor += end - start
        print(json.dumps({'part': index, 'chapter': clip['chapter'], 'frames': end-start}), flush=True)
    output = args.out / 'DriveWeave-traffic-and-sensors.mp4'
    writer = imageio_ffmpeg.write_frames(str(output), (1920, 1080), fps=20,
        codec='libx264', pix_fmt_in='rgb24', pix_fmt_out='yuv420p', macro_block_size=1,
        ffmpeg_log_level='error', output_params=['-preset', 'fast', '-crf', '18',
            '-threads', '4', '-g', '40', '-movflags', '+faststart'])
    writer.send(None)
    total = 0
    try:
        for index, record in enumerate(records):
            reader = imageio_ffmpeg.read_frames(str(parts / f'{index:02d}.mp4'))
            metadata = next(reader)
            if metadata['size'] != (1920, 1080):
                raise ValueError('Part dimensions mismatch')
            count = 0
            for frame in reader:
                writer.send(frame)
                count += 1
            if count != record['end_sample_exclusive'] - record['start_sample']:
                raise ValueError('Part decoded frame count mismatch')
            total += count
    finally:
        writer.close()
    result = subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-nostdin',
        '-i', str(output), '-map', '0:v:0', '-progress', 'pipe:1',
        '-fps_mode', 'passthrough', '-f', 'null', '-'], capture_output=True, text=True)
    if result.returncode or result.stderr.strip():
        raise RuntimeError(result.stderr)
    decoded = int([line.split('=', 1)[1] for line in result.stdout.splitlines() if line.startswith('frame=')][-1])
    reader = imageio_ffmpeg.read_frames(str(output))
    metadata = next(reader)
    reader.close()
    if decoded != cursor or total != cursor or abs(metadata['duration'] - cursor/20) > .01:
        raise ValueError('Output count or duration differs from source selections')
    report = {'status': 'encoded_and_full_decode_passed_visual_review_pending',
        'file': str(output.resolve()), 'sha256': sha(output), 'bytes': output.stat().st_size,
        'frames': cursor, 'fps': 20, 'duration_s': cursor/20, 'size': [1920,1080],
        'added_text': False, 'audio_added': False, 'playback_speed': 1,
        'interpolated_frames': False, 'full_decoded_frames': decoded,
        'specification_sha256': sha(args.spec), 'editorial_cuts': records,
        'limitations': specification['limitations'], 'published_to_bilibili': False}
    (args.out/'video-manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='editorial_cuts'}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
