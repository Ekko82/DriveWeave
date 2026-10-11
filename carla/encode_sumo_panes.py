"""Replace the upper-right pane with actual SUMO viewport recordings.

Recorded CARLA states replayed through SUMO are labelled as synchronized replay
in the manifest. This encoder never upgrades replay to live co-simulation.
"""
from pathlib import Path
import argparse, json, sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from encode_native import encode, sha
from PIL import Image


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--native-manifest', type=Path, required=True)
    p.add_argument('--sumo-manifest', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--name', required=True)
    p.add_argument('--sumo-frames', type=Path, required=True)
    p.add_argument('--limit', type=int)
    a = p.parse_args()
    nm = json.loads(a.native_manifest.read_text(encoding='utf-8'))
    sm = json.loads(a.sumo_manifest.read_text(encoding='utf-8'))
    run, processed = Path(nm['source_run']), Path(nm['processed_run'])
    ids = nm['native_frame_ids'][:a.limit]
    if not ids:
        raise ValueError('No source frames')
    fps = json.loads((run/'manifest.json').read_text(encoding='utf-8'))['fps']
    folders = [run/'chase', a.sumo_frames, processed/'image-processed', processed/'lidar-3d']
    for folder in folders:
        for frame in ids:
            if not (folder/f'{frame:08d}.png').is_file():
                raise ValueError(f'Missing frame {frame}: {folder}')
    a.out.mkdir(parents=True, exist_ok=True)
    def montage():
        for frame in ids:
            canvas = Image.new('RGB', (1920, 1080))
            for pane, folder in enumerate(folders):
                with Image.open(folder/f'{frame:08d}.png') as im:
                    canvas.paste(im.convert('RGB').resize((960,540), Image.Resampling.LANCZOS),
                                 ((pane%2)*960, (pane//2)*540))
            yield canvas
    destination = a.out/(a.name+'-CARLA与SUMO四视图.mp4')
    output = encode(destination, (1920,1080), fps, montage())
    manifest = {'source_run':str(run.resolve()), 'processed_run':str(processed.resolve()),
        'native_video_manifest':str(a.native_manifest.resolve()),
        'native_video_manifest_sha256':sha(a.native_manifest),
        'sumo_manifest':str(a.sumo_manifest.resolve()), 'sumo_manifest_sha256':sha(a.sumo_manifest),
        'sumo_frames':str(a.sumo_frames.resolve()), 'source_folders':[str(f.resolve()) for f in folders],
        'native_frame_ids':ids, 'outputs':[output],
        'panes':['native CARLA chase RGB','native SUMO viewport',
                 'native-mask image processing','processed native CARLA LiDAR 3D'],
        'sumo_mode':sm.get('mode', sm.get('source_mode', 'unclassified')),
        'added_text':False,'interpolated_frames':False,'playback_speed':1,
        'synthetic_sensor_data':False,'image_masks_are_simulator_ground_truth':True,
        'processing_controls_vehicle':False}
    target = a.out/(a.name+'-CARLA与SUMO视频清单.json')
    target.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps({'manifest':str(target),'output':output}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
