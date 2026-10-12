"""Original-speed native camera and sensor processing MP4s, with no added text."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from PIL import Image
from encode_native import encode, sha


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--processed',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--name',required=True)
    args=parser.parse_args()
    manifest=json.loads((args.run/'manifest.json').read_text())
    processed=json.loads((args.processed/'processing-manifest.json').read_text())
    records=[json.loads(x) for x in (args.run/'frames.jsonl').read_text().splitlines()]
    ids=[r['frame'] for r in records]
    if manifest['status']!='complete' or processed['generated_points'] or ids!=processed['native_frame_ids']:
        raise ValueError('Require a completed real capture and exactly matched native processing frames')
    args.out.mkdir(parents=True,exist_ok=False)
    folders={'chase':args.run/'chase','front':args.run/'rgb','rear':args.run/'rear',
        'image-processing':args.processed/'image-processed','lidar-processing':args.processed/'lidar-3d'}
    outputs=[]
    for name,folder in folders.items():
        if not all((folder/f'{frame:08d}.png').exists() for frame in ids):
            raise ValueError(f'Missing {name} native frames')
        with Image.open(folder/f'{ids[0]:08d}.png') as picture: size=picture.size
        def pictures():
            for frame in ids:
                with Image.open(folder/f'{frame:08d}.png') as im: yield im.convert('RGB')
        item=encode(args.out/f'{args.name}-{name}.mp4',size,manifest['fps'],pictures())
        item['role']=name
        outputs.append(item)
        print(json.dumps(item),flush=True)
    def montage():
        for frame in ids:
            canvas=Image.new('RGB',(1920,1080))
            for i,name in enumerate(['chase','front','image-processing','lidar-processing']):
                with Image.open(folders[name]/f'{frame:08d}.png') as im:
                    canvas.paste(im.convert('RGB').resize((960,540),Image.Resampling.LANCZOS),((i%2)*960,(i//2)*540))
            yield canvas
    item=encode(args.out/f'{args.name}-sensors.mp4',(1920,1080),manifest['fps'],montage())
    item['role']='sensors'
    outputs.append(item)
    result={'source_run':str(args.run.resolve()),'processed_run':str(args.processed.resolve()),
        'capture_manifest_sha256':sha(args.run/'manifest.json'),'source_frames_sha256':sha(args.run/'frames.jsonl'),
        'processing_manifest_sha256':sha(args.processed/'processing-manifest.json'),'native_frame_ids':ids,
        'outputs':outputs,'added_text':False,'playback_speed':1,'interpolated_frames':False,
        'image_masks_are_simulator_ground_truth':True,'pointclouds_are_native_returns':True,
        'raw_sources_modified':False,'final_validation_pending':True}
    (args.out/'media-manifest.json').write_text(json.dumps(result,indent=2),encoding='utf-8')


if __name__=='__main__': main()
