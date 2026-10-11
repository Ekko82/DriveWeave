"""Encode synchronized real CARLA frames without titles, HUDs or interpolation."""
from pathlib import Path
import argparse, hashlib, json, subprocess
import numpy as np
from PIL import Image
import imageio_ffmpeg

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        while b:=f.read(1<<20): h.update(b)
    return h.hexdigest()

def encode(destination, size, fps, frames):
    writer=imageio_ffmpeg.write_frames(str(destination),size,fps=fps,codec='libx264',
        pix_fmt_in='rgb24',pix_fmt_out='yuv420p',macro_block_size=1,
        output_params=['-preset','medium','-crf','18','-movflags','+faststart'],ffmpeg_log_level='error')
    writer.send(None)
    count=0
    try:
        for frame in frames:
            writer.send(np.asarray(frame,dtype=np.uint8))
            count+=1
    finally: writer.close()
    return {'file':str(destination.resolve()),'frames':count,'fps':fps,'duration_s':count/fps,
            'size':list(size),'sha256':sha(destination),'bytes':destination.stat().st_size}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--processed',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--name',default='CARLA-Tesla')
    p.add_argument('--start-sample',type=int,default=0)
    p.add_argument('--end-sample',type=int)
    p.add_argument('--individual',action='store_true')
    p.add_argument('--native-individual',action='store_true',help='Encode only native chase and front RGB individually, in addition to the synchronized quad')
    a=p.parse_args()
    m=json.loads((a.run/'manifest.json').read_text(encoding='utf-8'))
    pm=json.loads((a.processed/'processing-manifest.json').read_text(encoding='utf-8'))
    if m['status']!='complete' or m['synthetic_sensor_fallback'] or pm['generated_points']:
        raise ValueError('Only completed native data and source-indexed point processing are accepted')
    records=[json.loads(l) for l in (a.run/'frames.jsonl').read_text(encoding='utf-8').splitlines()]
    records=records[a.start_sample:a.end_sample]
    ids=[r['frame'] for r in records]
    if not ids or not set(ids).issubset(pm['native_frame_ids']):
        raise ValueError('Missing synchronized processing frames')
    folders=[a.run/'chase',a.run/'rgb',a.processed/'image-processed',a.processed/'lidar-3d']
    for folder in folders:
        if not all((folder/f'{i:08d}.png').exists() for i in ids):
            raise ValueError(f'Missing native frame in {folder}')
    a.out.mkdir(parents=True,exist_ok=True)
    def montage():
        for i in ids:
            canvas=Image.new('RGB',(1920,1080))
            for n,folder in enumerate(folders):
                with Image.open(folder/f'{i:08d}.png') as im:
                    pane=im.convert('RGB').resize((960,540),Image.Resampling.LANCZOS)
                    canvas.paste(pane,((n%2)*960,(n//2)*540))
            yield canvas
    outputs=[encode(a.out/(a.name+'-同步四视图.mp4'),(1920,1080),m['fps'],montage())]
    if a.individual or a.native_individual:
        labels=['原生跟车','原生前视RGB','图像处理','点云处理']
        individual_folders=folders if a.individual else folders[:2]
        for folder,label in zip(individual_folders,labels):
            with Image.open(folder/f'{ids[0]:08d}.png') as im: size=im.size
            def pictures(folder=folder):
                for i in ids:
                    with Image.open(folder/f'{i:08d}.png') as im: yield im.convert('RGB')
            outputs.append(encode(a.out/(a.name+'-'+label+'.mp4'),size,m['fps'],pictures()))
    manifest={'source_run':str(a.run.resolve()),'processed_run':str(a.processed.resolve()),
        'capture_manifest_sha256':sha(a.run/'manifest.json'),'processing_manifest_sha256':sha(a.processed/'processing-manifest.json'),
        'native_frame_ids':ids,'outputs':outputs,'panes':['native chase RGB','native front RGB','native-mask image processing','processed native LiDAR 3D'],
        'added_text':False,'interpolated_frames':False,'playback_speed':1,'synthetic_sensor_data':False,
        'image_masks_are_simulator_ground_truth':True,'processing_controls_vehicle':False}
    (a.out/(a.name+'-视频清单.json')).write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(outputs,ensure_ascii=False),flush=True)

if __name__=='__main__': main()
