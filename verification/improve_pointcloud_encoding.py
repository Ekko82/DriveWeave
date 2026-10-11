"""Improve only the native point-cloud video encoding, retaining original frames."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil

import imageio_ffmpeg
import numpy as np
from PIL import Image


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--crf',type=int,default=12)
    args=p.parse_args()
    original_manifest_hash=sha(args.manifest)
    manifest_backup=args.manifest.with_name(args.manifest.stem+'-原版-'+original_manifest_hash+'.json')
    if manifest_backup.exists():
        if sha(manifest_backup)!=original_manifest_hash:
            raise ValueError('Existing immutable manifest backup has an unexpected hash')
    else:
        shutil.copy2(args.manifest,manifest_backup)
    spec=json.loads(args.manifest.read_text(encoding='utf-8'))
    native=json.loads((Path(spec['source_run'])/'manifest.json').read_text(encoding='utf-8'))
    ids=spec['native_frame_ids']
    cloud=Path(spec['processed_run'])/'lidar-3d'
    item=next(i for i in spec['outputs'] if Path(i['file']).name.endswith('-点云处理.mp4'))
    target=Path(item['file'])
    previous_hash=sha(target)
    history=args.manifest.parents[2]/'verification/encoding-history'
    history.mkdir(parents=True,exist_ok=True)
    saved=history/(previous_hash+'.mp4')
    if not saved.exists():
        shutil.copy2(target,saved)
    temporary=target.with_name(target.stem+'-higher-quality-temporary.mp4')
    writer=imageio_ffmpeg.write_frames(str(temporary),item['size'],fps=native['fps'],codec='libx264',pix_fmt_in='rgb24',pix_fmt_out='yuv420p',macro_block_size=1,output_params=['-preset','medium','-crf',str(args.crf),'-movflags','+faststart'],ffmpeg_log_level='error')
    writer.send(None)
    try:
        for frame in ids:
            with Image.open(cloud/f'{frame:08d}.png') as im:
                writer.send(np.asarray(im.convert('RGB'),np.uint8))
    finally:
        writer.close()
    temporary.replace(target)
    item.update({'frames':len(ids),'duration_s':len(ids)/native['fps'],'sha256':sha(target),'bytes':target.stat().st_size,'encoding_crf':args.crf,'previous_video_sha256':previous_hash,'previous_encoding_preserved':str(saved),'source_frames_unchanged':True})
    args.manifest.write_text(json.dumps(spec,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'file':str(target),'frames':len(ids),'crf':args.crf,'sha256':item['sha256'],'previous_manifest_backup':str(manifest_backup),'previous_manifest_sha256':original_manifest_hash}))


if __name__=='__main__':
    main()
