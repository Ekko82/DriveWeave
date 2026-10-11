"""Decode every frame and compare encoded panes against recorded sources."""
from pathlib import Path
import argparse, json, math, sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from encode_native import sha
import imageio_ffmpeg
import numpy as np
from PIL import Image


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a=p.parse_args()
    spec=json.loads(a.manifest.read_text(encoding='utf-8'))
    ids=spec['native_frame_ids']
    folders=[Path(v) for v in spec['source_folders']]
    failures=[]
    if not ids or any(b!=a+1 for a,b in zip(ids,ids[1:])):
        failures.append('Nonconsecutive native frames')
    for key in ['native_video_manifest','sumo_manifest']:
        if sha(spec[key])!=spec[key+'_sha256']:
            failures.append(key+' changed')
    if spec['added_text'] or spec['interpolated_frames'] or spec['playback_speed']!=1:
        failures.append('Unexpected presentation changes')
    reviews=[]
    selected={0,len(ids)//4,len(ids)//2,3*len(ids)//4,len(ids)-1}
    evidence=a.out.with_suffix('')
    evidence.mkdir(parents=True,exist_ok=True)
    for item in spec['outputs']:
        if sha(item['file'])!=item['sha256']:
            failures.append('Video SHA mismatch')
        reader=imageio_ffmpeg.read_frames(item['file'], pix_fmt='rgb24')
        metadata=next(reader)
        width,height=metadata['size']
        if [width,height]!=item['size'] or abs(metadata['fps']-item['fps'])>1e-6:
            failures.append('Video dimensions/FPS mismatch')
        count=0
        for index,data in enumerate(reader):
            count+=1
            actual=np.frombuffer(data,np.uint8).reshape(height,width,3)
            if float(actual.std())<1:
                failures.append(f'Empty encoded frame {index}')
            if index not in selected:
                continue
            reference=Image.new('RGB',(1920,1080))
            for n,folder in enumerate(folders):
                with Image.open(folder/f'{ids[index]:08d}.png') as im:
                    reference.paste(im.convert('RGB').resize((960,540),Image.Resampling.LANCZOS),
                                    ((n%2)*960,(n//2)*540))
            psnrs=[]
            ref=np.asarray(reference).astype(np.float32)
            diff=actual.astype(np.float32)-ref
            for y,x in [(0,0),(0,960),(540,0),(540,960)]:
                mse=float(np.square(diff[y:y+540,x:x+960]).mean())
                psnrs.append(10*math.log10(255**2/mse) if mse else 99.)
            if min(psnrs)<29:
                failures.append(f'Encoded source mismatch {index}: {psnrs}')
            target=evidence/f'sample-{index:05d}-native-{ids[index]:08d}.png'
            Image.fromarray(actual).save(target)
            reviews.append({'sample':index,'native_frame':ids[index],
                            'pane_psnr_db':psnrs,'image':str(target.resolve())})
        if count!=len(ids):
            failures.append(f'Decoded {count} frames, expected {len(ids)}')
    report={'automated_checks_passed':not failures,'failures':failures,
        'manifest':str(a.manifest.resolve()),'manifest_sha256':sha(a.manifest),
        'decoded_frames':count,'native_frames_expected':len(ids),
        'sumo_mode':spec['sumo_mode'],'manual_visual_review_required':True,'samples':reviews}
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({'pass':not failures,'decoded_frames':count,'report':str(a.out)},ensure_ascii=False),flush=True)
    if failures:
        raise SystemExit(1)


if __name__=='__main__':
    main()
