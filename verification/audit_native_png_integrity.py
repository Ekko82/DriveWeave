"""Prove lossless PNG camera pixels reconstruct each exact native BGRA callback."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image


def audit(run, out):
    manifest=json.loads((run/'manifest.json').read_text())
    source=run/'frames.jsonl'
    counts={}
    failures=[]
    for line in source.read_text().splitlines():
        record=json.loads(line)
        for name,sensor in record['sensors'].items():
            if name=='lidar': continue
            path=run/('semantic_labels' if name=='semantic' else name)/f"{record['frame']:08d}.png"
            pixels=np.asarray(Image.open(path))
            bgra=np.zeros((sensor['height'],sensor['width'],4),dtype=np.uint8)
            bgra[:,:,3]=255
            if name=='semantic': bgra[:,:,2]=pixels
            else: bgra[:,:,:3]=pixels[:,:,::-1]
            actual=hashlib.sha256(bgra.tobytes()).hexdigest()
            counts[name]=counts.get(name,0)+1
            if actual!=sensor['raw_sha256']:
                failures.append({'frame':record['frame'],'sensor':name,'actual_reconstructed_sha256':actual,
                    'recorded_native_callback_sha256':sensor['raw_sha256']})
    passed=bool(manifest['status']=='complete' and counts and not failures and all(v==manifest['frames_captured'] for v in counts.values()))
    result={'pass':passed,'run':str(run.resolve()),'frames_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'manifest_sha256':hashlib.sha256((run/'manifest.json').read_bytes()).hexdigest(),
        'camera_counts':counts,'failures':failures,
        'method':'Decode actual lossless PNG, restore native BGRA ordering/alpha255 and semantic zero B/G, require exact SHA256 equality with each original sensor callback. Any differing alpha or channel fails.',
        'raw_or_png_changed':False,'replacement_samples_generated':False}
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result),flush=True)
    return passed


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--run',type=Path,required=True)
    cli.add_argument('--out',type=Path,required=True)
    args=cli.parse_args()
    raise SystemExit(0 if audit(args.run,args.out) else 1)
