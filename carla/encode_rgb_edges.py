"""Encode true RGB-derived Sobel edges from the selected native frame range."""
from pathlib import Path
import sys, argparse, json
sys.path.insert(0,str(Path(__file__).resolve().parent))
from encode_native import encode,sha
from PIL import Image

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--manifest',type=Path,required=True)
a=p.parse_args()
m=json.loads(a.manifest.read_text(encoding='utf-8'))
frames=m['native_frame_ids']
source=Path(m['processed_run'])/'rgb-edges'
first=source/f'{frames[0]:08d}.png'
with Image.open(first) as im: size=im.size
def images():
    for frame in frames:
        with Image.open(source/f'{frame:08d}.png') as im: yield im.convert('RGB')
name=a.manifest.name.removesuffix('-视频清单.json')
output=encode(a.manifest.parent/(name+'-RGB边缘提取.mp4'),size,m['outputs'][0]['fps'],images())
proof={'source_video_manifest':str(a.manifest.resolve()),'source_video_manifest_sha256':sha(a.manifest),
       'native_frame_ids':frames,'output':output,'method':'Sobel magnitude applied to actual native RGB; percentile threshold',
       'uses_semantic_labels':False,'added_text':False,'trained_model':False,'interpolated_frames':False}
(a.manifest.parent/(name+'-RGB边缘提取清单.json')).write_text(json.dumps(proof,indent=2,ensure_ascii=False),encoding='utf-8')
print(json.dumps(output,ensure_ascii=False),flush=True)
