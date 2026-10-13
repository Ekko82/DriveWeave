"""Re-render frozen fusion predictions with collision-aware compact labels.

Only display pixels/hashes change. Detection boxes, point membership, ranges,
scores and IDs must remain byte-for-byte equal after removing the two PNG hashes.
"""
from pathlib import Path
import argparse
import json
import sys

import numpy as np
from PIL import Image

sys.path.insert(0,str(Path(__file__).resolve().parent))
import fuse_camera_lidar as fusion


class Labels:
    def __init__(self):
        self.draw=None
        self.occupied=[]

    def __call__(self,draw,xy,text,color,size=17):
        if draw is not self.draw:
            self.draw,self.occupied=draw,[]
        width,height=draw._image.size
        face=fusion.font(size)
        box=draw.textbbox((0,0),text,font=face)
        w,h=box[2]-box[0]+6,box[3]-box[1]+4
        original_x,original_y=xy
        selected=None
        for offset in [0,-25,25,-50,50,-75,75,-100,100,-125,125]:
            x=float(np.clip(original_x,4,max(4,width-w-4)))
            y=float(np.clip(original_y+offset,4,max(4,height-h-4)))
            rect=(x-3,y+box[1]-2,x+w-3,y+box[1]+h-2)
            if all(rect[2]+3<r[0] or rect[0]-3>r[2] or rect[3]+3<r[1] or rect[1]-3>r[3]
                   for r in self.occupied):
                selected=(x,y,rect)
                break
        if selected is None:
            # Keep every genuine result even in a crowded view; do not hide it.
            x,y=float(np.clip(original_x,4,width-w-4)),float(np.clip(original_y,4,height-h-4))
            selected=(x,y,(x-3,y+box[1]-2,x+w-3,y+box[1]+h-2))
        x,y,rect=selected
        if abs(y-original_y)>8:
            draw.line((original_x,original_y+h,x,y+h/2),fill=color,width=1)
        draw.rectangle(rect,fill=(9,16,24))
        draw.text((x,y),text,font=face,fill=color)
        self.occupied.append(rect)


def rerender(run,processed,source,out,min_confidence=None):
    manifest=json.loads((run/'manifest.json').read_text())
    rows={row['frame']:row for row in map(json.loads,(run/'frames.jsonl').read_text().splitlines())}
    original=json.loads((source/'fusion-manifest.json').read_text())
    if fusion.sha(source/'associations.jsonl')!=original['associations_sha256']:
        raise ValueError('Frozen associations SHA mismatch')
    out.mkdir(parents=True,exist_ok=False)
    (out/'frames').mkdir()
    records=[json.loads(line) for line in (source/'associations.jsonl').read_text().splitlines()]
    if min_confidence is not None and not original['confidence_threshold'] <= min_confidence < 1:
        raise ValueError('Confidence filter must only raise the frozen detection threshold')
    fusion.label=Labels()
    with (out/'associations.jsonl').open('w',encoding='utf-8') as stream:
        for index,record in enumerate(records):
            if min_confidence is not None:
                record['detections']=[item for item in record['detections'] if item['score']>=min_confidence]
            immutable=json.dumps({k:v for k,v in record.items() if k not in ('camera_sha256','lidar_sha256')},sort_keys=True)
            frame=record['frame']; token=f'{frame:08d}'; row=rows[frame]
            raw=np.load(run/'lidar'/f'{token}.npy',allow_pickle=False)
            data_path=processed/'data'/f'{token}.npz'
            if fusion.sha(data_path)!=record['geometric_points_npz_sha256']:
                raise ValueError('Frozen geometric source changed')
            with np.load(data_path,allow_pickle=False) as data:
                xyz=raw[data['sampled_raw_indices'],:3]
                ground=data['ground_mask'].copy()
            rgb_path=run/'rgb'/f'{token}.png'
            if fusion.sha(rgb_path)!=record['rgb_file_sha256']:
                raise ValueError('Frozen RGB source changed')
            rgb=np.asarray(Image.open(rgb_path).convert('RGB'))
            projected,uv,_=fusion.project(xyz,fusion.matrix(row['sensors']['lidar']),
                fusion.matrix(row['sensors']['rgb']),manifest['camera_intrinsic'],rgb.shape[1],rgb.shape[0])
            camera,lidar=fusion.render(rgb,xyz,ground,projected,uv,record['detections'])
            for name,picture in [('camera',camera),('lidar',lidar)]:
                output=out/'frames'/f'{token}-{name}.png'
                picture.save(output,compress_level=1)
                record[name+'_sha256']=fusion.sha(output)
            after=json.dumps({k:v for k,v in record.items() if k not in ('camera_sha256','lidar_sha256')},sort_keys=True)
            if immutable!=after:
                raise ValueError('Display pass modified an inference result')
            stream.write(json.dumps(record,separators=(',',':'))+'\n')
            if (index+1)%200==0:
                print(json.dumps({'rendered':index+1,'frames':len(records)}),flush=True)
    original.update(associations_sha256=fusion.sha(out/'associations.jsonl'),
        display_source_manifest_sha256=fusion.sha(source/'fusion-manifest.json'),
        display_source_associations_sha256=fusion.sha(source/'associations.jsonl'),
        display_renderer_sha256=fusion.sha(__file__),
        display_change='collision-aware label positions only; predictions and all numeric values preserved',
        predictions_unchanged_after_display_pass=True)
    if min_confidence is not None:
        original.update(confidence_threshold=min_confidence,
            source_inference_confidence_threshold=json.loads((source/'fusion-manifest.json').read_text())['confidence_threshold'],
            confidence_filter_after_inference=min_confidence,
            predictions_unchanged_after_display_pass=False,
            retained_predictions_numerically_unchanged=True,
            display_change='Uniform confidence threshold on every class/frame, plus collision-aware labels; retained boxes/points/ranges/IDs unchanged',
            detected_boxes=sum(len(r['detections']) for r in records),
            associated_boxes=sum(d['fusion_status']=='associated' for r in records for d in r['detections']),
            frames_with_association=sum(any(d['fusion_status']=='associated' for d in r['detections']) for r in records))
        original['limits'] += ['Display confidence policy was selected after review of these recordings; this set is not an independent benchmark.',
                              'The higher threshold suppresses some correct distant or occluded detections as well as false detections.']
    (out/'fusion-manifest.json').write_text(json.dumps(original,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'status':'rendered_frozen_predictions','frames':len(records),'out':str(out)}),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run','processed','fusion','out'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--min-confidence',type=float)
    args=parser.parse_args()
    rerender(args.run,args.processed,args.fusion,args.out,args.min_confidence)
