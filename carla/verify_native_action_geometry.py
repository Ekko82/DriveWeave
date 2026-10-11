"""Independent geometry checks for the native action auditor; no server."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import json
import math
import numpy as np
import carla
from scipy.optimize import lsq_linear
from audit_native_actions import obb,obb_signed_clearance


def box(center,extent=(2,1,1),angles=(0,0,0),local_offset=(0,0,0),box_angles=(0,0,0)):
    pitch,yaw,roll=angles
    transform=carla.Transform(carla.Location(*center),carla.Rotation(pitch=pitch,yaw=yaw,roll=roll))
    return obb({'transform':{'world_from_local':transform.get_matrix()},
        'bounding_box':{'center_local_m':list(local_offset),'extent_m':list(extent),'rotation_rpy_deg':list(box_angles)}})


def oracle_distance(a,b):
    # Solve the convex bounded least-squares closest-points problem in two boxes.
    # This is independent from the SAT/vertices/edges feature-distance algorithm.
    matrix=np.hstack((a['axes'],-b['axes']))
    result=lsq_linear(matrix,b['center']-a['center'],bounds=(-np.r_[a['extent'],b['extent']],np.r_[a['extent'],b['extent']]),
        method='bvls',tol=1e-12,max_iter=1000)
    if not result.success:raise RuntimeError('Independent convex oracle failed')
    return float(np.linalg.norm(result.fun))


def main():
    fixtures=[('longitudinal_1m',box((0,0,0)),box((5,0,0)),1),
        ('diagonal_sqrt5m',box((0,0,0)),box((5,4,0)),math.sqrt(5)),
        ('vertically_stacked_2m',box((0,0,0)),box((0,0,4)),2),
        ('touching',box((0,0,0)),box((4,0,0)),0),
        ('overlap',box((0,0,0)),box((3,0,0)),-1),
        ('translated_bbox_offset',box((0,0,0),local_offset=(1,0,0)),box((6,0,0)),1)]
    results=[]
    for name,a,b,expected in fixtures:
        actual=obb_signed_clearance(a,b)
        if abs(actual-expected)>1e-5:raise RuntimeError(f'{name}: {actual} != {expected}')
        results.append({'name':name,'actual_signed_clearance_m':actual,'expected_m':expected})
    rng=np.random.default_rng(20261011)
    errors=[];overlaps=0
    for _ in range(100):
        a=box(rng.uniform(-5,5,3),rng.uniform(.2,3,3),rng.uniform(-180,180,3),rng.uniform(-1,1,3),rng.uniform(-25,25,3))
        b=box(rng.uniform(-5,5,3),rng.uniform(.2,3,3),rng.uniform(-180,180,3),rng.uniform(-1,1,3),rng.uniform(-25,25,3))
        measured=obb_signed_clearance(a,b);oracle=oracle_distance(a,b)
        if measured<0:
            overlaps+=1
            if oracle>1e-5:raise RuntimeError(f'False overlap: oracle distance={oracle}')
        else:
            errors.append(abs(measured-oracle))
            if abs(measured-oracle)>1e-5:raise RuntimeError(f'Exact 3D distance mismatch: measured={measured}, oracle={oracle}')
    report={'passed':True,'fixtures':results,'independent_random_box_pairs':100,'overlap_pairs':overlaps,
        'maximum_separated_distance_error_m':max(errors),'oracle':'SciPy convex bounded least-squares closest points; independent of SAT/feature algorithm',
        'native_server_connected':False,'native_simulation_started':False,'scenario_actions_validated':False}
    out=Path(__file__).resolve().parent/'plans'/'native-action-auditor-geometry-verification.json'
    out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
