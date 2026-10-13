"""Explicit source-bound processing for two reviewed native boundary deviations.

The frozen ordinary processor and its strict policy are unchanged. This entry
allows only the byte-exact reviewed native returns and preserves their strict
failures in the output. It never drops or changes a raw return to pass a gate.
"""
from pathlib import Path
import copy
import hashlib
import json
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fuse_production_sensors as base

APPROVED = {
    'f73b030b09d2a7f830098d21c74cb7d9c695696787b3076f34b2eea750776064': 3,
    '34d9b0347accc0c74be8c3f219a4f5fe55a7000419326b48f6ab93adabde17ed': 1,
}
FROZEN = {
    'carla/fuse_production_sensors.py': '5e1e49461f38b0f809efdca1b3dcf3d6361470f06d3e369ef0e48f7d50fc8ac0',
    'carla/render_production_fusion.py': '3bae2c6577e870c2933891399138dce1fe2f85e9eb3e1eaa08e0d629cd1176ae',
    'carla/detect_rgb_onnx.py': '44030538fad855573d25d24b09f482b9a145403f1141967e33b24518a5121dba',
    'carla/fuse_camera_lidar.py': '923420089f3f0319ee283c7c1294ba21266684e572d5f9e4311ebaf028b96fd3',
    'verification/process_carla_sensors.py': '9dbcaeade31f8d3dce102baaffd6f17a4d34e59eec5d4ffff37881f962ad07df',
}
STRICT_VALIDATE = base.validate_native_fov
ORIGINAL_PREPROCESS = base.preprocess_points


def raw_sha(raw):
    return hashlib.sha256(raw.astype('<f4', copy=False).tobytes()).hexdigest()


def outside_indices(raw):
    x, y, z = raw[:, :3].astype(np.float64).T
    radius = np.hypot(x, y)
    half, upper, lower = np.deg2rad([60., 12.9, -12.5])
    h = np.maximum(np.abs(y)*np.cos(half)-x*np.sin(half), 0.)
    v = np.maximum(np.maximum(z*np.cos(upper)-radius*np.sin(upper), 0.),
                   np.maximum(radius*np.sin(lower)-z*np.cos(lower), 0.))
    ha = np.maximum(np.abs(np.degrees(np.arctan2(y, x)))-60., 0.)
    elevation = np.degrees(np.arctan2(z, radius))
    va = np.maximum(np.maximum(elevation-12.9, -12.5-elevation), 0.)
    return np.flatnonzero((h > .005) | (v > .005) | (ha > .02) | (va > .02) | (x < 0))


class ExactNativeReview:
    def __init__(self, entries):
        self.entries = copy.deepcopy(entries)
        self.expected = {(item['frame'], item['sample'], item['raw_index']) for item in entries}
        base.require(len(self.expected) == len(entries), 'Duplicate reviewed native return')
        self.seen = set()

    def validate(self, raw, row):
        base.require(raw.ndim == 2 and raw.shape[1] == 4 and raw.dtype == np.dtype('float32') and len(raw),
                     'Expected native float32 Nx4')
        base.require(np.isfinite(raw).all(), 'Nonfinite returns can never be reviewed exceptions')
        base.require(np.all(raw[:, 0] >= 0), 'Negative x returns can never be reviewed exceptions')
        digest = raw_sha(raw)
        base.require(digest == row['sensors']['lidar']['raw_sha256'], 'Native callback SHA differs')
        entries = [item for item in self.entries if item['frame'] == row['frame']]
        try:
            metrics = STRICT_VALIDATE(raw)
        except ValueError as error:
            # The exact frozen strict validator serializes its measured metrics
            # in this failure. Preserve them instead of turning failure into pass.
            marker = 'no crop fallback: '
            if marker not in str(error):
                raise
            metrics = json.loads(str(error).split(marker, 1)[1])
        actual = outside_indices(raw)
        base.require({int(index) for index in actual} == {item['raw_index'] for item in entries},
                     'Actual strict failure set differs from reviewed exact native returns')
        base.require(metrics['engineering_tolerance_exceeded_points'] == len(actual), 'Strict metrics disagree')
        for item in entries:
            base.require(item['sample'] == row['sample'], 'Reviewed sample differs')
            base.require(item['raw_callback_sha256'] == digest, 'Reviewed callback SHA differs')
            point_bytes = raw[item['raw_index']].astype('<f4', copy=False).tobytes().hex()
            base.require(point_bytes == item['raw_point_float32_le_hex'], 'Reviewed raw point bytes differ')
            self.seen.add((row['frame'], row['sample'], item['raw_index']))
        return dict(metrics, reviewed_fov_exception_count=len(entries), strict_fov_passed=not len(actual))

    def require_complete(self):
        base.require(self.seen == self.expected, 'Not every reviewed native return was encountered')


def load_review(run, path, expected_sha):
    base.require(expected_sha in APPROVED and base.sha(path) == expected_sha, 'Unapproved or changed exception sidecar')
    sidecar = base.read(path)
    base.require(sidecar['schema_version'] == 1 and sidecar['strict_policy'] ==
                 {'boundary_m': .005, 'angle_deg': .02, 'minimum_x_m': 0.}, 'Strict policy must remain unchanged')
    base.require((base.ROOT/sidecar['source_run']).resolve() == run.resolve(), 'Sidecar belongs to another run')
    for name, key in [('manifest.json', 'source_manifest_sha256'), ('frames.jsonl', 'source_frames_sha256'),
                      ('sensor-profile.json', 'source_sensor_profile_sha256')]:
        base.require(base.sha(run/name) == sidecar[key], 'Reviewed native source SHA differs: '+name)
    count = APPROVED[expected_sha]
    base.require(sidecar['reviewed_exception_count'] == len(sidecar['exceptions']) == count,
                 'Reviewed exact exception count differs')
    report_path = (base.ROOT/sidecar['source_diagnostic_report']).resolve()
    base.require(report_path.is_relative_to(base.ROOT)
                 and base.sha(report_path) == sidecar['source_diagnostic_report_sha256'], 'Strict diagnostic changed')
    return sidecar, report_path


def process(args):
    run, out = args.run.resolve(), args.out.resolve()
    path = args.exceptions.resolve()
    base.require(not out.exists(), 'Never overwrite ordinary failed or prior qualified output')
    base.require(path.is_relative_to(base.ROOT), 'Sidecar must be a bound project source')
    base.require(args.start_sample == 0 and args.end_sample is None, 'Review requires the full native capture')
    base.require(all(base.sha(base.ROOT/name) == digest for name, digest in FROZEN.items()), 'Frozen module changed')
    sidecar, report_path = load_review(run, path, args.exceptions_sha256)
    rows = [json.loads(line) for line in (run/'frames.jsonl').read_text(encoding='utf-8').splitlines()]
    # Scan the entire unchanged native source before expensive inference, so a
    # fourth/unlisted violation cannot be hidden late in the recording.
    preflight = ExactNativeReview(sidecar['exceptions'])
    for row in rows:
        raw = np.load(run/'lidar'/f"{row['frame']:08d}.npy", allow_pickle=False)
        preflight.validate(raw, row)
    preflight.require_complete()
    guard = ExactNativeReview(sidecar['exceptions'])
    original_modules = list(base.MODULES)
    base.MODULES = [str(Path(__file__).resolve().relative_to(base.ROOT)).replace('\\', '/'),
                    *original_modules, str(path.relative_to(base.ROOT)).replace('\\', '/'),
                    str(report_path.relative_to(base.ROOT)).replace('\\', '/')]

    def reviewed_preprocess(raw, row):
        metrics = guard.validate(raw, row)

        def already_checked(candidate):
            base.require(candidate is raw, 'Geometry validation received a different raw array')
            return metrics

        # Scoped dependency substitution only within this explicitly bound entry.
        # The frozen original geometry runs on the identical raw array and all
        # normal range/self/voxel/ground operations remain exactly the same.
        base.validate_native_fov = already_checked
        try:
            return ORIGINAL_PREPROCESS(raw, row)
        finally:
            base.validate_native_fov = STRICT_VALIDATE

    base.preprocess_points = reviewed_preprocess
    try:
        base.process(args)
    finally:
        base.preprocess_points = ORIGINAL_PREPROCESS
        base.validate_native_fov = STRICT_VALIDATE
    guard.require_complete()
    load_review(run, path, args.exceptions_sha256)
    base.require(all(base.sha(base.ROOT/name) == digest for name, digest in FROZEN.items()), 'Frozen module changed')
    review = {'sidecar_path': str(path.relative_to(base.ROOT)).replace('\\', '/'),
              'sidecar_sha256': args.exceptions_sha256, 'reviewed_exception_count': len(sidecar['exceptions']),
              'strict_fov_passed': False, 'normal_pass': False, 'raw_points_modified': False,
              'strict_policy_changed': False, 'source_diagnostic_report_sha256': sidecar['source_diagnostic_report_sha256'],
              'acceptance': 'Only the exact source-bound reviewed native returns; ordinary strict policy still fails',
              'cause': 'Not established; no unique numerical or physical cause is claimed'}
    processing_path, fusion_path = out/'processing-manifest.json', out/'fusion-manifest.json'
    processing, fusion = base.read(processing_path), base.read(fusion_path)
    for manifest in (processing, fusion):
        manifest.update(status='qualified_native_source_processing_pending_independent_review',
                        native_fov_exception_review=review, strict_fov_passed=False, normal_pass=False,
                        qualified_source_processing=True, exception_sha256=args.exceptions_sha256)
    processing_path.write_text(json.dumps(processing, indent=2)+'\n', encoding='utf-8')
    fusion['source_processing_manifest_sha256'] = base.sha(processing_path)
    fusion['limits'].append('Ordinary strict FOV validation failed for '+str(len(sidecar['exceptions']))+
                           ' exact source-bound native boundary returns; all original returns are preserved and this result is qualified.')
    fusion_path.write_text(json.dumps(fusion, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'status': fusion['status'], 'strict_fov_passed': False,
                      'reviewed_exception_count': len(sidecar['exceptions']), 'frames': fusion['frames']}), flush=True)


if __name__ == '__main__':
    parser = base.parser()
    parser.description = __doc__
    parser.add_argument('--exceptions', type=Path, required=True)
    parser.add_argument('--exceptions-sha256', required=True)
    process(parser.parse_args())
