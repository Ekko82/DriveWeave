"""Independent exact-source review of approved native boundary returns.

The frozen strict auditor really runs. Its expected horizontal FOV failures and
legacy single-entrypoint assertion remain failures. This review independently
verifies the frozen new entrypoint and original dependencies. No failed assertion
is rewritten as passed; strict FOV compliance remains false.
"""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import sys

import numpy as np

STRICT_AUDITOR_SHA256 = 'a68dd6dbce6295c994e5b583451194b7acab176d09147759b6eddea79fcf93d4'
QUALIFIED_ENTRYPOINT = 'carla/fuse_production_sensors_qualified.py'
ENTRYPOINT_ASSERTION = 'Processing entry-point SHA differs'
FROZEN_SOURCE_SHA256 = {
    QUALIFIED_ENTRYPOINT: '1f9cf47671d64a43273d7158ad8adabcb24a9630524f2da2afcb3b8b594db69c',
    'carla/fuse_production_sensors.py': '5e1e49461f38b0f809efdca1b3dcf3d6361470f06d3e369ef0e48f7d50fc8ac0',
    'carla/render_production_fusion.py': '3bae2c6577e870c2933891399138dce1fe2f85e9eb3e1eaa08e0d629cd1176ae',
    'carla/detect_rgb_onnx.py': '44030538fad855573d25d24b09f482b9a145403f1141967e33b24518a5121dba',
    'carla/fuse_camera_lidar.py': '923420089f3f0319ee283c7c1294ba21266684e572d5f9e4311ebaf028b96fd3',
    'verification/process_carla_sensors.py': '9dbcaeade31f8d3dce102baaffd6f17a4d34e59eec5d4ffff37881f962ad07df',
}
APPROVED_SIDECARS = {
    'f73b030b09d2a7f830098d21c74cb7d9c695696787b3076f34b2eea750776064': {'scene': 'ramp-entry', 'count': 3},
    '34d9b0347accc0c74be8c3f219a4f5fe55a7000419326b48f6ab93adabde17ed': {'scene': 'roundabout', 'count': 1},
}
SOURCE_KEYS = {'manifest.json': 'source_manifest_sha256', 'frames.jsonl': 'source_frames_sha256',
               'sensor-profile.json': 'source_sensor_profile_sha256'}


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def native_deviations(raw):
    """Independent polar geometry, never the processing validator's code."""
    require(raw.ndim == 2 and raw.shape[1] == 4 and raw.dtype == np.float32 and len(raw),
            'Invalid native float32 XYZI array')
    require(np.isfinite(raw).all(), 'Nonfinite raw point can never be excepted')
    xyz = raw[:, :3].astype(np.float64)
    require(np.all(xyz[:, 0] >= 0), 'Negative-x raw point can never be excepted')
    rho = np.hypot(xyz[:, 0], xyz[:, 1])
    radius = np.linalg.norm(xyz, axis=1)
    azimuth = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
    elevation = np.degrees(np.arctan2(xyz[:, 2], rho))
    horizontal = np.maximum(abs(azimuth) - 60., 0.)
    vertical = np.maximum.reduce([np.zeros(len(raw)), elevation - 12.9, -12.5 - elevation])
    horizontal_m = rho * np.sin(np.radians(horizontal))
    vertical_m = radius * np.sin(np.radians(vertical))
    invalid = (horizontal > .02) | (vertical > .02) | (horizontal_m > .005) | (vertical_m > .005)
    return invalid, {'horizontal_angle_excess_deg': horizontal, 'vertical_angle_excess_deg': vertical,
                     'horizontal_boundary_excess_m': horizontal_m, 'vertical_boundary_excess_m': vertical_m}


def exception_inventory(sidecar, expected_count=3):
    require(sidecar.get('schema_version') == 1, 'Unknown exception schema')
    require(sidecar.get('strict_policy') == {'boundary_m': .005, 'angle_deg': .02, 'minimum_x_m': 0},
            'Exception file changes the strict policy')
    items = sidecar['exceptions']
    require(len(items) == expected_count, 'Only this source-specific reviewed return inventory is allowed')
    result = {}
    for item in items:
        key = item['frame'], item['raw_index']
        require(all(type(x) is int and x >= 0 for x in key) and type(item['sample']) is int,
                'Exception frame, sample and raw row must be integers')
        require(key not in result, 'Duplicate exception point')
        require(len(bytes.fromhex(item['raw_point_float32_le_hex'])) == 16, 'Exception must bind all four float32 values')
        result[key] = item
    return result


def check_exception_point(item, frame, sample, raw_index, raw, callback_sha, deviations):
    require((item['frame'], item['sample'], item['raw_index']) == (frame, sample, raw_index),
            'Exception native row identity differs')
    require(item['raw_callback_sha256'] == callback_sha, 'Exception callback byte SHA differs')
    require(item['raw_point_float32_le_hex'] == raw[raw_index].astype('<f4', copy=False).tobytes().hex(),
            'Exception raw XYZI bytes differ')
    for key, values in deviations.items():
        expected = item[key]
        require(isinstance(expected, (int, float)) and np.isfinite(expected) and
                abs(float(expected) - float(values[raw_index])) <= 1e-9,
                'Exception measured deviation differs: ' + key)


def review_raw_inventory(run, sidecar, expected_count=3):
    approved = exception_inventory(sidecar, expected_count)
    observed, count, examined = set(), 0, 0
    rows = [json.loads(line) for line in (run / 'frames.jsonl').read_text(encoding='utf-8').splitlines()]
    reviewed = []
    for row in rows:
        frame = row['frame']
        raw = np.load(run / 'lidar' / f'{frame:08d}.npy', allow_pickle=False)
        invalid, deviations = native_deviations(raw)
        callback_sha = hashlib.sha256(raw.astype('<f4', copy=False).tobytes()).hexdigest()
        require(callback_sha == row['sensors']['lidar']['raw_sha256'], 'Native callback provenance differs')
        count += len(raw)
        examined += 1
        for index in np.flatnonzero(invalid):
            key = frame, int(index)
            require(key in approved, 'Unreviewed native FOV violation: ' + str(key))
            item = approved[key]
            check_exception_point(item, frame, row['sample'], int(index), raw, callback_sha, deviations)
            observed.add(key)
            reviewed.append({'frame': frame, 'sample': row['sample'], 'raw_index': int(index),
                             **{name: float(values[index]) for name, values in deviations.items()}})
    require(observed == set(approved), 'Reviewed exception set differs from all observed violations')
    require(len(rows) == read(run / 'manifest.json')['frames_captured'], 'Native capture frame count differs')
    return {'frames': examined, 'native_raw_points': count, 'reviewed_points': reviewed}


def check_strict_result(report, actual_exit_code, reviewed, expected_count=3):
    fov_failures = [f"LiDAR {point['frame']}: native horizontal FOV engineering budget exceeded" for point in reviewed]
    require(len(set(fov_failures)) == expected_count, 'Expected reviewed points must occupy their distinct source frames')
    expected = sorted([*fov_failures, ENTRYPOINT_ASSERTION])
    require(actual_exit_code == 1 and report.get('status') == 'failed', 'Strict auditor must retain its real failure')
    require(report.get('auditor_sha256') == STRICT_AUDITOR_SHA256, 'Strict report came from another auditor')
    require(sorted(report['failures']) == expected and
            report['assertions']['failed_assertions'] == len(expected) and not report['failures_truncated'],
            'Strict report contains unexpected failures or conceals an expected FOV failure')
    require(report['assertions']['passed_assertions'] + len(expected) == report['assertions']['assertions'],
            'Strict assertion accounting differs')
    require(report['all_output_frames_geometry_audited'] is True and report['stride'] == 1,
            'Qualified review requires full strict geometry audit')
    require(report['accuracy_claimed'] is False, 'Accuracy cannot be inferred from this review')


def verify_entrypoint_provenance(document, source_root):
    """Prove the actual new entrypoint, rather than waiving arbitrary hash errors."""
    modules = document.get('source_modules_sha256', {})
    require(document.get('script_sha256') == modules.get(QUALIFIED_ENTRYPOINT) ==
            FROZEN_SOURCE_SHA256[QUALIFIED_ENTRYPOINT], 'Qualified script SHA does not match the frozen actual entrypoint')
    for name, expected in FROZEN_SOURCE_SHA256.items():
        require(modules.get(name) == expected, 'Frozen source mapping differs: ' + name)
        require(sha(source_root / name) == expected, 'Frozen source file bytes differ: ' + name)
    return {'explicit_entrypoint_provenance_verified': True, 'actual_entrypoint': QUALIFIED_ENTRYPOINT,
            'actual_entrypoint_sha256': FROZEN_SOURCE_SHA256[QUALIFIED_ENTRYPOINT],
            'preserved_legacy_strict_failure': ENTRYPOINT_ASSERTION,
            'underlying_processor_and_dependencies_frozen': True,
            'meaning': 'The old audit assumes the ordinary processor is the sole entrypoint. The qualified entrypoint and every original dependency are independently SHA-bound here.'}


def check_exception_declaration(document, sidecar_sha, expected_count=3):
    require(document.get('strict_fov_passed') is False and document.get('normal_pass') is False and
            document.get('qualified_source_processing') is True, 'Qualification must be explicitly disclosed')
    review = document.get('native_fov_exception_review', {})
    require(review.get('sidecar_sha256') == sidecar_sha and review.get('reviewed_exception_count') == expected_count,
            'Derived manifest binds a different exception inventory')
    for key in ('strict_fov_passed', 'normal_pass', 'raw_points_modified', 'strict_policy_changed'):
        require(review.get(key) is False, 'Invalid exception declaration: ' + key)


def review_point_fate(fusion, reviewed):
    records = {row['frame']: row for row in map(json.loads, (fusion / 'associations.jsonl').read_text(encoding='utf-8').splitlines())}
    allowed_frames = {point['frame'] for point in reviewed}
    for frame, record in records.items():
        require(record['geometry_statistics'].get('reviewed_fov_exception_count') == int(frame in allowed_frames),
                'Per-frame exception disclosure differs')
    result = []
    for point in reviewed:
        frame, index = point['frame'], point['raw_index']
        with np.load(fusion / 'data' / f'{frame:08d}.npz', allow_pickle=False) as data:
            sampled_rows = np.flatnonzero(data['sampled_raw_indices'] == index)
            entry = {'frame': frame, 'raw_index': index, 'retained_in_native_raw': True,
                     'kept_by_unchanged_range_and_self_filter': bool(np.any(data['kept_raw_indices'] == index)),
                     'selected_by_unchanged_voxel_filter': bool(len(sampled_rows)),
                     'sampled_index': int(sampled_rows[0]) if len(sampled_rows) else None,
                     'classified_ground_by_unchanged_plane': bool(data['ground_mask'][sampled_rows[0]]) if len(sampled_rows) else None,
                     'associated_display_ids': [item['display_track_id'] for item in records[frame]['detections']
                                                if index in item['raw_point_indices']]}
        result.append(entry)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('run', 'fusion', 'exceptions', 'out'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--exceptions-sha256', required=True)
    args = parser.parse_args()
    run, fusion, sidecar_path, out = (path.resolve() for path in (args.run, args.fusion, args.exceptions, args.out))
    strict_script = Path(__file__).with_name('audit_production_fusion.py')
    strict_out, strict_log = out.with_name(out.stem + '-strict.json'), out.with_name(out.stem + '-strict.log')
    require(not any(path.exists() for path in (out, strict_out, strict_log)), 'Preserve existing audit evidence')
    out.parent.mkdir(parents=True, exist_ok=True)
    auditor_sha = sha(__file__)
    result = {'status': 'qualified_review_failed', 'normal_pass': False, 'strict_fov_passed': False,
              'qualified_source_integrity_passed': False, 'auditor_sha256': auditor_sha,
              'strict_auditor_sha256': STRICT_AUDITOR_SHA256, 'sidecar_sha256': sha(sidecar_path),
              'raw_points_modified_or_cropped': False, 'strict_policy_modified': False, 'accuracy_claimed': False}
    code = 1
    try:
        require(sha(strict_script) == STRICT_AUDITOR_SHA256, 'Frozen strict auditor changed')
        require(result['sidecar_sha256'] == args.exceptions_sha256 and args.exceptions_sha256 in APPROVED_SIDECARS,
                'Only the SHA-fixed reviewed sidecar is accepted')
        approved = APPROVED_SIDECARS[args.exceptions_sha256]
        result['reviewed_scene'], result['reviewed_exception_count'] = approved['scene'], approved['count']
        sidecar = read(sidecar_path)
        bindings = {name: sha(run / name) for name in SOURCE_KEYS}
        for name, key in SOURCE_KEYS.items():
            require(sidecar.get(key) == bindings[name], 'Exception source binding differs: ' + name)
        result['source_input_sha256'] = bindings
        for name in ('processing-manifest.json', 'fusion-manifest.json'):
            document = read(fusion / name)
            check_exception_declaration(document, result['sidecar_sha256'], approved['count'])
            if name == 'processing-manifest.json':
                result['entrypoint_provenance_bridge'] = verify_entrypoint_provenance(document, Path(__file__).resolve().parents[1])
        inventory = review_raw_inventory(run, sidecar, approved['count'])
        result['raw_inventory'] = inventory
        command = [sys.executable, '-X', 'utf8', str(strict_script), '--run', str(run), '--fusion', str(fusion),
                   '--out', str(strict_out), '--stride', '1']
        result['strict_command'] = command
        with strict_log.open('w', encoding='utf-8') as stream:
            execution = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=1800, check=False)
        result.update(strict_auditor_exit_code=execution.returncode, strict_report=str(strict_out),
                      strict_log_sha256=sha(strict_log))
        require(strict_out.exists(), 'Strict auditor did not produce a report')
        strict = read(strict_out)
        result['strict_report_sha256'] = sha(strict_out)
        check_strict_result(strict, execution.returncode, inventory['reviewed_points'], approved['count'])
        require(strict['stats']['frames'] == inventory['frames'], 'Every native frame must be processed and audited')
        result['exception_point_fate'] = review_point_fate(fusion, inventory['reviewed_points'])
        require(all(sha(run / name) == digest for name, digest in bindings.items()), 'Native source changed during review')
        require(sha(sidecar_path) == args.exceptions_sha256 and sha(__file__) == auditor_sha and
                sha(strict_script) == STRICT_AUDITOR_SHA256, 'Review source changed during execution')
        result.update(status='qualified_source_integrity_passed_with_native_fov_deviations',
                      qualified_source_integrity_passed=True, all_other_strict_checks_passed=True,
                      explicit_entrypoint_provenance_verified=True, strict_failures_preserved=strict['failures'],
                      strict_assertions=strict['assertions'], stats=strict['stats'], by_camera=strict['by_camera'],
                      source_modules_sha256=strict['source_modules_sha256'],
                      limits=['Strict nominal FOV engineering compliance failed for the exact reviewed native return inventory.',
                              'Source preservation and association geometry do not establish detector accuracy.',
                              'The sole cause of the native boundary deviations is not established.',
                              'The legacy sole-entrypoint assertion remains failed; the actual frozen new entrypoint is separately verified.',
                              'Nominal 120-degree sensor configuration is not a claim that every returned hit lies exactly inside it.'])
        code = 0
    except (ValueError, KeyError, TypeError, OSError, subprocess.TimeoutExpired) as error:
        result['error'] = type(error).__name__ + ': ' + str(error)
    result['qualified_review_exit_code'] = code
    out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({key: result.get(key) for key in ('status', 'qualified_review_exit_code', 'strict_auditor_exit_code',
                     'strict_fov_passed', 'normal_pass', 'error', 'stats', 'by_camera')}, indent=2))
    raise SystemExit(code)


if __name__ == '__main__':
    main()
