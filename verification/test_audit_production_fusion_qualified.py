"""Synthetic rejection fixtures for qualification; never actual capture evidence."""
from pathlib import Path
import copy
import hashlib
import json
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_production_fusion_qualified import (STRICT_AUDITOR_SHA256, FROZEN_SOURCE_SHA256, QUALIFIED_ENTRYPOINT,
                                               ENTRYPOINT_ASSERTION, verify_entrypoint_provenance, native_deviations, exception_inventory,
                                               check_exception_point, review_raw_inventory,
                                               check_strict_result, check_exception_declaration)


def point_fixture():
    angle = np.radians(60.025)
    raw = np.array([[15 * np.cos(angle), 15 * np.sin(angle), 0., .75]], np.float32)
    _, metrics = native_deviations(raw)
    digest = hashlib.sha256(raw.tobytes()).hexdigest()
    item = {'frame': 1, 'sample': 0, 'raw_index': 0, 'raw_callback_sha256': digest,
            'raw_point_float32_le_hex': raw[0].tobytes().hex(),
            **{name: float(values[0]) for name, values in metrics.items()}}
    return raw, metrics, item


def sidecar_fixture():
    raw, metrics, item = point_fixture()
    items = [dict(item, frame=frame, sample=frame - 1) for frame in (1, 2, 3)]
    return raw, {'schema_version': 1, 'strict_policy': {'boundary_m': .005, 'angle_deg': .02, 'minimum_x_m': 0},
                 'exceptions': items}


def write_capture(root, raw):
    (root / 'lidar').mkdir()
    rows = []
    for frame in (1, 2, 3):
        np.save(root / 'lidar' / f'{frame:08d}.npy', raw)
        rows.append({'frame': frame, 'sample': frame - 1,
                     'sensors': {'lidar': {'raw_sha256': hashlib.sha256(raw.tobytes()).hexdigest()}}})
    (root / 'frames.jsonl').write_text('\n'.join(json.dumps(row) for row in rows))
    (root / 'manifest.json').write_text(json.dumps({'frames_captured': 3}))


class QualifiedAuditFixtures(unittest.TestCase):
    def test_exact_three_preserved_native_points_inventory(self):
        raw, sidecar = sidecar_fixture()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_capture(root, raw)
            result = review_raw_inventory(root, sidecar)
        self.assertEqual(len(result['reviewed_points']), 3)
        self.assertEqual(result['native_raw_points'], 3)

    def test_raw_intensity_edit_cannot_hide_behind_unchanged_xyz(self):
        raw, metrics, item = point_fixture()
        raw[0, 3] = .5
        with self.assertRaisesRegex(ValueError, 'XYZI bytes'):
            check_exception_point(item, 1, 0, 0, raw, item['raw_callback_sha256'], metrics)

    def test_negative_x_point_never_eligible_for_exception(self):
        with self.assertRaisesRegex(ValueError, 'Negative-x'):
            native_deviations(np.array([[-.00001, 0, 0, 1]], np.float32))

    def test_duplicate_exception_or_policy_change_rejected(self):
        _, sidecar = sidecar_fixture()
        sidecar['exceptions'][1] = copy.deepcopy(sidecar['exceptions'][0])
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            exception_inventory(sidecar)
        sidecar['strict_policy']['boundary_m'] = .0066
        with self.assertRaisesRegex(ValueError, 'changes the strict policy'):
            exception_inventory(sidecar)

    def test_independent_one_point_inventory_cannot_accept_three_point_permission(self):
        _, sidecar = sidecar_fixture()
        with self.assertRaisesRegex(ValueError, 'source-specific'):
            exception_inventory(sidecar, expected_count=1)
        sidecar['exceptions'] = sidecar['exceptions'][:1]
        self.assertEqual(len(exception_inventory(sidecar, expected_count=1)), 1)

    def test_an_extra_violation_cannot_be_silently_qualified(self):
        raw, sidecar = sidecar_fixture()
        raw = np.vstack((raw, raw))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_capture(root, raw)
            for item in sidecar['exceptions']:
                item['raw_callback_sha256'] = hashlib.sha256(raw.tobytes()).hexdigest()
            with self.assertRaisesRegex(ValueError, 'Unreviewed'):
                review_raw_inventory(root, sidecar)

    def test_disappearing_exception_cannot_be_silently_qualified(self):
        _, sidecar = sidecar_fixture()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_capture(root, np.array([[10., 0, 0, 1]], np.float32))
            with self.assertRaisesRegex(ValueError, 'set differs'):
                review_raw_inventory(root, sidecar)

    def test_strict_failures_preserved_and_other_failures_rejected(self):
        reviewed = [{'frame': frame} for frame in (1, 2, 3)]
        failures = [ENTRYPOINT_ASSERTION, *[f'LiDAR {frame}: native horizontal FOV engineering budget exceeded' for frame in (1, 2, 3)]]
        report = {'status': 'failed', 'auditor_sha256': STRICT_AUDITOR_SHA256, 'failures': failures,
                  'assertions': {'assertions': 104, 'passed_assertions': 100, 'failed_assertions': 4},
                  'failures_truncated': False, 'all_output_frames_geometry_audited': True, 'stride': 1,
                  'accuracy_claimed': False}
        check_strict_result(report, 1, reviewed)
        self.assertEqual(report['assertions']['failed_assertions'], 4)
        with self.assertRaisesRegex(ValueError, 'real failure'):
            check_strict_result(report, 0, reviewed)
        report['failures'].append('Native PNG hash differs')
        with self.assertRaisesRegex(ValueError, 'unexpected failures'):
            check_strict_result(report, 1, reviewed)

    def test_frozen_new_entrypoint_and_all_original_dependencies_verified(self):
        document = {'script_sha256': FROZEN_SOURCE_SHA256[QUALIFIED_ENTRYPOINT],
                    'source_modules_sha256': copy.deepcopy(FROZEN_SOURCE_SHA256)}
        with patch('audit_production_fusion_qualified.sha', side_effect=lambda p: FROZEN_SOURCE_SHA256[p.as_posix()]) as checker:
            proof = verify_entrypoint_provenance(document, Path('.'))
        self.assertTrue(proof['explicit_entrypoint_provenance_verified'])
        self.assertEqual(checker.call_count, len(FROZEN_SOURCE_SHA256))

    def test_wrong_entrypoint_script_mapping_or_bytes_rejected(self):
        source = {'script_sha256': FROZEN_SOURCE_SHA256[QUALIFIED_ENTRYPOINT],
                  'source_modules_sha256': copy.deepcopy(FROZEN_SOURCE_SHA256)}
        document = copy.deepcopy(source)
        document['script_sha256'] = 'wrong'
        with self.assertRaisesRegex(ValueError, 'script SHA'):
            verify_entrypoint_provenance(document, Path('.'))
        document = copy.deepcopy(source)
        document['source_modules_sha256'][QUALIFIED_ENTRYPOINT] = 'wrong'
        with self.assertRaisesRegex(ValueError, 'script SHA'):
            verify_entrypoint_provenance(document, Path('.'))
        with patch('audit_production_fusion_qualified.sha', return_value='wrong'):
            with self.assertRaisesRegex(ValueError, 'source file bytes'):
                verify_entrypoint_provenance(source, Path('.'))

    def test_original_dependency_mapping_cannot_change_under_new_entrypoint(self):
        document = {'script_sha256': FROZEN_SOURCE_SHA256[QUALIFIED_ENTRYPOINT],
                    'source_modules_sha256': copy.deepcopy(FROZEN_SOURCE_SHA256)}
        document['source_modules_sha256']['carla/fuse_production_sensors.py'] = 'wrong'
        with patch('audit_production_fusion_qualified.sha', side_effect=lambda p: FROZEN_SOURCE_SHA256[p.as_posix()]):
            with self.assertRaisesRegex(ValueError, 'source mapping'):
                verify_entrypoint_provenance(document, Path('.'))

    def test_qualified_manifest_cannot_advertise_a_normal_or_strict_pass(self):
        review = {'sidecar_sha256': 'test', 'reviewed_exception_count': 3,
                  'strict_fov_passed': False, 'normal_pass': False, 'raw_points_modified': False,
                  'strict_policy_changed': False}
        manifest = {'strict_fov_passed': False, 'normal_pass': False, 'qualified_source_processing': True,
                    'native_fov_exception_review': review}
        check_exception_declaration(manifest, 'test')
        manifest['normal_pass'] = True
        with self.assertRaisesRegex(ValueError, 'explicitly disclosed'):
            check_exception_declaration(manifest, 'test')


if __name__ == '__main__':
    unittest.main()
