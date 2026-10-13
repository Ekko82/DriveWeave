"""Synthetic refusal tests; these fixtures are not native capture evidence."""
from pathlib import Path
import copy
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fuse_production_sensors_qualified import ExactNativeReview, load_review, raw_sha


def fixture():
    angle = np.deg2rad(-60.019)
    raw = np.array([[20*np.cos(angle), 20*np.sin(angle), 0., .7], [10., 0., 0., .8]], np.float32)
    row = {'frame': 62182, 'sample': 284, 'sensors': {'lidar': {'raw_sha256': raw_sha(raw)}}}
    entry = {'frame': row['frame'], 'sample': row['sample'], 'raw_index': 0,
             'raw_callback_sha256': raw_sha(raw), 'raw_point_float32_le_hex': raw[0].astype('<f4').tobytes().hex()}
    return raw, row, entry


class QualifiedNativeTests(unittest.TestCase):
    def test_exact_review_preserves_bytes_and_strict_failure(self):
        raw, row, entry = fixture()
        before = raw.tobytes()
        guard = ExactNativeReview([entry])
        result = guard.validate(raw, row)
        self.assertFalse(result['strict_fov_passed'])
        self.assertEqual(result['engineering_tolerance_exceeded_points'], 1)
        self.assertEqual(result['reviewed_fov_exception_count'], 1)
        self.assertEqual(raw.tobytes(), before)
        guard.require_complete()

    def test_unlisted_additional_violation_refused(self):
        raw, row, entry = fixture()
        raw[1] = raw[0]
        row['sensors']['lidar']['raw_sha256'] = raw_sha(raw)
        entry['raw_callback_sha256'] = raw_sha(raw)
        with self.assertRaisesRegex(ValueError, 'failure set'):
            ExactNativeReview([entry]).validate(raw, row)

    def test_different_callback_refused(self):
        raw, row, entry = fixture()
        entry['raw_callback_sha256'] = '0'*64
        with self.assertRaisesRegex(ValueError, 'callback SHA'):
            ExactNativeReview([entry]).validate(raw, row)

    def test_different_xyzi_bytes_refused(self):
        raw, row, entry = fixture()
        entry['raw_point_float32_le_hex'] = '0'*32
        with self.assertRaisesRegex(ValueError, 'point bytes'):
            ExactNativeReview([entry]).validate(raw, row)

    def test_different_sample_refused(self):
        raw, row, entry = fixture()
        entry['sample'] += 1
        with self.assertRaisesRegex(ValueError, 'sample'):
            ExactNativeReview([entry]).validate(raw, row)

    def test_missing_expected_violation_refused(self):
        raw, row, entry = fixture()
        raw[0, 1] = 0
        row['sensors']['lidar']['raw_sha256'] = raw_sha(raw)
        with self.assertRaisesRegex(ValueError, 'failure set'):
            ExactNativeReview([entry]).validate(raw, row)

    def test_nonfinite_never_excepted(self):
        raw, row, entry = fixture()
        raw[0, 2] = np.nan
        with self.assertRaisesRegex(ValueError, 'Nonfinite'):
            ExactNativeReview([entry]).validate(raw, row)

    def test_negative_x_never_excepted(self):
        raw, row, entry = fixture()
        raw[0, 0] = -.001
        with self.assertRaisesRegex(ValueError, 'Negative x'):
            ExactNativeReview([entry]).validate(raw, row)

    def test_duplicate_review_refused(self):
        _, _, entry = fixture()
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            ExactNativeReview([entry, copy.deepcopy(entry)])

    def test_unencountered_review_refused(self):
        _, _, entry = fixture()
        with self.assertRaisesRegex(ValueError, 'every reviewed'):
            ExactNativeReview([entry]).require_complete()

    def test_unapproved_sidecar_refused_before_source_access(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'fake.json'
            path.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'Unapproved'):
                load_review(Path(directory), path, '0'*64)


if __name__ == '__main__':
    unittest.main()
