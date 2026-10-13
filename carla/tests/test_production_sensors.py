"""Synthetic unit fixtures for production processing, never native capture evidence."""
from pathlib import Path
import hashlib
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fuse_production_sensors import camera_only, image_and_intrinsic, preprocess_points, validate_native_fov


def fixture():
    xyz = np.array([[x, y, -1.67] for x in np.linspace(8, 30, 24)
                    for y in np.linspace(-5, 5, 24)], dtype=np.float32)
    raw = np.column_stack((xyz, np.ones(len(xyz), dtype=np.float32)))
    mount = np.eye(4)
    mount[:3, 3] = [.45, 0, 1.67]
    row = {'frame': 10, 'sensors': {'lidar': {'transform': {'world_from_local': mount.tolist()}}},
           'vehicles': [{'ego': False, 'unread_other_vehicle_gt': 'deliberately invalid unused object'},
                        {'ego': True, 'transform': {'world_from_local': np.eye(4).tolist()},
                         'bounding_box': {'center_local_m': [0, 0, .8], 'extent_m': [2.4, 1., .8],
                                          'rotation_rpy_deg': [0, 0, 0]}}]}
    return raw, row


class ProductionSensorTests(unittest.TestCase):
    @staticmethod
    def ray(distance, azimuth=0., elevation=0.):
        azimuth, elevation = np.deg2rad([azimuth, elevation])
        return np.array([[distance*np.cos(elevation)*np.cos(azimuth),
                          distance*np.cos(elevation)*np.sin(azimuth),
                          distance*np.sin(elevation), 1.]], np.float32)

    def test_near_boundary_requires_angular_limit_despite_submillimeter_residual(self):
        for azimuth, elevation in [(60.03, 0), (-60.03, 0), (0, 12.93), (0, -12.53)]:
            with self.subTest(azimuth=azimuth, elevation=elevation):
                with self.assertRaisesRegex(ValueError, 'engineering limits'):
                    validate_native_fov(self.ray(1., azimuth, elevation))

    def test_far_boundary_requires_metric_limit_despite_small_angle(self):
        for azimuth, elevation in [(60.004, 0), (-60.004, 0), (0, 12.904), (0, -12.504)]:
            with self.subTest(azimuth=azimuth, elevation=elevation):
                with self.assertRaisesRegex(ValueError, 'engineering limits'):
                    validate_native_fov(self.ray(100., azimuth, elevation))

    def test_small_both_limits_accepted_and_strict_excess_recorded_without_raw_edit(self):
        raw = np.vstack([self.ray(10., az, el) for az, el in
                         [(60.005, 0), (-60.005, 0), (0, 12.905), (0, -12.505)]])
        original = raw.copy()
        metrics = validate_native_fov(raw)
        np.testing.assert_array_equal(raw, original)
        self.assertEqual(metrics['strict_horizontal_outside_points'], 2)
        self.assertEqual(metrics['strict_vertical_outside_points'], 2)
        self.assertEqual(metrics['engineering_tolerance_exceeded_points'], 0)
        self.assertGreater(metrics['max_horizontal_angle_excess_deg'], .004)
        self.assertLess(metrics['max_vertical_boundary_excess_m'], .001)

    def test_rear_ray_rejected_even_when_metric_error_is_under_budget(self):
        with self.assertRaisesRegex(ValueError, 'x>=0'):
            validate_native_fov(self.ray(.001, 180.))

    def test_preprocessing_preserves_exact_native_rows_and_ground(self):
        raw, row = fixture()
        kept, sampled, xyz, ground, plane, metrics = preprocess_points(raw, row)
        np.testing.assert_array_equal(xyz, raw[sampled, :3])
        self.assertTrue(set(sampled).issubset(set(kept)))
        np.testing.assert_array_equal(ground, np.abs(xyz.astype(float)@plane[:3]+plane[3]) < .18)
        self.assertTrue(ground.all())
        self.assertEqual(metrics['raw_points'], len(raw))

    def test_rear_ray_rejected_before_range_filtering(self):
        raw, row = fixture()
        raw = np.vstack((raw, np.array([[-150, 0, -1.67, 1]], np.float32)))
        with self.assertRaisesRegex(ValueError, 'outside forward'):
            preprocess_points(raw, row)

    def test_side_ray_rejected_instead_of_crop(self):
        raw, row = fixture()
        raw = np.vstack((raw, np.array([[0, 20, -1.67, 1]], np.float32)))
        with self.assertRaisesRegex(ValueError, 'outside forward'):
            preprocess_points(raw, row)

    def test_nonfinite_native_points_rejected(self):
        raw, row = fixture()
        raw[0, 1] = np.nan
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            preprocess_points(raw, row)

    def test_ego_hit_removed_using_only_ego_geometry(self):
        raw, row = fixture()
        raw = np.vstack((raw, np.array([[.8, 0, -.1, 1]], np.float32)))
        kept, sampled, *_ = preprocess_points(raw, row)
        self.assertNotIn(len(raw)-1, kept)
        self.assertNotIn(len(raw)-1, sampled)

    def test_camera_only_has_no_depth_or_point_membership(self):
        source = {'bbox_xyxy': [10, 20, 100, 120], 'class_id': 2, 'score': .91, 'class_name': 'car'}
        for camera in ('rear', 'side_left', 'side_right'):
            result = camera_only(source, camera)
            self.assertEqual(result['camera'], camera)
            self.assertEqual(result['fusion_status'], 'camera_only')
            for field in ('range_m', 'observed_center_lidar_m', 'world_observed_center_m'):
                self.assertIsNone(result[field])
            self.assertEqual(result['raw_point_indices'], [])
            self.assertEqual(result['sampled_point_indices'], [])
        self.assertNotIn('camera', source)

    def test_each_camera_uses_its_own_native_fov_and_hash(self):
        rgb = np.zeros((720, 1280, 3), np.uint8)
        rgb[:, :, 0] = 90
        bgra = np.concatenate((rgb[:, :, ::-1], np.full((720, 1280, 1), 255, np.uint8)), axis=2)
        raw_sha = hashlib.sha256(bgra.tobytes()).hexdigest()
        manifest = {'sensors': {}, 'camera_intrinsics': {}}
        sensors = {}
        for name, fov in [('rgb', 90), ('rear', 100), ('side_left', 110), ('side_right', 110)]:
            focal = 1280/(2*np.tan(np.deg2rad(fov)/2))
            K = [[focal, 0, 640], [0, focal, 360], [0, 0, 1]]
            sensors[name] = dict(width=1280, height=720, fov=fov, raw_sha256=raw_sha, intrinsic=K)
            manifest['sensors'][name] = {'intrinsic': K}
            manifest['camera_intrinsics'][name] = K
        manifest['camera_intrinsic'] = sensors['rgb']['intrinsic']
        with tempfile.TemporaryDirectory(prefix='production-fixture-') as temporary:
            path = Path(temporary)/'synthetic-unit-fixture.png'
            Image.fromarray(rgb).save(path)
            focals = []
            for name in sensors:
                _, K = image_and_intrinsic(path, sensors[name], manifest, name)
                focals.append(K[0, 0])
            self.assertGreater(focals[0], focals[1])
            self.assertGreater(focals[1], focals[2])
            sensors['rear']['raw_sha256'] = '0'*64
            with self.assertRaisesRegex(ValueError, 'BGRA'):
                image_and_intrinsic(path, sensors['rear'], manifest, 'rear')


if __name__ == '__main__':
    unittest.main()
