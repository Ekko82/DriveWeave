"""Synthetic audit rejection fixtures, never native sensor or accuracy evidence."""
from pathlib import Path
import copy
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_production_fusion import (Checks, CAMERAS, OFFSETS, check_detections,
                                     check_points, independent_projection,
                                     intrinsic, matrix_from_pose, check_native_sector, check_fov_policy)


def point_fixture():
    ground = np.asarray([[x, y, -1.67] for x in (8., 10., 12., 14., 16., 18., 20., 22.)
                         for y in (-2., -1.5, -1., -.5, 0., .5, 1., 1.5)], np.float32)
    target = np.asarray([[10., y, z] for y in (-.5, -.3, -.1, .1, .3, .5)
                         for z in (-.1, .3)], np.float32)
    xyz = np.vstack((ground, target))
    raw = np.column_stack((xyz, np.ones(len(xyz), np.float32)))
    mount = np.eye(4)
    mount[:3, 3] = [.45, 0, 1.67]
    row = {'frame': 10, 'timestamp': .5, 'sensors': {'lidar': {
        'transform': {'world_from_local': mount.tolist()}, 'points': len(raw),
        'channels': 128, 'counts_per_channel': [len(raw)] + [0] * 127}},
        'vehicles': [{'ego': False, 'unread_gt': 'invalid on purpose'},
                     {'ego': True, 'transform': {'world_from_local': np.eye(4).tolist()},
                      'bounding_box': {'rotation_rpy_deg': [0, 0, 0], 'center_local_m': [0, 0, .8],
                                       'extent_m': [2.4, 1, .8]}}]}
    data = {'kept_raw_indices': np.arange(len(raw)), 'sampled_raw_indices': np.arange(len(raw)),
            'ground_mask': np.r_[np.ones(len(ground), bool), np.zeros(len(target), bool)],
            'ground_plane_abcd': np.array([0, 0, 1, 1.67])}
    attributes = {'channels': 128, 'points_per_second': 1536000,
                  'lower_fov': -12.5, 'upper_fov': 12.9}
    return raw, row, {'fps': 20}, data, attributes


def detection_fixture():
    points = np.asarray([[10, y, z] for y in (-.5, -.3, -.1, .1, .3, .5)
                         for z in (-.1, .3)], np.float32)
    raw = {'bbox_xyxy': [580, 320, 700, 390], 'class_id': 2, 'class_name': 'car', 'score': .9}
    item = dict(raw, camera='front', display_track_id=1, track_id_local=1,
                tracking_scope='same_camera_only', fusion_status='associated',
                sampled_point_indices=list(range(len(points))), raw_point_indices=list(range(len(points))),
                range_m=float(np.median(np.linalg.norm(points, axis=1))),
                observed_center_lidar_m=np.median(points, axis=0).tolist(),
                world_observed_center_m=np.median(points, axis=0).tolist(),
                observed_bounds_min_m=points.min(axis=0).tolist(),
                observed_bounds_max_m=points.max(axis=0).tolist(), association_support_points=len(points))
    record = {'frame': 10, 'raw_detections': {camera: [] for camera in CAMERAS}, 'detections': [item]}
    record['raw_detections']['front'] = [copy.deepcopy(raw)]
    summary = {'display_confidence': {'front': .75, 'rear': .5, 'side_left': .5, 'side_right': .5},
               'display_track_id_base': 0, 'inference_confidence': .3}
    projection = independent_projection(points, np.eye(4), np.eye(4), intrinsic(1280, 720, 90), 1280, 720)
    return record, summary, points, projection


class ProductionAuditFixtures(unittest.TestCase):
    def test_opposed_camera_projection_and_distinct_native_intrinsics(self):
        points = np.array([[10, 0, 0], [-10, 0, 0], [10, 10, 0]], float)
        front = independent_projection(points, np.eye(4), np.eye(4), intrinsic(1280, 720, 90), 1280, 720)
        rear_pose = matrix_from_pose([0, 0, 0], [0, 180, 0])
        rear = independent_projection(points, np.eye(4), rear_pose, intrinsic(1280, 720, 100), 1280, 720)
        self.assertEqual(front[0].tolist(), [True, False, False])
        self.assertEqual(rear[0].tolist(), [False, True, False])
        np.testing.assert_allclose(rear[1][1], [640, 360])
        self.assertGreater(intrinsic(1280, 720, 90)[0, 0], intrinsic(1280, 720, 100)[0, 0])
        self.assertGreater(intrinsic(1280, 720, 100)[0, 0], intrinsic(1280, 720, 110)[0, 0])

    def test_valid_native_row_voxel_and_ground_fixture(self):
        arguments = point_fixture()
        checks = Checks()
        xyz, ground, indices, _ = check_points(*arguments, checks)
        self.assertEqual(checks.counts['failed_assertions'], 0, checks.failures)
        np.testing.assert_array_equal(xyz, arguments[0][indices, :3])
        self.assertEqual(int(ground.sum()), 64)

    def test_rear_raw_ray_is_rejected_even_if_derived_data_omits_it(self):
        raw, row, manifest, data, attributes = point_fixture()
        raw = np.vstack((raw, np.array([[-150., 0, 0, 1]], np.float32)))
        row['sensors']['lidar']['points'] += 1
        row['sensors']['lidar']['counts_per_channel'][0] += 1
        checks = Checks()
        check_points(raw, row, manifest, data, attributes, checks)
        self.assertTrue(any('negative-x' in failure for failure in checks.failures))

    def test_changed_sampled_native_row_is_rejected(self):
        raw, row, manifest, data, attributes = point_fixture()
        data['sampled_raw_indices'] = data['sampled_raw_indices'][::-1]
        data['ground_mask'] = data['ground_mask'][::-1]
        checks = Checks()
        check_points(raw, row, manifest, data, attributes, checks)
        self.assertTrue(any('voxel representatives' in failure for failure in checks.failures))

    def test_valid_front_measured_association_fixture(self):
        record, summary, points, projection = detection_fixture()
        checks = Checks()
        counters = check_detections(record, summary, points, np.zeros(len(points), bool),
                                    np.arange(len(points)), projection, np.eye(4), checks)
        self.assertEqual(checks.counts['failed_assertions'], 0, checks.failures)
        self.assertEqual(counters['front']['associated_native_points'], len(points))

    def test_non_front_lidar_points_and_distance_are_rejected(self):
        record, summary, points, projection = detection_fixture()
        item = record['detections'][0]
        item.update(camera='rear', display_track_id=10001)
        record['raw_detections']['rear'] = record['raw_detections']['front']
        record['raw_detections']['front'] = []
        checks = Checks()
        check_detections(record, summary, points, np.zeros(len(points), bool),
                         np.arange(len(points)), projection, np.eye(4), checks)
        self.assertTrue(any('Non-front camera claims' in failure for failure in checks.failures))

    def test_camera_only_fabricated_distance_is_rejected(self):
        record, summary, points, projection = detection_fixture()
        source = record['detections'][0]
        source.update(camera='side_left', display_track_id=20001, fusion_status='camera_only',
                      sampled_point_indices=[], raw_point_indices=[], range_m=12.)
        record['raw_detections']['side_left'] = record['raw_detections']['front']
        record['raw_detections']['front'] = []
        checks = Checks()
        check_detections(record, summary, points, np.zeros(len(points), bool),
                         np.arange(len(points)), projection, np.eye(4), checks)
        self.assertTrue(any('fabricated geometry' in failure for failure in checks.failures))

    def test_foreign_camera_id_namespace_is_rejected(self):
        record, summary, points, projection = detection_fixture()
        record['detections'][0]['display_track_id'] = OFFSETS['rear'] + 1
        checks = Checks()
        check_detections(record, summary, points, np.zeros(len(points), bool),
                         np.arange(len(points)), projection, np.eye(4), checks)
        self.assertTrue(any('ID namespace' in failure for failure in checks.failures))

    def test_edited_display_detection_cannot_claim_original_rgb_prediction(self):
        record, summary, points, projection = detection_fixture()
        record['detections'][0]['score'] = .99
        checks = Checks()
        check_detections(record, summary, points, np.zeros(len(points), bool),
                         np.arange(len(points)), projection, np.eye(4), checks)
        self.assertTrue(any('thresholded RGB results' in failure for failure in checks.failures))

    def test_small_boundary_residual_passes_both_fixed_engineering_limits(self):
        angle = np.radians(60.003)
        points = np.array([[40 * np.cos(angle), 40 * np.sin(angle), 0.]])
        checks = Checks()
        metrics = check_native_sector(points, -12.5, 12.9, checks, 'fixture')
        self.assertEqual(checks.counts['failed_assertions'], 0, checks.failures)
        self.assertEqual(metrics['strict_horizontal_outside_points'], 1)
        self.assertGreater(metrics['max_horizontal_boundary_excess_m'], 0)

    def test_near_large_angle_cannot_pass_only_because_metric_error_is_small(self):
        angle = np.radians(60.03)
        points = np.array([[np.cos(angle), np.sin(angle), 0.]])
        checks = Checks()
        metrics = check_native_sector(points, -12.5, 12.9, checks, 'fixture')
        self.assertLess(metrics['max_horizontal_boundary_excess_m'], .005)
        self.assertGreater(metrics['max_horizontal_angle_excess_deg'], .02)
        self.assertTrue(any('horizontal FOV' in failure for failure in checks.failures))

    def test_far_small_angle_cannot_pass_when_metric_error_exceeds_budget(self):
        angle = np.radians(60.004)
        points = np.array([[100 * np.cos(angle), 100 * np.sin(angle), 0.]])
        checks = Checks()
        metrics = check_native_sector(points, -12.5, 12.9, checks, 'fixture')
        self.assertGreater(metrics['max_horizontal_boundary_excess_m'], .005)
        self.assertLess(metrics['max_horizontal_angle_excess_deg'], .02)
        self.assertTrue(any('horizontal FOV' in failure for failure in checks.failures))

    def test_vertical_boundary_obeys_both_limits(self):
        angle = np.radians(12.904)
        points = np.array([[100 * np.cos(angle), 0., 100 * np.sin(angle)]])
        checks = Checks()
        metrics = check_native_sector(points, -12.5, 12.9, checks, 'fixture')
        self.assertGreater(metrics['max_vertical_boundary_excess_m'], .005)
        self.assertLess(metrics['max_vertical_angle_excess_deg'], .02)
        self.assertTrue(any('vertical FOV' in failure for failure in checks.failures))

    def test_any_negative_x_point_is_rejected_even_at_submillimetre_range(self):
        checks = Checks()
        check_native_sector(np.array([[-.00001, 0., 0.]]), -12.5, 12.9, checks, 'fixture')
        self.assertTrue(any('negative-x' in failure for failure in checks.failures))

    def test_manifest_policy_cannot_silently_increase_a_fixed_budget(self):
        attributes = point_fixture()[-1]
        policy = {'kind': 'engineering_numeric_tolerance', 'native_horizontal_fov_deg': 120.,
                  'native_upper_fov_deg': 12.9, 'native_lower_fov_deg': -12.5,
                  'maximum_boundary_perpendicular_excess_m': .005, 'maximum_angle_excess_deg': .02,
                  'minimum_x_m': 0., 'horizontal_ray_spacing_deg': .2, 'vertical_channel_spacing_deg': .2,
                  'angular_budget_fraction_of_ray_spacing': .1, 'all_limits_required_simultaneously': True,
                  'post_capture_fov_crop': False}
        checks = Checks()
        check_fov_policy(policy, attributes, 20, checks)
        self.assertEqual(checks.counts['failed_assertions'], 0, checks.failures)
        policy['maximum_boundary_perpendicular_excess_m'] = .00501
        checks = Checks()
        check_fov_policy(policy, attributes, 20, checks)
        self.assertTrue(any('maximum_boundary_perpendicular_excess_m' in failure for failure in checks.failures))

    def test_manifest_policy_cannot_allow_cropping_or_either_limit(self):
        checks = Checks()
        check_fov_policy({'all_limits_required_simultaneously': False, 'post_capture_fov_crop': True},
                         point_fixture()[-1], 20, checks)
        self.assertTrue(any('all limits without cropping' in failure for failure in checks.failures))


if __name__ == '__main__':
    unittest.main()
