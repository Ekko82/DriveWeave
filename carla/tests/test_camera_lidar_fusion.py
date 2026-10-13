"""Adversarial checks for measured-point association and display tracking.

Synthetic fixtures are test inputs only; no fixture is recorded as a native run.
"""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fuse_camera_lidar import associate, AssociationTracker, project


K = np.array([[400., 0, 320], [0, 400., 240], [0, 0, 1]])


def box(score=.9):
    return dict(class_id=2, class_name='car', score=score,
                bbox_xyxy=[294., 224., 346., 256.])


def surface(depth=20., ny=9, nz=7):
    return np.array([[depth, y*depth/20, z*depth/20]
                     for y in np.linspace(-1., 1., ny)
                     for z in np.linspace(-.5, .5, nz)])


def run(points, boxes=None, ground=None):
    projected, uv, depths = project(points, np.eye(4), np.eye(4), K, 640, 480)
    return associate([box()] if boxes is None else boxes, points,
                     np.arange(len(points))*3+7,
                     np.zeros(len(points), bool) if ground is None else ground,
                     projected, uv, depths)


class FusionAssociationTests(unittest.TestCase):
    def test_range_is_observed_surface_median_and_indices_remain_native(self):
        points = surface()
        result, = run(points)
        self.assertEqual(result['fusion_status'], 'associated')
        ids = np.array(result['sampled_point_indices'])
        np.testing.assert_array_equal(result['raw_point_indices'], ids*3+7)
        self.assertAlmostEqual(result['range_m'], float(np.median(np.linalg.norm(points[ids], axis=1))))
        np.testing.assert_allclose(result['observed_center_lidar_m'], np.median(points[ids], axis=0))

    def test_ground_is_never_associated(self):
        result, = run(surface(), ground=np.ones(len(surface()), bool))
        self.assertEqual(result['fusion_status'], 'camera_only')
        self.assertIsNone(result['range_m'])

    def test_behind_camera_points_cannot_provide_range(self):
        result, = run(surface(-20))
        self.assertEqual(result['fusion_status'], 'camera_only')
        self.assertEqual(result['raw_point_indices'], [])

    def test_sparse_returns_are_explicitly_unmatched(self):
        result, = run(surface()[[0, 8, 16, 24, 34]])
        self.assertEqual(result['fusion_status'], 'camera_only')
        self.assertIsNone(result['observed_center_lidar_m'])

    def test_narrow_pole_in_box_is_not_a_car_surface(self):
        points = np.array([[20., 0., z] for z in np.linspace(-.5, .5, 21)])
        result, = run(points)
        self.assertEqual(result['fusion_status'], 'camera_only')

    def test_two_well_supported_depths_are_ambiguous(self):
        result, = run(np.concatenate((surface(20), surface(40))))
        self.assertEqual(result['fusion_status'], 'ambiguous_depth')
        self.assertIsNone(result['range_m'])
        self.assertEqual(result['raw_point_indices'], [])

    def test_dense_background_does_not_hide_supported_near_component(self):
        result, = run(np.concatenate((surface(20), surface(40, 13, 9))))
        self.assertEqual(result['fusion_status'], 'ambiguous_depth')

    def test_overlapping_detections_do_not_duplicate_measured_points(self):
        first, second = run(surface(), [box(.95), box(.85)])
        self.assertEqual(first['fusion_status'], 'associated')
        self.assertEqual(second['fusion_status'], 'camera_only')
        self.assertFalse(set(first['raw_point_indices']) & set(second['raw_point_indices']))

    def test_projection_uses_current_extrinsic_and_rejects_out_of_view(self):
        camera = np.eye(4)
        camera[1, 3] = 3
        points = np.array([[20., 3., 0.], [20., 50., 0.], [-20., 3., 0.]])
        indices, pixels, depth = project(points, np.eye(4), camera, K, 640, 480)
        np.testing.assert_array_equal(indices, [0])
        np.testing.assert_allclose(pixels, [[320., 240.]])
        np.testing.assert_allclose(depth, [20.])


class DisplayTrackingTests(unittest.TestCase):
    def item(self, center=None, class_id=2):
        return dict(box(), observed_center_lidar_m=center,
                    fusion_status='camera_only') | {'class_id': class_id}

    def test_missing_lidar_does_not_inherit_previous_range(self):
        tracker = AssociationTracker()
        first = self.item([20., 0., 0.])
        tracker.update([first], 1., np.eye(4))
        second = self.item()
        tracker.update([second], 1.05, np.eye(4))
        self.assertEqual(second['display_track_id'], first['display_track_id'])
        self.assertIsNone(second['observed_center_lidar_m'])
        self.assertIsNone(second['world_observed_center_m'])

    def test_empty_frame_does_not_hallucinate_detections(self):
        tracker = AssociationTracker()
        tracker.update([self.item([20., 0., 0.])], 1., np.eye(4))
        detections = []
        tracker.update(detections, 1.05, np.eye(4))
        self.assertEqual(detections, [])

    def test_expired_track_gets_new_id(self):
        tracker = AssociationTracker()
        first, second = self.item(), self.item()
        tracker.update([first], 1., np.eye(4))
        tracker.update([second], 1.6, np.eye(4))
        self.assertNotEqual(first['display_track_id'], second['display_track_id'])

    def test_nonmonotonic_or_nonfinite_time_is_rejected(self):
        for invalid in [1., .95, float('nan'), float('inf')]:
            with self.subTest(invalid=invalid):
                tracker = AssociationTracker()
                tracker.update([], 1., np.eye(4))
                with self.assertRaises(ValueError):
                    tracker.update([], invalid, np.eye(4))

    def test_ego_motion_compensates_display_association(self):
        tracker = AssociationTracker()
        first, second = self.item([20., 0., 0.]), self.item([10., 0., 0.])
        tracker.update([first], 1., np.eye(4))
        transform = np.eye(4)
        transform[0, 3] = 10.
        tracker.update([second], 1.05, transform)
        self.assertEqual(first['display_track_id'], second['display_track_id'])
        np.testing.assert_allclose(first['world_observed_center_m'], second['world_observed_center_m'])


if __name__ == '__main__':
    unittest.main()
