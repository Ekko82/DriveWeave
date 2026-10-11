"""Persistence and native float32 boundary regressions; no CARLA runtime."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from process_carla_sensors import fit_ground
import tempfile
import unittest
import numpy as np


class GroundNumericTests(unittest.TestCase):
    def assert_persisted_mask(self, xyz, seed):
        normal,offset,ground=fit_ground(xyz,seed=seed)
        self.assertEqual(normal.dtype,np.dtype('float64'))
        plane=np.r_[normal,offset]
        with tempfile.TemporaryDirectory() as folder:
            saved=Path(folder)/'actual-derived-result.npz'
            np.savez_compressed(saved,plane=plane,ground=ground)
            with np.load(saved) as result:
                residual=abs(xyz.astype(float)@result['plane'][:3]+result['plane'][3])
                np.testing.assert_array_equal(residual<.18,result['ground'])

    def test_float32_native_points_roundtrip_mask_uses_unchanged_boundary(self):
        rng=np.random.default_rng(21)
        xy=rng.uniform([-14,-7],[34,7],(1200,2))
        z=-2.+xy[:,0]*.003+xy[:,1]*.002+rng.normal(0,.015,len(xy))
        xyz=np.column_stack([xy,z]).astype(np.float32)
        self.assert_persisted_mask(xyz,seed=77)

    def test_actual_failed_native_frame_reconstructs_exactly_after_refit(self):
        base=Path(__file__).resolve().parents[1]
        source=base/'carla/captures-v2/20261012-052923-964542/lidar/01200116.npy'
        old=base/'carla/processed-v2/left-final-coast-routed/data/01200116.npz'
        if not source.exists() or not old.exists():
            self.skipTest('Optional original native diagnostic arrays absent in source-only package')
        raw=np.load(source)
        with np.load(old) as prior:
            xyz=raw[prior['sampled_raw_indices'],:3]
            residual=abs(xyz.astype(float)@prior['ground_plane_abcd'][:3]+prior['ground_plane_abcd'][3])
            self.assertEqual(np.count_nonzero((residual<.18)!=prior['ground_mask']),1)
        self.assert_persisted_mask(xyz,seed=1200116)


if __name__=='__main__':unittest.main(verbosity=2)
