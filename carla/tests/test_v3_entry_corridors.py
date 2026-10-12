"""Geometry boundary tests for the independent broad-gate refinement."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from audit_v3_entry_corridors import corridor, conflict_patches, rectangle, simultaneous_patch_occupation


class EntryCorridorTests(unittest.TestCase):
    gate=[[-10,-10],[10,-10],[10,10],[-10,10]]

    def test_parallel_native_geometry_has_no_shared_body_corridor(self):
        a=corridor([[-20,0],[20,0]],2.4,1.1,self.gate)
        b=corridor([[-20,3.5],[20,3.5]],1.9,.9,self.gate)
        self.assertEqual(conflict_patches(a,b),[])

    def test_crossing_geometry_rejects_simultaneous_occupation(self):
        a=corridor([[-20,0],[20,0]],2.4,1.1,self.gate)
        b=corridor([[0,-20],[0,20]],1.9,.9,self.gate)
        patches=conflict_patches(a,b)
        self.assertTrue(patches)
        self.assertTrue(simultaneous_patch_occupation(rectangle([-1,0],[1,0],2.4,1.1),
                                                    rectangle([0,-1],[0,1],1.9,.9),patches))

    def test_same_path_split_at_road_boundary_is_still_conflicting(self):
        # Different road/segment partitions cannot exempt a shared physical path.
        a=corridor([[-20,0],[20,0]],2.4,1.1,self.gate)
        b=corridor([[-20,0],[0,0],[20,0]],1.9,.9,self.gate)
        self.assertTrue(simultaneous_patch_occupation(rectangle([0,0],[1,0],2.4,1.1),
                                                    rectangle([2,0],[1,0],1.9,.9),conflict_patches(a,b)))

    def test_center_outside_crossing_does_not_mean_body_clear(self):
        a=corridor([[-20,0],[20,0]],2.4,1.1,self.gate)
        b=corridor([[0,-20],[0,20]],1.9,.9,self.gate)
        # Centers are outside the crossing square, but both bumpers occupy it.
        self.assertTrue(simultaneous_patch_occupation(rectangle([-2.5,0],[1,0],2.4,1.1),
                                                    rectangle([0,-2],[0,1],1.9,.9),conflict_patches(a,b)))


if __name__=='__main__':
    unittest.main()
