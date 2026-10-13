"""Counterexamples for the narrow, actual-body sampled road12 supplement."""
from pathlib import Path
import copy
import tempfile
import unittest

from audit_roundabout_sampled_geometry import (bracket_geometry, check_bound_files,
    sha, strict_failures_eligible, EXPECTED_FAILURES, corridor_eligible, CORRIDOR_FAILURES)


def sample(index,x,y=0.):
    return {'sample':index,'frame':100+index,'timestamp':index*.05,'xyz':[x,y,0.],
        'yaw_deg':0.,'speed_mps':5.2,
        'body_xy':[[x-2,y-1],[x+2,y-1],[x+2,y+1],[x-2,y+1]]}


class SampledGeometryTests(unittest.TestCase):
    def test_actual_short_road_bracket_and_bodies(self):
        self.assertTrue(bracket_geometry(sample(0,-.01),sample(1,.25),[0,0,0],[.2,0,0])['passed'])

    def test_never_reaches_road_end_rejected(self):
        self.assertFalse(bracket_geometry(sample(0,-.28),sample(1,-.02),[0,0,0],[.2,0,0])['passed'])

    def test_passes_beside_road_rejected(self):
        self.assertFalse(bracket_geometry(sample(0,-.01,2),sample(1,.25,2),[0,0,0],[.2,0,0])['passed'])

    def test_skipped_frame_rejected(self):
        self.assertFalse(bracket_geometry(sample(0,-.01),sample(2,.25),[0,0,0],[.2,0,0])['passed'])

    def test_implausible_jump_rejected(self):
        self.assertFalse(bracket_geometry(sample(0,-1),sample(1,1),[0,0,0],[.2,0,0])['passed'])

    def test_wrong_heading_rejected(self):
        a=sample(0,-.01);a['yaw_deg']=90.
        self.assertFalse(bracket_geometry(a,sample(1,.25),[0,0,0],[.2,0,0])['passed'])

    def test_center_cross_without_recorded_body_coverage_rejected(self):
        a,b=sample(0,-.01),sample(1,.25)
        a['body_xy']=[[-.02,-.01],[0.,-.01],[0.,.01],[-.02,.01]]
        self.assertFalse(bracket_geometry(a,b,[0,0,0],[.2,0,0])['passed'])

    def test_modified_bound_source_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'source';p.write_bytes(b'original');binding={p:sha(p)}
            check_bound_files(binding);p.write_bytes(b'altered')
            with self.assertRaises(ValueError):check_bound_files(binding)

    def test_extra_strict_failure_rejected(self):
        report={'scenario_actions_validated':False,'failed_checks':list(EXPECTED_FAILURES),
            'checks':{**{k:False for k in EXPECTED_FAILURES},'clearance':True}}
        self.assertTrue(strict_failures_eligible(report))
        report['checks']['clearance']=False;report['failed_checks'].append('clearance')
        self.assertFalse(strict_failures_eligible(report))

    def test_conflict_and_clearance_not_waived(self):
        pair={'precise_conflict_simultaneous_samples':0,'precise_conflict_samples':[],
              'native_swept_body_conflict_patch_count':2,'minimum_entry_phase_obb':{'clearance_m':2.}}
        report={'supplementary_precise_geometry_passed':False,'failed_checks':list(CORRIDOR_FAILURES),
            'checks':{**{k:False for k in CORRIDOR_FAILURES},'geometry':True},'pair_reviews':{'hero':pair}}
        self.assertTrue(corridor_eligible(report,['hero']))
        bad=copy.deepcopy(report);bad['pair_reviews']['hero']['precise_conflict_simultaneous_samples']=1
        self.assertFalse(corridor_eligible(bad,['hero']))
        bad=copy.deepcopy(report);bad['pair_reviews']['hero']['minimum_entry_phase_obb']['clearance_m']=1.49
        self.assertFalse(corridor_eligible(bad,['hero']))


if __name__=='__main__':unittest.main()
