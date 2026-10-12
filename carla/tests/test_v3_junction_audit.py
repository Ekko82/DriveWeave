"""Semantic audit boundaries; synthetic fixtures never prove native driving."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_v3_junction import gate_interaction, role_groups


def trace(occupied):
    return [{'frame': i, 'time_s': i * .05, 'body_in_gate': i in occupied} for i in range(12)]


class V3JunctionAuditTests(unittest.TestCase):
    def test_followers_are_not_accepted_as_opposing_straight_actors(self):
        plan = {'traffic': [{'name': n} for n in ['opponent', 'rear0', 'rear1']],
                'expected_evidence': {'opposing_roles': ['opponent'], 'following_queue_roles': ['rear0', 'rear1']}}
        self.assertEqual(role_groups(plan)['opposing'], ['opponent'])
        plan['expected_evidence']['opposing_roles'].append('rear0')
        with self.assertRaises(ValueError):
            role_groups(plan)

    def test_all_clear_then_cross_is_not_between_streams_demonstration(self):
        proof = gate_interaction(trace({6, 7}), {'first': trace({1, 2}), 'second': trace({3, 4})})
        self.assertTrue(proof['no_simultaneous_body_occupancy'])
        self.assertFalse(proof['gap_between_streams_observed'])
        self.assertEqual(proof['observed_pattern'], 'all_observed_conflict_traffic_cleared_before_entry')
        actual_gap = gate_interaction(trace({6, 7}), {'first': trace({1, 2}), 'later': trace({9, 10})})
        self.assertTrue(actual_gap['gap_between_streams_observed'])

    def test_last_opponent_body_sample_is_still_a_conflict(self):
        proof = gate_interaction(trace({6, 7}), {'opponent': trace({4, 5, 6})})
        self.assertFalse(proof['no_simultaneous_body_occupancy'])
        self.assertEqual(proof['simultaneous_samples'][0]['frame'], 6)
        self.assertFalse(proof['gap_between_streams_observed'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
