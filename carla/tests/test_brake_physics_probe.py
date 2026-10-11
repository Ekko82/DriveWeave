"""Offline checks only: no CARLA Client, world, or native tick."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ast
import unittest
import numpy as np
import brake_physics_probe as probe


class ProbePolicyTests(unittest.TestCase):
    def test_select_only_requested_modes(self):
        cli = probe.build_parser()
        args = cli.parse_args(['--owned-fresh-world', '--out', 'fresh-test-not-created',
                              '--modes', 'coast_only', 'neutral_low', 'tiny_throttle_low'])
        self.assertEqual(args.modes, ['coast_only', 'neutral_low', 'tiny_throttle_low'])
        self.assertIsNone(args.mode)

    def test_individual_mode_parser_remains_available(self):
        args = probe.build_parser().parse_args(['--owned-fresh-world', '--out', 'unused',
                                               '--mode', 'coast_only', '--mode', 'fixed_0.01'])
        self.assertEqual(args.mode, ['coast_only', 'fixed_0.01'])

    def test_zero_coast_has_no_hidden_brake(self):
        q = probe.diagnostic_control_request('coast_only', 'brake', 1.4)
        self.assertEqual(q, dict(throttle=0., brake=0., manual_gear_shift=False, gear=0))
        self.assertEqual(probe.diagnostic_control_request('coast_only', 'hold', .05)['brake'], .35)

    def test_neutral_only_after_actual_low_phase(self):
        self.assertFalse(probe.diagnostic_control_request('neutral_low', 'brake', 3.)['manual_gear_shift'])
        q = probe.diagnostic_control_request('neutral_low', 'brake', 2.4, 0.)
        self.assertTrue(q['manual_gear_shift'])
        self.assertEqual(q['gear'], 0)
        self.assertEqual(q['brake'], .01)
        self.assertTrue(probe.diagnostic_control_request('neutral_low', 'hold', .05, 1.)['manual_gear_shift'])

    def test_tiny_throttle_continuous_ramp(self):
        q = [probe.diagnostic_control_request('tiny_throttle_low', 'brake', 2., t)
             for t in [0., .05, .25, .5, 1.]]
        np.testing.assert_allclose([r['throttle'] for r in q], [0., .002, .01, .02, .02])
        self.assertTrue(all(r['brake'] == .01 for r in q))
        self.assertEqual(probe.diagnostic_control_request('tiny_throttle_low', 'hold', .05, 2.)['throttle'], 0.)

    def test_native_vehicle_control_manual_gear_fields_preserved(self):
        q = probe.diagnostic_control_request('neutral_low', 'brake', 2., .1)
        control = probe.carla.VehicleControl(**q, steer=0., hand_brake=False, reverse=False)
        record = probe.control_record(control)
        self.assertTrue(record['manual_gear_shift'])
        self.assertEqual(record['gear'], 0)
        self.assertFalse(record['hand_brake'])

    def test_tapered_brake_becomes_exact_zero(self):
        for mode in ['tapered_coast', 'tapered_tiny_drive']:
            self.assertAlmostEqual(probe.diagnostic_control_request(mode, 'brake', 2.8)['brake'], .01)
            self.assertAlmostEqual(probe.diagnostic_control_request(mode, 'brake', 2.65)['brake'], .005)
            self.assertEqual(probe.diagnostic_control_request(mode, 'brake', 2.5)['brake'], 0.)
            for elapsed in [.0, .05, .5, 2., 6.]:
                q = probe.diagnostic_control_request(mode, 'brake', 2., elapsed)
                self.assertEqual(q['brake'], 0.)
                self.assertFalse(q['manual_gear_shift'])
        self.assertEqual(probe.diagnostic_control_request('tapered_coast', 'brake', 3., 1.)['brake'], 0.)

    def test_tiny_drive_uses_true_speed_and_elapsed_feedback(self):
        q = probe.diagnostic_control_request('tapered_tiny_drive', 'brake', 2., .5)
        self.assertGreater(q['throttle'], 0.)
        self.assertEqual(q['brake'], 0.)
        self.assertLess(probe.diagnostic_control_request('tapered_tiny_drive', 'brake', 1., .5)['throttle'], q['throttle'])
        self.assertLess(probe.diagnostic_control_request('tapered_tiny_drive', 'brake', 2., 4.)['throttle'], q['throttle'])
        self.assertEqual(probe.diagnostic_control_request('tapered_tiny_drive', 'brake', 2., 6.)['throttle'], 0.)
        self.assertEqual(probe.diagnostic_control_request('tapered_tiny_drive', 'brake', 2., 0.)['throttle'], 0.)

    def test_no_runtime_state_overwrite_api(self):
        tree = ast.parse(Path(probe.__file__).read_text())
        prohibited = {'set_transform', 'set_target_velocity', 'set_velocity',
                      'set_simulate_physics', 'apply_physics_control'}
        self.assertFalse(any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                             and n.func.attr in prohibited for n in ast.walk(tree)))

    def test_actual_speed_derivatives_keep_zero_speed_discontinuity(self):
        # This deliberately fails the same declared jerk target. It proves the
        # helper retains a low-speed physical discontinuity rather than hiding it.
        rows = [dict(sim_time_s=i*.05, speed_mps=1.6 if i < 10 else 0.,
                     native_acceleration_forward_mps2=0.) for i in range(25)]
        thresholds = probe.load_thresholds(Path(probe.__file__).parent / 'verification/motion-quality-thresholds-v2.json')
        metrics = probe.derivatives(rows, thresholds, .3)
        self.assertFalse(metrics['braking_with_actual_transition']['jerk_target_met'])
        self.assertGreater(metrics['braking_with_actual_transition']['actual_windowed_jerk_mps3']['maximum_absolute'], 12.)
        self.assertTrue(any(abs(r['raw_speed_jerk_mps3']) > 12. for r in rows))


if __name__ == '__main__':
    unittest.main(verbosity=2)
