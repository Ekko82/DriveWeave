"""Behavior checks for comfort requests and emergency overrides."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
from native_controller import LongitudinalGovernor


class GovernorBehavior(unittest.TestCase):
    def test_normal_request_obeys_jerk_limit_after_speed_target_change(self):
        g=LongitudinalGovernor(.05)
        previous=0.0
        for target in [15.]*100+[0.]*100+[10.]*100:
            throttle,brake=g.step(target,7.)
            self.assertLessEqual(abs(g.requested_acceleration-previous),2.5*.05+1e-10)
            self.assertGreaterEqual(throttle,0.)
            self.assertGreaterEqual(brake,0.)
            self.assertFalse(throttle>.01 and brake>.01)
            previous=g.requested_acceleration

    def test_emergency_brakes_immediately_despite_prior_acceleration(self):
        g=LongitudinalGovernor(.05)
        for _ in range(60): g.step(20.,10.)
        throttle,brake=g.step(0.,10.,emergency=True)
        self.assertEqual((throttle,brake),(0.,1.))

    def test_zero_target_at_rest_holds_brake(self):
        g=LongitudinalGovernor(.05)
        for _ in range(40): throttle,brake=g.step(0.,0.)
        self.assertEqual(throttle,0.)
        self.assertGreater(brake,.25)

    def test_simple_physical_plant_accelerates_and_settles(self):
        # A deliberately different plant tests actual closed-loop behavior,
        # not equality with the governor's internal formulas.
        g=LongitudinalGovernor(.05)
        speed=0.
        for _ in range(600):
            throttle,brake=g.step(10.,speed)
            speed=max(0.,speed+.05*(5.*throttle-7.*brake-.06*speed))
        self.assertGreater(speed,9.0)
        self.assertLess(speed,11.0)
        for _ in range(300):
            throttle,brake=g.step(0.,speed)
            speed=max(0.,speed+.05*(5.*throttle-7.*brake-.06*speed))
        self.assertLess(speed,.25)


if __name__=='__main__': unittest.main()
