import math
import unittest
from offline_low_speed_policy import alternative,wheel_rpm


class OfflinePolicyTests(unittest.TestCase):
    def test_does_not_turn_stops_or_faults_into_motion(self):
        for method in ('recorded','floor','deadband'):
            for reason in ('FOLLOW_WAITING_LEADER','FOLLOW_WAITING_LEADER_REVERSE',
                           'FOLLOW_WAITING_LEADER_ROTATION','STOP_STALE_LEADER',
                           'STOP_FOLLOW_CLEARANCE','DEVICE_STATUS_NONZERO'):
                self.assertEqual(alternative(.02,.001,reason,method),(0.,0.))

    def test_floor_preserves_curvature_and_rpm_cap(self):
        for v in (.004,.014,.028,.04,.06):
            for curvature in (-3.,-1.,0.,1.,3.):
                a,b=alternative(v,v*curvature,'TRACKED_PURSUIT','floor')
                self.assertAlmostEqual(b/a,curvature)
                self.assertLessEqual(a,.06+1e-12)
                self.assertLessEqual(max(wheel_rpm(a,b)),10)
                self.assertGreaterEqual(min(wheel_rpm(a,b)),0)

    def test_deadband_can_create_more_stops(self):
        self.assertEqual(alternative(.014,0.,'TRACKED_PURSUIT','deadband'),(0.,0.))
        self.assertEqual(alternative(.028,0.,'TRACKED_PURSUIT','deadband'),(.028,0.))

    def test_invalid_values_fail_without_output(self):
        with self.assertRaises(ValueError):alternative(math.nan,0.,'TRACKED_PURSUIT','floor')
        with self.assertRaises(ValueError):alternative(.01,.2,'TRACKED_PURSUIT','floor')


if __name__=='__main__':unittest.main()
