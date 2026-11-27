import unittest
from stability_core import Readiness, source_time_issue


class StabilityTests(unittest.TestCase):
    def test_old_queued_message_is_not_made_fresh_by_receipt(self):
        c=Readiness({'pose':(.3,.05)})
        c.observe('pose',1_000_000_000,10.)
        r=c.evaluate(2_000_000_000,10.)
        self.assertFalse(r['ready']);self.assertIn('pose:SOURCE_STALE',r['issues'])

    def test_future_map_transform_is_allowed_but_future_measurement_is_not(self):
        self.assertEqual(source_time_issue(2_000_000_000,1_000_000_000), 'SOURCE_IN_FUTURE')
        self.assertIsNone(source_time_issue(2_000_000_000,1_000_000_000,future_allowance=1.1))

    def test_five_continuous_seconds_then_loss_clears_readiness(self):
        c=Readiness({'pose':(.3,.05)})
        for i in range(52):
            ns=1_000_000_000+i*100_000_000
            c.observe('pose',ns,i*.1);r=c.evaluate(ns,i*.1)
        self.assertTrue(r['ready'])
        self.assertFalse(c.evaluate(ns+500_000_000,5.6)['ready'])
        c.observe('pose',ns+600_000_000,5.7)
        self.assertFalse(c.evaluate(ns+600_000_000,5.7)['ready'])

    def test_frozen_stamp_and_backward_stamp_fail(self):
        c=Readiness({'pose':(.3,.05)})
        c.observe('pose',1_000_000_000,1.)
        c.observe('pose',1_000_000_000,1.5)
        self.assertIn('pose:SOURCE_STALE',c.evaluate(1_500_000_000,1.5)['issues'])
        c.observe('pose',900_000_000,1.6)
        self.assertIn('pose:SOURCE_BACKWARDS',c.evaluate(1_600_000_000,1.6)['issues'])

    def test_invalid_and_missing_are_never_ready(self):
        c=Readiness({'pose':(.3,.05)})
        self.assertFalse(c.evaluate(1_000_000_000,1.)['ready'])
        c.observe('pose',1_000_000_000,1.,False)
        self.assertIn('pose:INVALID',c.evaluate(1_000_000_000,1.)['issues'])
