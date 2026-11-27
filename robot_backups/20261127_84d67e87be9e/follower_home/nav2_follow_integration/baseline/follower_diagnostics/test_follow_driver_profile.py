import unittest

from local_avoidance_drive import PROFILE
from run_local_avoidance_trial import motor_problem


class FollowDriverProfileTests(unittest.TestCase):
    def status(self, continuous20):
        return dict(continuous20=continuous20,
                    continuous_stop_clearance_m=.1 if continuous20 else .3,
                    mode='HARDWARE_CANDIDATE', input_kind='local_avoidance',
                    motion_profile=PROFILE, input_source_timeout_s=.8,
                    trial_max_rpm=10, trial_max_seconds=360, enabled=True,
                    zero_stop_prearm_supported=True,
                    zero_stop_response_retry_supported=True,
                    zero_response_retry_version=2,
                    zero_reply_hold_supported=True,
                    fault=None, io_failed=False, feedback_fresh=True)

    def test_follow_only_rejects_continuous20_driver_with_specific_recovery_reason(self):
        problem = motor_problem((self.status(True), 5.), 5.01, 360,
                                continuous20=False, follow_only=True)
        self.assertEqual(problem,
                         'FOLLOW_ONLY_REQUIRES_STANDARD_DRIVER_WITHOUT_CONTINUOUS20_ACK')

    def test_follow_only_accepts_matching_standard_driver_profile(self):
        self.assertIsNone(motor_problem((self.status(False), 5.), 5.01, 360,
                                        continuous20=False, follow_only=True))


if __name__ == '__main__':
    unittest.main()
