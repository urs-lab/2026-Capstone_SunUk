import unittest
import math
import numpy as np

from nav2_follow_safety import build_scene, bounded_pursuit_result


class FollowSafetyTests(unittest.TestCase):
    def test_optional_005_speed_keeps_rpm_and_obstacle_bounds(self):
        straight = bounded_pursuit_result(self.scene(), .1, 0., 'TRACKED_PURSUIT', max_linear=.05)
        self.assertAlmostEqual(straight['v'], .05)
        for w in (-.2, 0., .2):
            r = bounded_pursuit_result(self.scene(), .1, w, 'TRACKED_PURSUIT', max_linear=.05)
            rpm = max(abs(r['v']-.28*r['w']), abs(r['v']+.28*r['w']))/.065*60/(2*math.pi)
            self.assertLessEqual(rpm, 10.+1e-9)
        blocked = bounded_pursuit_result(self.scene(point=(.50, 0.)), .05, 0.,
                                         'TRACKED_PURSUIT', max_linear=.05)
        self.assertEqual((blocked['v'], blocked['w']), (0., 0.))

    def scene(self, point=(2.0, 0.0), pose_stamp=1_000_000_000,
              scan_stamp=1_100_000_000, now_ns=1_200_000_000):
        return build_scene(
            pose_row=(pose_stamp, 10.0, (0.0, 0.0, 0.0, 0.0, 0.0)),
            scan_row=(scan_stamp, 10.1, [point]),
            now_ns=now_ns, now_mono=10.2)

    def test_map_pose_and_base_scan_make_fresh_scene_without_odom_scan_bracket(self):
        scene = self.scene()
        self.assertAlmostEqual(scene['pose_scan_delta_s'], .1)
        self.assertGreater(scene['clearance'], .3)
        self.assertEqual(scene['source_stamps']['follower_map'], 1_000_000_000)

    def test_numpy_scan_points_are_accepted(self):
        scene = build_scene(
            pose_row=(1_000_000_000, 10.0, (0.0, 0.0, 0.0, 0.0, 0.0)),
            scan_row=(1_100_000_000, 10.1, np.asarray([[2.0, 0.0], [2.0, 0.1]])),
            now_ns=1_200_000_000, now_mono=10.2)
        self.assertEqual(scene['points'].shape, (2, 2))

    def test_former_303ms_filtered_scan_bracket_gap_no_longer_requires_interpolation(self):
        scene = self.scene(pose_stamp=1_000_000_000, scan_stamp=1_303_000_000,
                           now_ns=1_400_000_000)
        self.assertAlmostEqual(scene['pose_scan_delta_s'], .303)
        self.assertLess(scene['uncertainty'], .15)

    def test_stale_pose_or_scan_fails_closed(self):
        with self.assertRaisesRegex(ValueError, 'FOLLOW_POSE_STALE'):
            build_scene(pose_row=(1, 10., (0., 0., 0., 0., 0.)),
                        scan_row=(1_100_000_000, 10.1, [(2., 0.)]),
                        now_ns=1_200_000_000, now_mono=10.2)
        with self.assertRaisesRegex(ValueError, 'FOLLOW_SCAN_STALE'):
            build_scene(pose_row=(1_000_000_000, 10., (0., 0., 0., 0., 0.)),
                        scan_row=(1, 10.1, [(2., 0.)]),
                        now_ns=1_200_000_000, now_mono=10.2)

    def test_large_scan_pose_skew_fails_closed(self):
        with self.assertRaisesRegex(ValueError, 'FOLLOW_POSE_SCAN_TIME_MISMATCH'):
            self.scene(scan_stamp=1_500_000_000, now_ns=1_600_000_000)

    def test_near_obstacle_stops_without_detour(self):
        result = bounded_pursuit_result(self.scene(point=(.50, 0.)), .04, 0.,
                                        'TRACKED_PURSUIT')
        self.assertEqual(result['reason'], 'STOP_FOLLOW_CLEARANCE')
        self.assertEqual((result['motion'], result['v'], result['w']), ('STOP', 0., 0.))

    def test_swept_path_stops_and_forward_command_is_bounded(self):
        scene = self.scene(point=(.68, 0.))
        result = bounded_pursuit_result(scene, .10, .20, 'TRACKED_PURSUIT')
        self.assertIn(result['reason'], ('STOP_FOLLOW_SWEPT_CLEARANCE', 'TRACKED_PURSUIT'))
        self.assertLessEqual(result['v'], .04)
        self.assertLessEqual(abs(result['w']), result['v'] / .28 + 1e-9)

    def test_leader_wait_reason_does_not_request_follower_motion(self):
        result = bounded_pursuit_result(self.scene(), 0., 0., 'STOP_LEADER_STOPPED')
        self.assertEqual(result['reason'], 'FOLLOW_WAITING_LEADER')
        self.assertEqual((result['motion'], result['v'], result['w']), ('STOP', 0., 0.))


if __name__ == '__main__':
    unittest.main()
