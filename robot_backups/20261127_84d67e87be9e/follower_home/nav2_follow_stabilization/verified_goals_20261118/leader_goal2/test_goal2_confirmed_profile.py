import unittest
import test_distance_runtime as runtime


class Goal2Tests(unittest.TestCase):
    def test_three_runs_each_use_their_own_odom_origin_and_confirmed_scale(self):
        scale=.935594233291917
        start=0.
        for index in range(3):
            with self.subTest(run=index+1):
                f,code,r=runtime.DistanceTests().run_flow(scale=scale,speed=.1,
                    initial_odom_x=start,operator_clear_path=True)
                self.assertEqual(code,0)
                self.assertEqual(r['trial_reason'],'DISTANCE_TARGET_REACHED')
                self.assertTrue(r['leader_zero_confirmed']);self.assertFalse(r['follower_commanded'])
                distance=(f.x-start)*scale
                self.assertGreaterEqual(distance,3.);self.assertLess(distance,3.01)
                self.assertAlmostEqual(r['raw_odom_target_m'],3./scale)
                self.assertTrue(all(0<=v<=.1 for v in f.commands));self.assertEqual(f.commands[-1],0.)
                self.assertFalse(r['scan_used_as_motion_gate'])
                start=f.x


if __name__=='__main__':unittest.main()
