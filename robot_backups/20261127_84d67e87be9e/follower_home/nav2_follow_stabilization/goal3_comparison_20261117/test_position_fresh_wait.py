import importlib.util
import unittest
from pathlib import Path

ROOT = Path('/home/sunuk/nav2_follow_stabilization/success_tf_recovery')
spec = importlib.util.spec_from_file_location('fresh_wait', ROOT/'position_profile_fresh_wait.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FreshWaitTests(unittest.TestCase):
    def sample(self, stamp=9_900_000_000, received=9.9):
        return (stamp, received, 'follower/odom', 'follower/base_link')

    def test_missing_stale_future_wait_then_fresh_input_allows_start(self):
        for sample in (None, self.sample(1), self.sample(11_000_000_000), self.sample(received=1)):
            with self.subTest(sample=sample):
                self.assertFalse(module.readiness(1, 0, sample, 10_000_000_000, 10)[0])
        self.assertFalse(module.readiness(0, 0, self.sample(), 10_000_000_000, 10)[0])
        self.assertTrue(module.readiness(1, 0, self.sample(), 10_000_000_000, 10)[0])

    def test_duplicate_sources_existing_ekf_wrong_frame_still_rejected(self):
        for sources, filtered, sample in ((2, 0, self.sample()), (1, 1, self.sample()),
                                          (1, 0, (9_900_000_000, 9.9, 'map', 'base_link'))):
            with self.subTest(sources=sources, filtered=filtered):
                with self.assertRaises(RuntimeError):
                    module.readiness(sources, filtered, sample, 10_000_000_000, 10)

    def test_sensor_process_configuration_is_unchanged(self):
        original = Path('/home/sunuk/follower_diagnostics/follower_position_fixed_lidar.launch.py').read_text()
        expected = original.replace('ROOT=Path(__file__).resolve().parent', "ROOT=Path.home()/'follower_diagnostics'")
        expected = expected.replace('    from position_profile_contract import preflight',
            "    here=str(Path(__file__).resolve().parent)\n    if here not in sys.path:sys.path.insert(0,here)\n    from position_profile_fresh_wait import preflight")
        self.assertEqual((ROOT/'follower_position_fixed_lidar_fresh_wait.launch.py').read_text(), expected)


if __name__ == '__main__':
    unittest.main()
