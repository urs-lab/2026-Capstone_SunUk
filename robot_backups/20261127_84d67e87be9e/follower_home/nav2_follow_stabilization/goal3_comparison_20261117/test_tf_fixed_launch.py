import unittest
from pathlib import Path
from test_amcl_tf_candidate import load_nodes, ROOT, Tests


class FixedLaunchTests(unittest.TestCase):
    def test_only_amcl_library_environment_changes(self):
        original = load_nodes(ROOT/'follower_diagnostics/follower_localization.launch.py')
        changed = load_nodes(ROOT/'nav2_follow_stabilization/success_tf_recovery/follower_localization_tf_fixed.launch.py')
        for old, new in zip(original, changed):
            new = dict(new)
            if new['name'] == 'amcl':
                environment = new.pop('additional_env')
                self.assertEqual(set(environment), {'LD_LIBRARY_PATH'})
                library_dir = Path(environment['LD_LIBRARY_PATH'].split(':')[0])
                self.assertTrue((library_dir/'libtf2.so').is_file())
            self.assertEqual(new, old)

    def test_stack_only_selects_fixed_localization_and_fresh_start_wait(self):
        folder = ROOT/'nav2_follow_stabilization/success_tf_recovery'
        original = (folder/'follower_success_tf_stack.launch.py').read_text()
        candidate = (folder/'follower_success_tf_stack_fixed.launch.py').read_text()
        self.assertEqual(candidate, original.replace(
            "localization=root/'follower_localization.launch.py'",
            "localization=here/'follower_localization_tf_fixed.launch.py'").replace(
            "sensor=root/'follower_position_fixed_lidar.launch.py'",
            "sensor=here/'follower_position_fixed_lidar_fresh_wait.launch.py'"))


if __name__ == '__main__':
    unittest.main()
