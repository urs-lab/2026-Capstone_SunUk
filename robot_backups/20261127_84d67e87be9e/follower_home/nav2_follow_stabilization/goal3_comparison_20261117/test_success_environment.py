import json,os,subprocess,tempfile,unittest
from pathlib import Path
ROOT=Path('/home/sunuk/nav2_follow_stabilization')

class Tests(unittest.TestCase):
    def test_actual_wrapper_clears_only_transport_overrides_and_passes_arguments(self):
        original=ROOT/'success_tf_recovery/start_localization_success_env.bash'
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'wrapper.bash').write_bytes(original.read_bytes())
            (p/'start_localization.bash').write_text('''python3 - "$@" <<'INNER'
import os,sys,json
keys=['FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','ROS_DOMAIN_ID','RMW_FASTRTPS_PUBLICATION_MODE']
print(json.dumps(dict(args=sys.argv[1:],environment={k:os.environ.get(k) for k in keys})))
INNER
''')
            env=dict(os.environ,FASTRTPS_DEFAULT_PROFILES_FILE='udp-a.xml',FASTDDS_DEFAULT_PROFILES_FILE='udp-b.xml',ROS_DOMAIN_ID='10',RMW_FASTRTPS_PUBLICATION_MODE='ASYNCHRONOUS')
            r=subprocess.run(['bash',str(p/'wrapper.bash'),'/tmp/example map.yaml'],env=env,capture_output=True,text=True,check=True)
            row=json.loads(r.stdout.splitlines()[-1])
            self.assertEqual(row['args'],['/tmp/example map.yaml'])
            self.assertIsNone(row['environment']['FASTRTPS_DEFAULT_PROFILES_FILE']);self.assertIsNone(row['environment']['FASTDDS_DEFAULT_PROFILES_FILE'])
            self.assertEqual(row['environment']['ROS_DOMAIN_ID'],'10');self.assertEqual(row['environment']['RMW_FASTRTPS_PUBLICATION_MODE'],'ASYNCHRONOUS')
    def test_success_localization_runtime_sources_remain_identical(self):
        old=Path('/home/sunuk/robot_github_backup_20261113/staging/capstone_goal3/home')
        for rel in ('follower_diagnostics/follower_localization.launch.py','follower_diagnostics/follower_position_fixed_lidar.launch.py','follower_diagnostics/follower_observation.launch.py','robot_ws/src/cnu_capstone_2026/config/follower_amcl_candidate.yaml','nav2_follow_stabilization/success_tf_recovery/follower_success_tf_stack.launch.py','nav2_follow_stabilization/success_tf_recovery/real_follower_bridge_tf_candidate.py'):
            self.assertEqual((old/rel).read_bytes(),(Path('/home/sunuk')/rel).read_bytes(),rel)
    def test_three_cm_and_input_wait_components_remain_enabled(self):
        r=ROOT/'evaluation3_reference_wait_stop03'
        policies=[(r/f'{side}/evaluation3_clearance_policy.py').read_text() for side in ('driver','candidate/follower_diagnostics')]
        self.assertEqual(policies[0],policies[1]);env={};exec(policies[0],env)
        self.assertEqual(env['PLAN_CLEARANCE_M'],.1);self.assertEqual(env['STOP_CLEARANCE_M'],.03)
        command=(r/'start_evaluation3.bash').read_text();self.assertIn('--reference-wait',command);self.assertIn('--seconds 0',command)
        s=(r/'candidate/follower_diagnostics/run_local_avoidance_trial.py').read_text()
        for phrase in ('motor_status_wait=MotorStatusWait() if a.reference_wait','filtered_input_wait=ReferenceWait() if a.reference_wait','clearance_wait=ClearanceWait() if a.reference_wait'):
            self.assertIn(phrase,s)

if __name__=='__main__':unittest.main()
