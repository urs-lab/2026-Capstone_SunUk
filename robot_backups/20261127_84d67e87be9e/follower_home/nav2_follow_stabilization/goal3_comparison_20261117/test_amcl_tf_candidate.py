import importlib.util,sys,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
ROOT=Path('/home/sunuk')

def load_nodes(path):
    fake={'launch':S(LaunchDescription=lambda x:x),
          'launch.actions':S(DeclareLaunchArgument=lambda *a,**kw:None,LogInfo=lambda **kw:S(log=kw),OpaqueFunction=lambda **kw:None),
          'launch.substitutions':S(LaunchConfiguration=lambda name:S(perform=lambda context:context[name])),
          'launch_ros.actions':S(Node=lambda **kw:kw)}
    with patch.dict(sys.modules,fake):
        spec=importlib.util.spec_from_file_location('temporary_localization',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        return [x for x in m.localization({'enable_localization':'true','map':str(ROOT/'robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml')}) if isinstance(x,dict)]

class Tests(unittest.TestCase):
    def test_only_two_amcl_subscription_parameters_differ(self):
        old=load_nodes(ROOT/'follower_diagnostics/follower_localization.launch.py')
        new=load_nodes(ROOT/'nav2_follow_stabilization/success_tf_recovery/follower_localization_tf_be.launch.py')
        self.assertEqual(len(old),3);self.assertEqual(len(new),3)
        for a,b in zip(old,new):
            if a['name']!='amcl':self.assertEqual(a,b);continue
            import yaml
            self.assertEqual(b['parameters'][1],a['parameters'][0]);self.assertEqual(b['remappings'],a['remappings'])
            override=yaml.safe_load(Path(b['parameters'][0]).read_text())['/**']['ros__parameters']
            self.assertEqual(override,{'qos_overrides./tf.subscription.reliability':'best_effort','qos_overrides./tf.subscription.depth':10})
    def test_driver_and_stop03_operators_are_not_replaced(self):
        from hashlib import sha256
        root=ROOT/'nav2_follow_stabilization/evaluation3_reference_wait_stop03'
        import json
        latest={}
        for current in ('raw_wait_status_implementation_20261117.json','startup_alignment_implementation_20261117.json'):
            latest_report=root/current
            if latest_report.exists():latest.update(json.loads(latest_report.read_text())['sources'])
        for report_name in ('telemetry_isolation_implementation_report_20261117.json','clearance_reobserve_implementation_report_20261117.json'):
            report=json.loads((root/report_name).read_text())
            for rel,digest in report['sources'].items():self.assertEqual(sha256((root/rel).read_bytes()).hexdigest(),latest.get(rel,digest),rel)
        p={};exec((root/'driver/evaluation3_clearance_policy.py').read_text(),p)
        self.assertEqual(p['STOP_CLEARANCE_M'],.03)

if __name__=='__main__':unittest.main()
