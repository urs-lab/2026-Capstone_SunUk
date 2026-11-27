"""Lifecycle policy and launch dictionaries only; no ROS nodes or services."""
import ast
import hashlib
import importlib.util
from pathlib import Path
import sys
import subprocess
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch
from readiness_policy import ready,odom_ready

ROOT=Path(__file__).resolve().parent
SUCCESS=ROOT.parent/'goal3_success_tf02524'


def launch_nodes(path):
    captured=[]
    class Config:
        def __init__(self,name):self.name=name
        def perform(self,context):
            return {'enable_localization':'true','map':'/home/sunuk/robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml'}[self.name]
    def node(**kwargs):captured.append(kwargs);return kwargs
    modules={'launch':S(LaunchDescription=lambda x:x),
             'launch.actions':S(DeclareLaunchArgument=lambda *a,**k:None,LogInfo=lambda **k:None,OpaqueFunction=lambda **k:None),
             'launch.substitutions':S(LaunchConfiguration=Config),'launch_ros':S(),'launch_ros.actions':S(Node=node)}
    with patch.dict(sys.modules,modules):
        spec=importlib.util.spec_from_file_location('localization_config_under_test',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        module.localization(None)
    return captured


class Tests(unittest.TestCase):
    def test_original_failure_map_active_amcl_inactive_is_not_ready(self):
        self.assertFalse(ready({'map_server':(3,10.),'amcl':(2,10.)},(False,10.),10.))
    def test_both_active_without_manager_bonds_is_not_ready(self):
        self.assertFalse(ready({'map_server':(3,10.),'amcl':(3,10.)},(False,10.),10.))
    def test_only_fresh_active_nodes_and_active_manager_are_ready(self):
        states={'map_server':(3,10.),'amcl':(3,10.)}
        self.assertTrue(ready(states,(True,10.),10.))
        self.assertFalse(ready(states,(True,10.),14.))
        self.assertFalse(ready(states,None,10.))
    def test_map_odom_requires_fresh_original_and_receive_time(self):
        self.assertTrue(odom_ready((10_000_000_000,10.),10_100_000_000,10.1))
        self.assertFalse(odom_ready((9_000_000_000,10.),10_100_000_000,10.1))
        self.assertFalse(odom_ready((11_000_000_000,10.),10_100_000_000,10.1))
    def test_launch_retains_amcl_map_and_manager_except_bond_timeout(self):
        old=launch_nodes(SUCCESS/'localization/follower_localization_tf02524.launch.py')
        new=launch_nodes(ROOT/'localization/follower_localization_tf02524.launch.py')
        self.assertEqual(old[0],new[0])
        old_amcl=dict(old[1]);new_amcl=dict(new[1])
        old_amcl.pop('additional_env');new_amcl.pop('additional_env')
        self.assertEqual(old_amcl,new_amcl)
        expected=dict(old[2]['parameters'][0],bond_timeout=20.)
        self.assertEqual(new[2]['parameters'],[expected])
        self.assertGreater(new[2]['parameters'][0]['bond_timeout'],0.)
    def test_same_tf_library_and_bridge_as_success(self):
        for rel in ('lib/libtf2.so','localization/real_follower_bridge_tf_candidate.py'):
            self.assertEqual((ROOT/rel).read_bytes(),(SUCCESS/rel).read_bytes())
    def test_watcher_has_no_publishers_or_state_changes(self):
        source=(ROOT/'watch_localization_ready.py').read_text();tree=ast.parse(source)
        names=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        self.assertNotIn('create_publisher',names)
        self.assertNotIn('ChangeState',source)
        self.assertIn('/is_active',source)
    def test_stack_staggers_localization_without_motion_nodes(self):
        text=(ROOT/'localization/follower_success_tf_stack.launch.py').read_text()
        tree=ast.parse(text)
        timer=next(n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='TimerAction')
        self.assertEqual(next(k.value.value for k in timer.keywords if k.arg=='period'),4.)
        self.assertNotIn('follower_drive_node',text)
        self.assertNotIn('cmd_vel',text)

    def test_bridge_imports_with_only_its_own_directory_as_local_source(self):
        # A fresh isolated interpreter cannot reuse this test process's
        # imported helper from the original success folder.
        script='''
import pathlib,runpy,sys,types
root=pathlib.Path(sys.argv[1])
sys.path.insert(0,str(root))
def module(name,**attributes):
    value=types.ModuleType(name);value.__dict__.update(attributes);sys.modules[name]=value
module('rclpy')
module('rclpy.duration',Duration=object)
module('rclpy.executors',ExternalShutdownException=RuntimeError)
module('rclpy.node',Node=object)
module('rclpy.qos',QoSProfile=object,ReliabilityPolicy=object,DurabilityPolicy=object,HistoryPolicy=object)
module('nav_msgs');module('nav_msgs.msg',Odometry=object)
module('tf2_ros',Buffer=object,TransformException=RuntimeError,TransformListener=object)
runpy.run_path(str(root/'real_follower_bridge_tf_candidate.py'),run_name='bridge_import_only')
import follower_input_guard
assert pathlib.Path(follower_input_guard.__file__).resolve().parent==root.resolve()
print('BRIDGE_IMPORT_CLOSURE_OK')
'''
        with tempfile.TemporaryDirectory() as cwd:
            result=subprocess.run([sys.executable,'-I','-B','-c',script,str(ROOT/'localization')],
                cwd=cwd,capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('BRIDGE_IMPORT_CLOSURE_OK',result.stdout)

    def test_guard_module_is_exact_success_backup(self):
        self.assertEqual((ROOT/'localization/follower_input_guard.py').read_bytes(),
                         (SUCCESS/'localization/follower_input_guard.py').read_bytes())


if __name__=='__main__':unittest.main()
