"""Goal1 profile and real child publication stall; no ROS or serial IO."""
import argparse
import ast
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from isolated_telemetry import PublisherProcess
from test_isolated_telemetry import blocked_publication,wait_until
from test_unlimited_follow_session import UnlimitedTests
from test_local_avoidance_drive import Tests as Setup


class Goal1Tests(unittest.TestCase):
    def test_exact_goal1_hardware_arguments_validate_before_ros_import(self):
        source=Path(__file__).with_name('follower_drive_node.py')
        main=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='main')
        body=[]
        for n in main.body:
            if isinstance(n,ast.Import) and any(a.name=='rclpy' for a in n.names):break
            body.append(n)
        args=['driver','--run-hardware','--ack-hardware-zero-rpm-tx','--isolated-telemetry',
              '--input-kind','local_avoidance','--ack-bounded-pivot-candidate',
              '--enable-motion-input','--disable-clearance-stop','--ack-extended-follow-speed',
              '--trial-max-rpm','15','--trial-seconds','0','--fault-snapshot-dir','/tmp/goal1_fault_test']
        main.name='parse_goal1';main.body=body+[ast.Return(value=ast.Call(func=ast.Name(id='locals',ctx=ast.Load()),args=[],keywords=[]))]
        scope={'argparse':argparse,'__doc__':'test','Path':Path}
        with patch('sys.argv',args):
            exec(compile(ast.fix_missing_locations(ast.Module(body=[main],type_ignores=[])),str(source),'exec'),scope)
            parsed=scope['parse_goal1']()['a']
        self.assertTrue(parsed.isolated_telemetry)
        self.assertTrue(parsed.disable_clearance_stop)
        self.assertTrue(parsed.ack_extended_follow_speed)
        self.assertFalse(parsed.ack_continuous20_candidate)
        self.assertEqual((parsed.trial_max_rpm,parsed.trial_seconds),(15,0))

    def test_goal1_15rpm_commands_survive_status_child_gil_stall(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder)
            worker=PublisherProcess(folder/'slot','status',str(folder/'trace.json'),10,'test',target=blocked_publication)
            clock,device,contract=UnlimitedTests().make()
            contract.clearance_stop_enabled=False
            epoch=contract.core.epoch
            try:
                worker.offer(b'block');wait_until(lambda:worker.stats().get('publish_started_mono',0)>0)
                start=previous=time.monotonic();maximum_gap=0.;commands=0
                while time.monotonic()-start<1.9:
                    mono=time.monotonic();maximum_gap=max(maximum_gap,mono-previous);previous=mono
                    command=Setup().envelope(clock,contract,'DRIVE',.1,0.)
                    self.assertTrue(contract.command(json.dumps(command),clock.ros),contract.last_command_rejection)
                    contract.tick(clock.ros);clock.now+=.02;commands+=1
                    self.assertIsNone(contract.core.fault)
                    self.assertLessEqual(max(abs(v) for v in device.rpm),15)
                    time.sleep(.01)
                self.assertLess(maximum_gap,.3);self.assertGreater(commands,30)
                self.assertTrue(contract.core.armed);self.assertEqual(contract.core.epoch,epoch)
                self.assertFalse(contract.continuous20)
                self.assertFalse(contract.status()['clearance_stop_enabled'])
                self.assertEqual(contract.core.config.response_timeout,.3)
                self.assertEqual(contract.core.config.feedback_timeout,.3)
                self.assertEqual(contract.core.config.command_timeout,.3)
            finally:
                contract.disarm();contract.tick(clock.ros)
                self.assertEqual(device.rpm,(0,0));worker.close()


if __name__=='__main__':unittest.main()
