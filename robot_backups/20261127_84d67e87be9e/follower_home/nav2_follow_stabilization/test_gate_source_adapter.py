"""Exercise the actual ROS callback bodies without ROS or physical devices."""
import ast
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as S
import unittest

ROOT=Path(__file__).parent
CODE=ROOT/'candidate/follower_diagnostics'
sys.path.insert(0,str(CODE))
from convoy_source_time import status_payload,time_issue
from nav2_convoy_gate_core import Nav2ConvoyGateCore


def adapter():
    tree=ast.parse((CODE/'nav2_convoy_gate.py').read_text())
    classes=[n for n in tree.body if isinstance(n,ast.ClassDef)]
    env=dict(Node=object,time=time,status_payload=status_payload,time_issue=time_issue)
    exec(compile(ast.Module(body=classes,type_ignores=[]),'gate_adapter','exec'),env)
    gate=env['Nav2ConvoyGate'].__new__(env['Nav2ConvoyGate'])
    gate.core=Nav2ConvoyGateCore();gate.source_rejections={}
    gate.get_clock=lambda:S(now=lambda:S(nanoseconds=2_000_000_000))
    return gate


class GateSourceTests(unittest.TestCase):
    def test_old_healthy_motor_status_is_rejected_on_fresh_receipt(self):
        gate=adapter()
        gate.motor_cb(S(data=json.dumps(dict(status_ros_ns=1_000_000_000,armed=True,
                                             feedback_fresh=True,io_failed=False,fault=None))))
        self.assertIsNone(gate.core.motor)
        self.assertEqual(gate.source_rejections['motor_status'],'SOURCE_STALE')

    def test_current_publication_does_not_hide_old_measurement(self):
        gate=adapter()
        gate.follower_cb(S(data=json.dumps(dict(status_ros_ns=2_000_000_000,
                                                source_stamp_ns=1_000_000_000))))
        self.assertIsNone(gate.core.follower)

    def test_missing_stamp_fails_closed_and_current_motor_is_accepted(self):
        gate=adapter();gate.motor_cb(S(data='{}'))
        self.assertIsNone(gate.core.motor)
        gate.motor_cb(S(data='{"status_ros_ns": 2000000000, "feedback_age_s": 0.01}'))
        self.assertIsNotNone(gate.core.motor)

    def test_feedback_age_includes_time_spent_in_transport(self):
        gate=adapter()
        gate.motor_cb(S(data='{"status_ros_ns": 1750000000, "feedback_age_s": 0.1}'))
        self.assertIsNone(gate.core.motor)
        self.assertEqual(gate.source_rejections['motor_status'],'FEEDBACK_STALE_AT_RECEIPT')

    def test_delayed_pose_clears_previous_pose(self):
        gate=adapter();gate.core.leader=(1.,0.,time.monotonic())
        msg=S(header=S(frame_id='map',stamp=S(sec=1,nanosec=0)),child_frame_id='base_link')
        gate.leader_cb(msg)
        self.assertIsNone(gate.core.leader)

    def test_future_pose_and_wrong_frame_are_rejected(self):
        gate=adapter()
        for frame,sec in [('map',3),('odom',2)]:
            gate.follower_pose_cb(S(header=S(frame_id=frame,stamp=S(sec=sec,nanosec=0)),
                                    child_frame_id='follower/base_link'))
            self.assertIsNone(gate.core.follower_pose)
