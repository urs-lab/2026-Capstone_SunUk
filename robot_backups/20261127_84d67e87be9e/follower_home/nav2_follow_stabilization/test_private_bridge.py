from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path.home()/'robot_ws/test_tools'))
import test_follower_input_safety as f

SOURCE=Path(__file__).parent/'candidate/robot_ws/src/cnu_capstone_2026/convoy_robot'


def bridge():
    original=f.Node
    class CaptureNode(original):
        def create_publisher(self,*args):
            self.publisher_args=args
            return super().create_publisher(*args)
    with patch.object(f,'ROOT',SOURCE),patch.object(f,'Node',CaptureNode):
        return f.load('real_follower_bridge_tf_candidate.py')


class PrivateBridgeTests(unittest.TestCase):
    def setup_node(self,tf_sec=100):
        n=bridge()
        m=f.odom('follower/base_link','follower/odom');n.filtered_odom_callback(m)
        tf=S(header=S(stamp=f.stamp(tf_sec)),transform=S(translation=f.vector(),rotation=m.pose.pose.orientation))
        n.tf_buffer.lookup_transform.return_value=tf
        return n

    def test_latest_pose_output_uses_bounded_best_effort_queue(self):
        n=self.setup_node();q=n.publisher_args[2]
        self.assertEqual(q.depth,1);self.assertEqual(q.reliability,'BEST_EFFORT')
        n.publish_state()
        self.assertEqual(n.pub.publish.call_args.args[0].header.frame_id,'map')

    def test_original_measurement_stamp_is_not_refreshed(self):
        n=self.setup_node();n.publish_state()
        msg=n.pub.publish.call_args.args[0]
        self.assertEqual(msg.header.stamp.sec,100)
        self.assertEqual(msg.header.stamp.nanosec,0)

    def test_stale_future_and_missing_tf_do_not_publish(self):
        for sec in (98,101,0):
            n=self.setup_node(sec);n.publish_state();n.pub.publish.assert_not_called()
        n=self.setup_node();n.tf_buffer.lookup_transform.side_effect=f.MissingTF('missing')
        n.publish_state();n.pub.publish.assert_not_called()
