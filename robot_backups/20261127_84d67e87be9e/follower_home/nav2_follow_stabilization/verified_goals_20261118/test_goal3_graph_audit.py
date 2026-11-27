import unittest
from types import SimpleNamespace as S
from check_demo_ready import goal3_graph_issues

COMMAND='/follower/motor_candidate/continuous20_command'


class Node:
    def __init__(self):self.extra={}
    def get_publishers_info_by_topic(self,topic):
        names=self.extra.get(topic,[] if topic in ('/cmd_vel',COMMAND) else ['source'])
        return [S(node_name=x) for x in names]
    def get_subscriptions_info_by_topic(self,topic):
        return [S(node_name='md200t_controller' if topic=='/cmd_vel' else 'follower_drive_candidate')]


class Tests(unittest.TestCase):
    def test_correct_goal3_support_graph_is_ready_with_operator_stopped(self):
        self.assertEqual(goal3_graph_issues(Node(),COMMAND),[])
    def test_nav2_gate_left_running_is_reported_as_command_conflict(self):
        n=Node();n.extra['/cmd_vel']=['nav2_convoy_gate']
        issues=goal3_graph_issues(n,COMMAND)
        self.assertEqual(issues[0]['topic'],'/cmd_vel');self.assertEqual(issues[0]['actual_nodes'],['nav2_convoy_gate'])
    def test_missing_bridge_and_duplicate_raw_publishers_are_reported(self):
        n=Node();n.extra['/follower/odom_map']=[];n.extra['/follower/odom_raw']=['a','b']
        issues=goal3_graph_issues(n,COMMAND)
        self.assertEqual({x['topic'] for x in issues},{'/follower/odom_map','/follower/odom_raw'})


if __name__=='__main__':unittest.main()
