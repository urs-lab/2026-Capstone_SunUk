import json,math,sys,unittest
from pathlib import Path
ROOT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'leader'),str(ROOT/'relay')]
from nav2_convoy_gate_core import Nav2ConvoyGateCore
from convoy_source_time import status_payload
from status_fast_send import send_status
from unicast_wire import decode
class FlowChanges(unittest.TestCase):
 def gate(self,gap=1.53,follower_v=.02):
  g=Nav2ConvoyGateCore(max_linear=.1);g.graph_ok=True
  g.update_nav(.15,.09,10.)
  g.update_follower(dict(trial_phase='FOLLOW',reason='TRACKED_PURSUIT',smooth_follow=True,follower_requested_v=follower_v,follower_follow_reason='TRACKED_PURSUIT'),10.)
  g.update_motor(dict(armed=True,fault=None,io_failed=False,feedback_fresh=True),10.)
  g.update_leader_pose(gap,0,10.);g.update_follower_pose(0,0,10.)
  return g
 def test_low_ramp_progress_and_coupling_cap(self):
  self.assertAlmostEqual(self.gate().evaluate(10.).linear_x,.06)
  self.assertAlmostEqual(self.gate(follower_v=.04).evaluate(10.).linear_x,.06)
  self.assertAlmostEqual(self.gate(follower_v=.09).evaluate(10.).linear_x,.1)
  self.assertAlmostEqual(self.gate(gap=1.49,follower_v=.04).evaluate(10.).linear_x,.04)
 def test_curvature_maintained(self):
  r=self.gate().evaluate(10.);self.assertAlmostEqual(r.angular_z/r.linear_x,.09/.15)
 def test_stops_retained(self):
  g=self.gate();g.follower=None;self.assertEqual(g.evaluate(10.).reason,'STOP_FOLLOWER_STATUS_STALE')
  g=self.gate();g.motor[0]['armed']=False;self.assertEqual(g.evaluate(10.).linear_x,0)
  g=self.gate(gap=.84);self.assertEqual(g.evaluate(10.).reason,'STOP_CONVOY_GAP')
  for why in ('STOP_LEADER_REVERSE','STOP_NEAR_LEADER'):
   g=self.gate();g.follower[0]['follower_follow_reason']=why;self.assertEqual(g.evaluate(10.).linear_x,0)
  g=self.gate();g.graph_ok=False;self.assertEqual(g.evaluate(10.).linear_x,0)
 def test_source_time_deadlines_remain_point_three(self):
  now=10000000000
  for age,expected in ((.29,None),(.301,'SOURCE_STALE'),(-.051,'SOURCE_IN_FUTURE')):
   _,issue=status_payload(json.dumps(dict(status_ros_ns=now,source_stamp_ns=now-round(age*1e9))),now,True)
   self.assertEqual(issue,expected)
 def test_immediate_relay_payload_and_channel_sequence_preserved(self):
  class Socket:
   def sendto(self,data,target):self.data=data;self.target=target
  sock=Socket();raw=json.dumps(dict(status_ros_ns=9990000000,source_stamp_ns=9800000000)).encode()
  seq=send_status(sock,'10.253.245.81',20180,25,10000000000,raw)
  c,s,payload=decode(sock.data,10001000000)
  self.assertEqual((c,s,payload),(5,26,raw));self.assertEqual(seq,26)
  self.assertEqual(sock.target,('10.253.245.81',20180))
 def test_timer_wait_can_push_original_measurements_over_gate_deadline(self):
  now=10000000000;source=now-205000000
  raw=json.dumps(dict(status_ros_ns=now,source_stamp_ns=source))
  self.assertEqual(status_payload(raw,now+120000000+50000000,True)[1],'SOURCE_STALE')
  self.assertIsNone(status_payload(raw,now+40000000+5000000,True)[1])
 def test_safety_helper_byte_identical_to_success_backup(self):
  old=Path.home()/'robot_github_backup_20261113/leader_export/leader_diagnostics/nav2_follow_stability_da0ab05848da/convoy_source_time.py'
  self.assertEqual((ROOT/'leader/convoy_source_time.py').read_bytes(),old.read_bytes())
if __name__=='__main__':unittest.main()
