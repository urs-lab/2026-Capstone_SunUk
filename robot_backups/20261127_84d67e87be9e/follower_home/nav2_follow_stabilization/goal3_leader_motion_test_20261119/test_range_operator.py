import unittest
from test_reference_wait_operator import OperatorTests as Harness
class RangeOperatorTests(unittest.TestCase):
 def test_distinct_node_flag_and_actual_velocity_diagnostic(self):
  f,rows=Harness().flow(loss_end=0.,range_test=True,high_leader_yaw=True)
  self.assertEqual(f.created_node_name,'goal3_leader_motion_range_test')
  self.assertTrue(rows[-1]['leader_motion_range_test'])
  self.assertFalse(rows[-1]['leader_motion_range_check_enabled'])
  observed=[r for r in rows if r['event']=='leader_motion_range_observed']
  self.assertTrue(observed);self.assertTrue(all(r['angular_z_radps']==.12 for r in observed))
  self.assertTrue(all(not r['check_enabled'] for r in observed))
  self.assertNotEqual(rows[-1]['reason'],'LEADER_UNEXPECTED_MOTION')
 def test_reference_loss_still_zero_waits_and_resumes(self):
  f,rows=Harness().flow(range_test=True)
  begin=next(r['ros_ns'] for r in rows if r['event']=='reference_wait_started')
  end=next(r['ros_ns'] for r in rows if r['event']=='reference_wait_resumed')
  held=[e[1] for e in f.events if e[0]=='envelope' and begin<=e[1]['stamp_ns']<end]
  self.assertTrue(held);self.assertTrue(all(r['v']==r['w']==0 for r in held))
  self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
 def test_local_imu_fault_still_exits(self):
  f,rows=Harness().flow(range_test=True,local_fault=True)
  self.assertEqual(rows[-1]['reason'],'IMU_STALE')
if __name__=='__main__':unittest.main()
