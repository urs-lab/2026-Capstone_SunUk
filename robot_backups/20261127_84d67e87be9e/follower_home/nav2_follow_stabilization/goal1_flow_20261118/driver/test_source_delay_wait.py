import json,unittest
from local_avoidance_drive import LocalAvoidanceContract,PivotDrivePort,PROFILE
from follower_motor_mock import MemoryTransport
from test_motor_candidate import Clock
class SourceDelayWaitTests(unittest.TestCase):
 def setup(self,enabled=True):
  c=Clock();t=MemoryTransport();p=PivotDrivePort(t,c,max_rpm=15)
  k=LocalAvoidanceContract(p,enabled=True,clock=c,max_rpm=15,trial_seconds=0,extended_follow=True)
  k.source_delay_wait_enabled=enabled;k.clearance_stop_enabled=False
  for _ in range(75):k.tick(c.ros);c.now+=.01
  self.assertIsNotNone(k.arm(c.ros));c.now+=.01
  for _ in range(130):
   self.assertTrue(k.command(self.msg(c,k),c.ros));k.tick(c.ros);c.now+=.01
  return c,t,k
 def msg(self,c,k,v=0.,age=0.,**extra):
  obj=dict(version=3,controller=PROFILE,epoch=k.core.epoch,frame='follower/base_link',stamp_ns=c.ros,
   source_stamp_ns=c.ros-round(age*1e9),scan_stamp_ns=c.ros,motion='DRIVE' if v else 'STOP',
   v=v,w=0.,clearance_m=0.,swept_clearance_m=0.)
  obj.update(extra);return json.dumps(obj)
 def delayed(self,c,k,**extra):
  s=self.msg(c,k,v=.04,age=.79,**extra);c.now+=.02
  return s
 def test_source_expiry_in_transit_holds_zero_and_resumes_same_epoch(self):
  c,t,k=self.setup();epoch=k.core.epoch
  self.assertTrue(k.command(self.delayed(c,k),c.ros),k.last_command_rejection)
  self.assertTrue(k.source_delay_wait_active);self.assertEqual(k.core.command[:2],(0.,0.));self.assertIsNone(k.core.fault)
  for _ in range(80):
   c.now+=.01;self.assertTrue(k.command(self.msg(c,k,v=.04),c.ros));k.tick(c.ros)
   self.assertEqual(k.core.command[:2],(0.,0.));self.assertTrue(k.source_delay_wait_active)
  self.assertEqual(t.rpm,(0,0));self.assertGreaterEqual(k.zero_samples,3)
  c.now+=.01;self.assertTrue(k.command(self.msg(c,k),c.ros));self.assertFalse(k.source_delay_wait_active)
  for _ in range(60):
   c.now+=.01;self.assertTrue(k.command(self.msg(c,k,v=.04),c.ros));k.tick(c.ros)
  self.assertEqual(k.core.epoch,epoch);self.assertTrue(k.core.armed);self.assertIsNone(k.core.fault)
  self.assertLess(t.rpm[0],0);self.assertGreater(t.rpm[1],0)
 def test_default_original_profile_still_rejects(self):
  c,t,k=self.setup(False);self.assertFalse(k.command(self.delayed(c,k),c.ros));self.assertFalse(k.core.armed)
 def test_future_prearm_epoch_expired_header_already_old_and_overspeed_fault(self):
  for kind in ('future','prearm','epoch','header','already_old','overspeed'):
   with self.subTest(kind=kind):
    c,t,k=self.setup();o=json.loads(self.delayed(c,k))
    if kind=='future':o['source_stamp_ns']=c.ros+1
    if kind=='prearm':o['source_stamp_ns']=k.core.arm_ros_ns-1
    if kind=='epoch':o['epoch']='0'*16
    if kind=='header':o['stamp_ns']=c.ros-310000000
    if kind=='already_old':o['source_stamp_ns']=o['stamp_ns']-810000000
    if kind=='overspeed':o['v']=.11
    self.assertFalse(k.command(json.dumps(o),c.ros));self.assertFalse(k.core.armed)
 def test_missing_commands_in_hold_still_timeout(self):
  c,t,k=self.setup();k.command(self.delayed(c,k),c.ros)
  for _ in range(45):k.tick(c.ros);c.now+=.01
  self.assertEqual(k.core.fault,'COMMAND_TIMEOUT');self.assertEqual(t.rpm,(0,0))
 def test_feedback_loss_still_faults(self):
  c,t,k=self.setup();k.command(self.delayed(c,k),c.ros);t.feedback_enabled=False
  for _ in range(50):
   c.now+=.01
   if k.core.armed:k.command(self.msg(c,k),c.ros)
   k.tick(c.ros)
  self.assertIsNotNone(k.core.fault);self.assertFalse(k.core.armed)
if __name__=='__main__':unittest.main()
