import unittest
from local_stop_turn_go import LocalCore
from continuous_local_core import ContinuousCore
from leader_motion_range import range_diagnostic
class LeaderMotionRangeTests(unittest.TestCase):
 def core(self,cls=LocalCore,v=.04,w=0.,enabled=True,point=(2.,0.)):
  c=cls();c.leader_motion_range_check_enabled=enabled;c.freeze((0.,0.,0.),(0.,0.,0.))
  stamp=100000000000
  c.raw(stamp,1.,(0.,0.,0.,0.,0.));c.inputs['leader']=(stamp,1.,(2.,0.,0.,v,w));c.offer_scan(stamp,1.,[point])
  return c
 def test_original_guard_still_default_and_raises_for_all_three_conditions(self):
  self.assertTrue(LocalCore().leader_motion_range_check_enabled)
  for v,w in ((.10001,0.),(-.01001,0.),(.04,.10001),(.04,-.10001)):
   with self.subTest(v=v,w=w):
    with self.assertRaisesRegex(ValueError,'LEADER_UNEXPECTED_MOTION'):self.core(v=v,w=w).scene(100050000000,1.05)
 def test_only_opt_in_test_allows_same_finite_speed_inputs(self):
  for v,w in ((.10001,0.),(-.01001,0.),(.04,.10001),(.04,-.10001)):
   c=self.core(v=v,w=w,enabled=False);self.assertIsNotNone(c.scene(100050000000,1.05))
 def test_stale_leader_and_scan_still_raise(self):
  c=self.core(enabled=False);c.inputs['leader']=(99000000000,0.,(2.,0.,0.,.5,.5))
  with self.assertRaisesRegex(ValueError,'LEADER_STALE'):c.scene(100050000000,1.05)
  c=self.core(enabled=False);c.scan=(99000000000,0.,c.scan[2])
  with self.assertRaisesRegex(ValueError,'SCAN_STALE'):c.scene(100050000000,1.05)
 def test_nonfinite_velocity_not_allowed_by_experiment(self):
  for v,w in ((float('nan'),0.),(.04,float('inf'))):
   with self.assertRaisesRegex(ValueError,'NONFINITE_LEADER_VELOCITY'):self.core(v=v,w=w,enabled=False).scene(100050000000,1.05)
 def test_diagnostic_names_exact_exceeded_fields(self):
  d=range_diagnostic((0,0,0,.2,-.3))
  self.assertEqual(d['exceeded'],['FORWARD_ABOVE_0_1','ABS_YAW_ABOVE_0_1'])
  self.assertEqual(d['linear_x_mps'],.2);self.assertEqual(d['angular_z_radps'],-.3)
  self.assertFalse(range_diagnostic((0,0,0,.1,.1))['outside_original_range'])
 def test_continuous_inheritance_keeps_follower_speed_and_clearance_limits(self):
  c=self.core(ContinuousCore,v=.2,w=.3,enabled=False)
  r=c.command(100050000000,1.05,{},follow_command=(.1,.1))
  self.assertLessEqual(r['v'],.025);self.assertLessEqual(abs(r['w']),.2)
  c=self.core(ContinuousCore,v=.2,w=.3,enabled=False,point=(.29,0.))
  r=c.command(100050000000,1.05,{},follow_command=(.025,0))
  self.assertEqual(r['reason'],'STOP_FOOTPRINT_CLEARANCE');self.assertEqual(r['v'],0.)
if __name__=='__main__':unittest.main()
