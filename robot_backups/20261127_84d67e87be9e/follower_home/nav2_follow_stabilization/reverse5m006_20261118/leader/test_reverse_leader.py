import unittest
from test_distance_runtime import DistanceTests
class ReverseLeaderTests(unittest.TestCase):
 def test_reverse_five_metres_from_each_origin(self):
  for origin in (0.,-5.,24.):
   f,code,r=DistanceTests().run_flow(direction='reverse',speed=.06,initial_odom_x=origin,operator_clear_path=True)
   self.assertEqual(code,0,r);self.assertTrue(r['leader_zero_confirmed']);self.assertAlmostEqual(origin-f.x,5.,delta=.005)
   self.assertTrue(all(v in (0.,-.06) for v in f.commands));self.assertFalse(r['follower_commanded'])
 def test_other_writer_blocks(self):
  f,code,r=DistanceTests().run_flow(direction='reverse',speed=.06,operator_clear_path=True,duplicate=True)
  self.assertEqual(code,1);self.assertFalse(any(f.commands))
 def test_sensor_loss_and_interrupt_zero(self):
  for kw in ({'stale':True},{'interrupt':True}):
   f,code,r=DistanceTests().run_flow(direction='reverse',speed=.06,operator_clear_path=True,**kw)
   self.assertEqual(code,1);self.assertEqual(f.commands[-1],0.)
   self.assertEqual(r['leader_zero_confirmed'],not kw.get('stale',False))
if __name__=='__main__':unittest.main()
