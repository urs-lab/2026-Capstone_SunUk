import unittest
from test_distance_runtime import DistanceTests
class ReverseNineTests(unittest.TestCase):
 def runflow(self,**kw):return DistanceTests().run_flow(direction='reverse',speed=.5,operator_clear_path=True,**kw)
 def test_nine_metres_each_new_origin_with_ramp_and_braking(self):
  for origin in (0.,-9.,30.):
   f,code,r=self.runflow(initial_odom_x=origin)
   self.assertEqual(code,0,r);self.assertTrue(r['leader_zero_confirmed'])
   self.assertAlmostEqual(origin-f.x,9.,delta=.005)
   self.assertTrue(all(-.5<=v<=0 for v in f.commands));self.assertAlmostEqual(min(f.commands),-.5)
   self.assertGreater(f.commands[0],-.02);self.assertEqual(f.commands[-1],0)
   peak=f.commands.index(min(f.commands));self.assertTrue(any(-.1<v<0 for v in f.commands[peak:]))
   self.assertEqual(r['distance_time_limit_s'],42.);self.assertFalse(r['follower_commanded'])
 def test_other_command_writer_prevents_motion(self):
  f,code,r=self.runflow(duplicate=True);self.assertEqual(code,1);self.assertFalse(any(f.commands))
 def test_sensor_loss_and_ctrl_c_send_zero(self):
  for kw in ({'stale':True},{'interrupt':True}):
   f,code,r=self.runflow(**kw);self.assertEqual(code,1);self.assertEqual(f.commands[-1],0)
   self.assertEqual(r['leader_zero_confirmed'],not kw.get('stale',False))
if __name__=='__main__':unittest.main()
