import json,sys,unittest
from unittest.mock import patch
from test_local_avoidance_drive import Tests as Original
from follower_motor_candidate import velocity_packet
from test_follower_distance_trial import DistanceFlow
import run_follower_distance_trial as runner
class ReverseTests(unittest.TestCase):
 def contract(self):
  c,t,p,k=Original().contract(360);k.disarm();k.distance_test_only=True;k.core.reverse_distance_only=True;p.reverse_distance_only=True
  self.assertIsNotNone(k.arm(c.ros));c.now+=.01
  return c,t,p,k
 def test_reverse_wheels_and_encoder(self):
  c,t,p,k=self.contract()
  for _ in range(400):
   self.assertTrue(k.command(json.dumps(Original().envelope(c,k,'DRIVE',-.06,0.)),c.ros),k.last_command_rejection)
   k.tick(c.ros);c.now+=.01;self.assertIsNone(k.core.fault)
  self.assertGreater(t.rpm[0],0);self.assertEqual(t.rpm[0],-t.rpm[1]);self.assertLess(k.core.odom_x,-.1)
 def test_invalid_motion_and_old_input_rejected(self):
  for v,w,age in ((.04,0,0),(-.061,0,0),(-.06,.01,0),(-.06,0,.9)):
   c,t,p,k=self.contract();o=Original().envelope(c,k,'DRIVE',v,w);o['source_stamp_ns']-=int(age*1e9)
   self.assertFalse(k.command(json.dumps(o),c.ros));self.assertFalse(k.core.armed)
 def test_distance_backstop_and_command_deadline(self):
  c,t,p,k=self.contract();k.distance_travelled=5.51;k.tick(c.ros);self.assertEqual(k.core.fault,'REVERSE_DISTANCE_5_5M_LIMIT')
  c,t,p,k=self.contract();k.command(json.dumps(Original().envelope(c,k,'STOP',0,0)),c.ros)
  for _ in range(40):k.tick(c.ros);c.now+=.01
  self.assertEqual(k.core.fault,'COMMAND_TIMEOUT');self.assertEqual(t.rpm,(0,0))
 def test_port_blocks_forward(self):
  c,t,p,k=self.contract();p.motion_kind='DRIVE'
  with self.assertRaises(ValueError):p.write(velocity_packet(-3,3))
 def flow(self,stale=False,plan=False):
  f=DistanceFlow(stale)
  class Pump:
   def __init__(self,node):pass
   def spin_once(self):f.spin(f,.01)
   def close(self):pass
  def save(w,kind,extra):f.summary=extra
  with patch.dict(sys.modules,f.modules()),patch.dict('os.environ',{'ROS_DOMAIN_ID':'10'}),patch('sys.argv',['reverse','--plan-only' if plan else '--run','--ack-clear-path-power-cut-ready']),patch.object(runner,'attach',return_value=f.watch),patch.object(runner,'save',side_effect=save),patch.object(runner.time,'monotonic',side_effect=lambda:f.t),patch('leader_short_forward_trial.wait_for_start',return_value=True),patch('callback_pump.CallbackPump',Pump):code=runner.main()
  return f,code
 def test_full_five_metre_reverse(self):
  f,code=self.flow();self.assertEqual(code,0,f.summary);self.assertTrue(f.summary['follower_zero_confirmed'])
  self.assertAlmostEqual(f.summary['distance_sources']['raw']['forward_m'],-5,delta=.005)
  self.assertTrue(all(c['v'] in (0.,-.06) and c['w']==0 for c in f.commands));self.assertFalse(f.summary['leader_commanded'])
 def test_sensor_loss_disarms(self):
  f,code=self.flow(stale=True);self.assertEqual(code,1);self.assertFalse(f.armed);self.assertTrue(f.summary['follower_zero_confirmed'])
 def test_plan_no_arm(self):
  f,code=self.flow(plan=True);self.assertEqual(code,0);self.assertEqual(f.services,[]);self.assertEqual(f.commands,[])
if __name__=='__main__':unittest.main()
