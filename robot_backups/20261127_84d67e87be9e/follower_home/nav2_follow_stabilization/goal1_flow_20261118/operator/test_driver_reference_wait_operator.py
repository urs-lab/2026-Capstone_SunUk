import unittest
from test_follow_only_operator import Tests as Harness
class DriverReferenceWaitOperatorTests(unittest.TestCase):
 def test_driver_hold_publishes_zero_then_stable_recovery_same_arm(self):
  f,code,rows=Harness().auto_flow(extended=True,smooth=True,disable_clearance_stop=True,unlimited=True,
   goal_end_at=30.,driver_source_wait_until=3.)
  self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
  begin=next(r['ros_ns'] for r in rows if r['event']=='follow_reference_wait' and r['wait_reason']=='DRIVER_REFERENCE_DELAY_WAIT')
  end=next(r['ros_ns'] for r in rows if r['event']=='follow_reference_resumed' and r['ros_ns']>begin)
  held=[e[1] for e in f.events if e[0]=='envelope' and begin<=e[1]['stamp_ns']<end]
  self.assertTrue(held);self.assertTrue(all(c['v']==c['w']==0 for c in held))
  self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
  self.assertTrue(any(e[0]=='envelope' and e[1]['v']>0 and e[1]['stamp_ns']>end for e in f.events))
  self.assertTrue(rows[-1]['follower_zero_confirmed'])
if __name__=='__main__':unittest.main()
