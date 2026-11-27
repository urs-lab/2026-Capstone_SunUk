import unittest
from test_follow_only_operator import Tests as Harness
class FirstPathWaitTests(unittest.TestCase):
 def flow(self,until):
  return Harness().auto_flow(extended=True,smooth=True,disable_clearance_stop=True,unlimited=True,goal_end_at=35.,first_path_until=until,leader_pause_until=2.)
 def test_singleton_after_leader_delay_remains_zero_until_goal_ends(self):
  f,code,rows=self.flow(1000.)
  self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
  self.assertTrue(any(r['event']=='follow_reference_wait' for r in rows))
  self.assertTrue(any(r['event']=='follow_reference_resumed' for r in rows))
  self.assertFalse(any(e[0]=='envelope' and e[1]['v']>0 for e in f.events))
  self.assertTrue(rows[-1]['follower_zero_confirmed'])
 def test_valid_second_point_after_recovery_enables_following(self):
  f,code,rows=self.flow(18.)
  self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
  self.assertTrue(any(r['event']=='follow_reference_resumed' for r in rows))
  moving=[e[1] for e in f.events if e[0]=='envelope' and e[1]['v']>0]
  self.assertTrue(moving)
  self.assertTrue(all(c['stamp_ns']>=118000000000 for c in moving))
if __name__=='__main__':unittest.main()
