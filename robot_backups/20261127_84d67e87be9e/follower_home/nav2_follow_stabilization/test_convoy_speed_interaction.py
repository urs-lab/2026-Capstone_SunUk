import unittest
from replay_convoy_speed_interaction import run


class InteractionTests(unittest.TestCase):
    def test_current_cores_toggle_at_small_gap_error_even_without_jitter(self):
        r=run(1.51)
        self.assertGreaterEqual(r['follower_drive_stop_edges'],15)
        moving=[x for x in r['rows'] if x['follower_v']>0]
        self.assertTrue(moving)
        self.assertTrue(all(x['leader_gate_v']<.01 for x in moving))

    def test_same_current_cores_can_remain_moving_at_larger_gap(self):
        r=run(1.70)
        self.assertEqual(r['follower_drive_stop_edges'],1)
        self.assertTrue(all(x['follower_v']>0 for x in r['rows'][1:]))

    def test_offline_floor_removes_this_counterexample_only(self):
        r=run(1.51,'floor')
        self.assertEqual(r['follower_drive_stop_edges'],1)
        self.assertTrue(all(x['follower_v']==.04 for x in r['rows'][1:]))
        self.assertTrue(all(x['leader_gate_v']<=.06 for x in r['rows']))

    def test_floor_does_not_drive_inside_existing_follow_stop_distance(self):
        r=run(.99,'floor')
        self.assertTrue(all(x['follower_v']==0 for x in r['rows']))
        self.assertTrue(all(x['leader_gate_v']==0 for x in r['rows']))


if __name__=='__main__':unittest.main()
