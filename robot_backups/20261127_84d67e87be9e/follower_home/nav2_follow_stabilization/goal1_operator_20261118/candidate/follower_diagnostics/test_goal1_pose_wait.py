"""Actual operator flow with fake ROS and clocks; no robot operations."""
import unittest
from test_follow_only_operator import Tests as FlowTests
from follow_reference_recovery import FollowReferenceRecovery
from follow_pose_recovery import recoverable_pose_delay
from types import SimpleNamespace as S


class PoseWaitTests(unittest.TestCase):
    def flow(self,**kwargs):
        return FlowTests().auto_flow(extended=True,disable_clearance_stop=True,unlimited=True,goal_end_at=35.,**kwargs)

    def test_delay_longer_than_ten_seconds_holds_zero_and_resumes_same_arm(self):
        f,code,rows=self.flow(pose_delay_until=14.)
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        waits=[r for r in rows if r['event']=='follow_reference_wait']
        resumed=next(r for r in rows if r['event']=='follow_reference_resumed')
        self.assertTrue(waits)
        begin=waits[0]['ros_ns'];end=resumed['ros_ns']
        self.assertGreater((end-begin)/1e9,10.)
        held=[e[1] for e in f.events if e[0]=='envelope' and begin<=e[1]['stamp_ns']<end]
        self.assertTrue(held);self.assertTrue(all(x['motion']=='STOP' and x['v']==x['w']==0. for x in held))
        self.assertTrue(any(e[0]=='envelope' and e[1]['stamp_ns']>end and e[1]['v']>0. for e in f.events))
        self.assertEqual(rows[-1]['reference_recovery_timeout_s'],0.)
        self.assertTrue(rows[-1]['leader_zero_confirmed']);self.assertTrue(rows[-1]['follower_zero_confirmed'])

    def test_permanent_pose_delay_stays_zero_until_goal_finishes(self):
        f,code,rows=self.flow(pose_delay_until=1000.)
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        self.assertFalse(any(r['event']=='follow_reference_resumed' for r in rows))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)

    def test_future_pose_is_not_turned_into_a_recoverable_delay(self):
        f,code,rows=self.flow(pose_delay_until=14.,pose_future=True)
        self.assertEqual(rows[-1]['reason'],'FOLLOW_POSE_STALE')
        self.assertFalse(any(r['event']=='follow_reference_wait' for r in rows))

    def test_raw_loss_during_pose_wait_still_stops(self):
        f,code,rows=self.flow(pose_delay_until=14.,raw_loss=True)
        self.assertEqual(rows[-1]['reason'],'FOLLOW_ZERO_RAW_STALE')
        self.assertTrue(rows[-1]['follower_zero_confirmed'])

    def test_real_motor_fault_still_terminates(self):
        f,code,rows=self.flow(pose_delay_until=14.,motor_fault_during_wait=True)
        self.assertEqual(rows[-1]['reason'],'MOTOR_FAULT_OR_FEEDBACK_STALE')

    def test_missing_pose_only_waits_for_a_valid_past_time_rejection(self):
        self.assertTrue(recoverable_pose_delay('FOLLOW_MAP_POSE_MISSING',None,10_000_000_000,10.,
            dict(key='follower',reason='INVALID_SOURCE_TIME',source_ns=9_000_000_000)))
        self.assertFalse(recoverable_pose_delay('FOLLOW_MAP_POSE_MISSING',None,10_000_000_000,10.,
            dict(key='follower',reason='INVALID_GEOMETRY_OR_FRAME',source_ns=9_000_000_000)))
        self.assertFalse(recoverable_pose_delay('FOLLOW_POSE_SCAN_TIME_MISMATCH',
            (S(sec=11,nanosec=0),9.,()),10_000_000_000,10.))

    def test_only_unlimited_profile_disables_recovery_timeout(self):
        finite=FollowReferenceRecovery();finite.wait(1.)
        with self.assertRaisesRegex(ValueError,'RECOVERY_TIMEOUT'):finite.wait(11.)
        unlimited=FollowReferenceRecovery(timeout_s=0.);unlimited.wait(1.);unlimited.wait(10001.)
        self.assertTrue(unlimited.active)

    def test_actual_smooth_direct_unicast_options_recover_then_drive(self):
        f,code,rows=self.flow(pose_delay_until=14.,smooth=True,direct_unicast=True)
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        resumed=next(r for r in rows if r['event']=='follow_reference_resumed')
        self.assertTrue(any(e[0]=='envelope' and e[1]['stamp_ns']>resumed['ros_ns'] and e[1]['v']>0. for e in f.events))
        self.assertTrue(rows[-1]['smooth_follow']);self.assertTrue(rows[-1]['direct_unicast'])
        self.assertTrue(rows[-1]['isolated_status']);self.assertTrue(f.status_process_closed)

    def test_motor_status_receive_delay_and_replayed_status_both_hold_then_recover(self):
        for replayed in (False,True):
            with self.subTest(replayed=replayed):
                f,code,rows=self.flow(status_delay_until=14.,status_replayed=replayed)
                self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
                resumed=next(r for r in rows if r['event']=='follow_reference_resumed')
                wait=next(r for r in rows if r['event']=='follow_reference_wait')
                held=[e[1] for e in f.events if e[0]=='envelope' and wait['ros_ns']<=e[1]['stamp_ns']<resumed['ros_ns']]
                self.assertTrue(held);self.assertTrue(all(x['motion']=='STOP' and x['v']==x['w']==0. for x in held))
                self.assertEqual(sum(e[0]=='arm' for e in f.events),1)

    def test_new_epoch_after_status_wait_is_terminal(self):
        f,code,rows=self.flow(status_delay_until=14.,status_epoch_changed=True)
        self.assertEqual(rows[-1]['reason'],'SESSION_CHANGED')

    def test_alignment_delay_immediately_before_arm_stays_disarmed_then_starts(self):
        f,code,rows=self.flow(startup_delay_until=14.)
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        resumed=next(r for r in rows if r['event']=='startup_pose_wait_resumed')
        arm=next(r for r in rows if r['event']=='follower_arm')
        self.assertGreaterEqual(arm['ros_ns'],resumed['ros_ns'])
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(rows[-1]['startup_pose_wait_episodes'],1)

    def test_permanent_prearm_alignment_loss_never_arms(self):
        f,code,rows=self.flow(startup_delay_until=1000.)
        self.assertEqual(rows[-1]['reason'],'NAV2_GOAL_FINISHED')
        self.assertFalse(any(e[0]=='arm' for e in f.events))


if __name__=='__main__':unittest.main()
