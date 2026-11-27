"""Real operator execution with fake ROS: no hardware or sockets."""
import unittest
from test_reference_wait_operator import OperatorTests

class Tests(unittest.TestCase):
    def flow(self,**kw):return OperatorTests().flow(loss_end=0.,**kw)
    def test_status_gap_waits_zero_then_resumes_without_rearm(self):
        f,rows=self.flow(status_loss_end=14.)
        start=next(r for r in rows if r['event']=='motor_status_wait_started')
        resumed=next(r for r in rows if r['event']=='motor_status_wait_resumed')
        self.assertGreater(resumed['waited_s'],3.)
        held=[e[1] for e in f.events if e[0]=='envelope' and start['ros_ns']<=e[1]['stamp_ns']<resumed['ros_ns']]
        self.assertTrue(held);self.assertTrue(all(r['motion']=='STOP' and r['v']==r['w']==0 for r in held))
        self.assertTrue(any(e[0]=='envelope' and e[1]['stamp_ns']>resumed['ros_ns'] and e[1]['v']>0 for e in f.events))
        self.assertEqual(start['phase'],resumed['phase'])
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertEqual(rows[-1]['motor_status_wait_episodes'],1)
    def test_permanent_status_loss_waits_until_operator_shutdown(self):
        f,rows=self.flow(status_loss_end=1000.)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertFalse(any(r['event']=='motor_status_wait_resumed' for r in rows))
        self.assertTrue(rows[-1]['follower_zero_confirmed'])
    def test_driver_fault_is_not_automatically_cleared(self):
        f,rows=self.flow(status_loss_end=14.,status_return='fault')
        self.assertEqual(rows[-1]['reason'],'MOTOR_FAULT_OR_FEEDBACK_STALE')
        self.assertFalse(any(r['event']=='motor_status_wait_resumed' for r in rows))
    def test_restarted_driver_is_not_automatically_rearmed(self):
        f,rows=self.flow(status_loss_end=14.,status_return='restart')
        self.assertEqual(rows[-1]['reason'],'SESSION_CHANGED')
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
    def test_returned_disarmed_status_does_not_resume(self):
        f,rows=self.flow(status_loss_end=14.,status_return='disarmed')
        self.assertEqual(rows[-1]['reason'],'UNEXPECTED_DISARM')
    def test_replayed_old_status_does_not_resume(self):
        f,rows=self.flow(status_loss_end=14.,status_return='stale_stamp')
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertFalse(any(r['event']=='motor_status_wait_resumed' for r in rows))
    def test_local_imu_fault_still_exits_during_status_wait(self):
        f,rows=self.flow(status_loss_end=1000.,local_fault=True)
        self.assertEqual(rows[-1]['reason'],'IMU_STALE')
    def test_overlapping_reference_wait_does_not_double_count(self):
        f,rows=OperatorTests().flow(loss_end=25.,status_loss_end=14.)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        first=next(r for r in rows if r['event']=='reference_wait_started')
        end=next(r for r in rows if r['event']=='reference_wait_resumed')
        total=rows[-1]['reference_wait_total_s']+rows[-1]['motor_status_wait_total_s']
        self.assertAlmostEqual(total,(end['ros_ns']-first['ros_ns'])/1e9,delta=.1)

if __name__=='__main__':unittest.main()

class UnitTests(unittest.TestCase):
    def test_frozen_feedback_never_releases_wait(self):
        from motor_status_wait import MotorStatusWait
        w=MotorStatusWait();w.enter(1.)
        for i in range(20):
            now=2_000_000_000+i*100_000_000
            self.assertFalse(w.ready_status(now,1.+i*.1,dict(status_ros_ns=now,valid_feedback=7,measured_rpm=[0,0],zero_feedback_samples=5)))
    def test_old_or_future_status_never_releases_wait(self):
        from motor_status_wait import MotorStatusWait
        for offset in (-1_000_000_000,1_000_000_000):
            w=MotorStatusWait();w.enter(1.)
            for i in range(20):
                now=2_000_000_000+i*100_000_000
                self.assertFalse(w.ready_status(now,1.+i*.1,dict(status_ros_ns=now+offset,valid_feedback=i+1,measured_rpm=[0,0],zero_feedback_samples=5)))
    def test_nonzero_rpm_never_releases_wait(self):
        from motor_status_wait import MotorStatusWait
        w=MotorStatusWait();w.enter(1.)
        for i in range(20):
            now=2_000_000_000+i*100_000_000
            self.assertFalse(w.ready_status(now,1.+i*.1,dict(status_ros_ns=now,valid_feedback=i+1,measured_rpm=[1,0],zero_feedback_samples=0)))
