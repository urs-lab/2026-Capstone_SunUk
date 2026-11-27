"""No ROS/hardware: real-core freshness, recorded gap, and full operator recovery."""
from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parent/'candidate/follower_diagnostics'))
from filtered_input_wait import filtered_stamp,FilteredUnavailable,fresh_zero_sources
from test_reference_wait_operator import OperatorTests
import test_continuous20 as fixture

class CoreTests(unittest.TestCase):
    def gap(self):
        c,ns=fixture.CoreTests().core();last=c.inputs['filtered'][0]
        now=ns+864_101_451;mono=1.864101451
        c.raw(now,mono,(0.,0.,0.,0.,0.));c.imu(now,mono,(0.,0.,0.,0.,0.));c.offer_scan(now,mono,[(5.,5.)])
        return c,last,now,mono
    def test_recorded_864ms_gap_reproduces_core_failure_but_zero_has_fresh_original_sources(self):
        c,last,now,mono=self.gap()
        with self.assertRaisesRegex(ValueError,'FILTERED_STALE'):c.scene(now,mono,require_leader=False)
        with self.assertRaises(FilteredUnavailable):filtered_stamp(c,now,mono)
        zero=fresh_zero_sources(c,now,mono)
        self.assertEqual(zero['source'],now);self.assertNotIn('filtered',zero['source_stamps'])
        self.assertEqual(c.inputs['filtered'][0],last)
        self.assertNotIn('pose',zero);self.assertNotIn('points',zero)
    def test_fresh_ekf_after_recorded_gap_restores_actual_local_scene(self):
        c,last,now,mono=self.gap();c.filtered(now,mono,(0.,0.,0.,0.,0.))
        self.assertEqual(filtered_stamp(c,now,mono),now)
        self.assertEqual(c.scene(now,mono,require_leader=False)['scan_stamp'],now)
    def test_bad_raw_imu_or_scan_cannot_support_zero_hold(self):
        for bad in ('raw','imu','scan'):
            c,last,now,mono=self.gap()
            if bad=='scan':c.scan=None
            else:c.inputs.pop(bad)
            with self.assertRaises(ValueError):fresh_zero_sources(c,now,mono)
    def test_future_filtered_timestamp_is_not_a_recoverable_stale_sample(self):
        c,last,now,mono=self.gap();c.filtered(now+1,mono,(0.,0.,0.,0.,0.))
        with self.assertRaisesRegex(ValueError,'FILTERED_FUTURE'):filtered_stamp(c,now,mono)

class OperatorTestsFiltered(unittest.TestCase):
    def flow(self,**kw):return OperatorTests().flow(loss_end=0.,**kw)
    def held_zero(self,f,rows):
        start=next(r for r in rows if r['event']=='filtered_input_wait_started')
        end=next(r for r in rows if r['event']=='filtered_input_wait_resumed')
        commands=[e[1] for e in f.events if e[0]=='envelope' and start['ros_ns']<=e[1]['stamp_ns']<end['ros_ns']]
        self.assertTrue(commands);self.assertTrue(all(r['motion']=='STOP' and r['v']==r['w']==0 for r in commands))
        self.assertTrue(any(e[0]=='envelope' and e[1]['stamp_ns']>end['ros_ns'] and e[1]['v']>0 for e in f.events))
        self.assertEqual(start['phase'],end['phase']);self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        return start,end
    def test_filtered_gap_during_actual_planning_phase_resumes_same_phase(self):
        from types import SimpleNamespace as S
        from unittest.mock import patch
        import run_local_avoidance_trial as runner
        pool=S(submit=lambda *a,**kw:S(done=lambda:False),shutdown=lambda **kw:None)
        with patch.object(runner,'ThreadPoolExecutor',return_value=pool):
            f,rows=self.flow(filtered_loss_end=20.,planning_wait=True)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        start=next(r for r in rows if r['event']=='filtered_input_wait_started')
        end=next(r for r in rows if r['event']=='filtered_input_wait_resumed')
        self.assertEqual(start['phase'],'PLANNING');self.assertEqual(end['phase'],'PLANNING')
        commands=[e[1] for e in f.events if e[0]=='envelope' and start['ros_ns']<=e[1]['stamp_ns']<end['ros_ns']]
        self.assertTrue(commands);self.assertTrue(all(r['v']==r['w']==0 for r in commands))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
    def test_recorded_864ms_gap_waits_then_resumes(self):
        f,rows=self.flow(filtered_loss_end=10.864101451);self.held_zero(f,rows)
    def test_twenty_second_gap_does_not_expire(self):
        f,rows=self.flow(filtered_loss_end=30.);start,end=self.held_zero(f,rows)
        self.assertGreater(end['waited_s'],19.)
    def test_permanent_gap_only_ends_at_operator_shutdown(self):
        f,rows=self.flow(filtered_loss_end=1000.)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertFalse(any(r['event']=='filtered_input_wait_resumed' for r in rows))
        self.assertTrue(rows[-1]['follower_zero_confirmed'])
    def test_reference_gap_and_filtered_gap_overlap_without_double_count(self):
        f,rows=OperatorTests().flow(loss_end=25.,filtered_loss_end=20.)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        start=next(r for r in rows if r['event']=='reference_wait_started')
        end=next(r for r in rows if r['event']=='reference_wait_resumed')
        summed=rows[-1]['reference_wait_total_s']+rows[-1]['filtered_input_wait_total_s']
        self.assertAlmostEqual(summed,(end['ros_ns']-start['ros_ns'])/1e9,delta=.1)
    def test_motor_gap_and_filtered_gap_overlap_in_zero_helper(self):
        f,rows=self.flow(status_loss_end=14.,filtered_loss_end=20.);self.held_zero(f,rows)
        self.assertEqual(rows[-1]['motor_status_wait_episodes'],1)
    def test_new_filtered_samples_do_not_resume_before_scan_alignment(self):
        f,rows=self.flow(filtered_loss_end=20.,alignment_end=22.)
        start,end=self.held_zero(f,rows)
        self.assertGreater(end['waited_s'],12.)
    def test_imu_fault_during_filtered_wait_is_not_hidden(self):
        f,rows=self.flow(filtered_loss_end=1000.,imu_loss_end=1000.)
        self.assertEqual(rows[-1]['reason'],'IMU_STALE')
    def test_raw_loss_during_filtered_wait_holds_zero_without_ending_session(self):
        f,rows=self.flow(filtered_loss_end=1000.,raw_loss_end=1000.)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertTrue(any(r['event']=='raw_input_wait_started' for r in rows))
        self.assertFalse(any(r['event']=='raw_input_wait_resumed' for r in rows))
    def test_motor_fault_during_filtered_wait_is_not_hidden(self):
        f,rows=self.flow(filtered_loss_end=1000.,status_loss_end=14.,status_return='fault')
        self.assertEqual(rows[-1]['reason'],'MOTOR_FAULT_OR_FEEDBACK_STALE')
    def test_old_filtered_stamps_keep_waiting(self):
        f,rows=self.flow(filtered_loss_end=14.,filtered_return='stale_stamp')
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertFalse(any(r['event']=='filtered_input_wait_resumed' for r in rows))
    def test_future_filtered_stamps_do_not_resume(self):
        f,rows=self.flow(filtered_loss_end=14.,filtered_return='future')
        self.assertEqual(rows[-1]['reason'],'FILTERED_FUTURE')
    def test_invalid_filtered_measurements_do_not_resume(self):
        f,rows=self.flow(filtered_loss_end=14.,filtered_return='invalid_velocity')
        self.assertEqual(rows[-1]['reason'],'FILTERED_MISSING')

if __name__=='__main__':unittest.main()
