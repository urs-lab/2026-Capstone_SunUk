import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent/'candidate/follower_diagnostics'))
from clearance_reobserve import ClearanceWait
from test_reference_wait_operator import OperatorTests

class UnitTests(unittest.TestCase):
    def state(self):return dict(armed=True,feedback_fresh=True,measured_rpm=[0,0],zero_feedback_samples=5)
    def test_persistent_recorded_278cm_sweep_keeps_waiting_without_expiry(self):
        w=ClearanceWait();w.enter(1.)
        for t in (1.,2.,10.,100.,1000.):
            row=dict(reason='STOP_SWEPT_CLEARANCE',clearance_m=.06822859783714987,swept_clearance_m=.027796300385449276,scan_stamp_ns=int(t*1e9))
            self.assertFalse(w.observe(row,t,self.state()))
        self.assertTrue(w.active);self.assertEqual(w.episodes,1)
    def test_requires_clear_scan_progress_stability_and_zero_feedback(self):
        w=ClearanceWait();w.enter(1.)
        row=dict(reason='CONTINUOUS_READY',clearance_m=.30,swept_clearance_m=.29,scan_stamp_ns=1)
        for i in range(20):self.assertFalse(w.observe(row,1.+i*.1,self.state()))
        for i in range(20):
            row['scan_stamp_ns']=100+i
            self.assertFalse(w.observe(row,3.+i*.1,dict(self.state(),measured_rpm=[1,0])))
        ready=False
        for i in range(15):
            row['scan_stamp_ns']=200+i;ready=w.observe(row,5.+i*.1,self.state())
        self.assertTrue(ready)
    def test_stop_threshold_and_resume_hysteresis_are_separate(self):
        w=ClearanceWait();w.enter(1.)
        self.assertAlmostEqual(w.resume_clearance_m,.05)
        for i in range(20):
            self.assertFalse(w.observe(dict(reason='CONTINUOUS_READY',clearance_m=.06,swept_clearance_m=.04,scan_stamp_ns=i+1),1.+i*.1,self.state()))
        self.assertFalse(w.accepts(dict(reason='STOP_FILTERED_ODOM_JUMP')))

class FlowTests(unittest.TestCase):
    def flow(self,**kw):return OperatorTests().flow(loss_end=0.,**kw)
    def check_hold(self,f,rows):
        start=next(r for r in rows if r['event']=='clearance_wait_started')
        end=next(r for r in rows if r['event']=='clearance_wait_resumed')
        commands=[e[1] for e in f.events if e[0]=='envelope' and start['ros_ns']<=e[1]['stamp_ns']<=end['ros_ns']]
        self.assertTrue(commands);self.assertTrue(all(r['motion']=='STOP' and r['v']==r['w']==0 for r in commands))
        self.assertTrue(any(e[0]=='envelope' and e[1]['stamp_ns']>end['ros_ns'] and e[1]['v']>0 for e in f.events))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(start['phase'],end['phase']);self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
    def test_one_scan_sweep_spike_waits_and_resumes_same_arm(self):
        self.check_hold(*self.flow(clearance_end=10.12))
    def test_footprint_stop_also_reobserves_without_legacy_two_second_abort(self):
        self.check_hold(*self.flow(clearance_end=14.,clearance_kind='footprint'))
    def test_persistent_sweep_never_resumes_or_times_out(self):
        f,rows=self.flow(clearance_end=1000.)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertFalse(any(r['event']=='clearance_wait_resumed' for r in rows))
        first=next(r for r in rows if r['event']=='clearance_wait_started')
        self.assertTrue(all(e[1]['v']==e[1]['w']==0 for e in f.events if e[0]=='envelope' and e[1]['stamp_ns']>=first['ros_ns']))
    def test_repeated_same_scan_never_resumes(self):
        f,rows=self.flow(clearance_end=10.12,clearance_return='frozen_scan')
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertFalse(any(r['event']=='clearance_wait_resumed' for r in rows))
    def test_marginal_gap_does_not_chatter_back_to_drive(self):
        f,rows=self.flow(clearance_end=10.12,clearance_return='marginal')
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertFalse(any(r['event']=='clearance_wait_resumed' for r in rows))
    def test_filtered_loss_overlapping_clearance_wait_still_recovers(self):
        self.check_hold(*self.flow(clearance_end=14.,filtered_loss_end=20.))
    def test_driver_fault_during_clearance_wait_still_aborts(self):
        f,rows=self.flow(clearance_end=1000.,status_loss_end=14.,status_return='fault')
        self.assertEqual(rows[-1]['reason'],'MOTOR_FAULT_OR_FEEDBACK_STALE')
    def test_unrelated_geometry_or_pose_fault_still_aborts(self):
        f,rows=self.flow(clearance_end=14.,clearance_return='fault')
        self.assertEqual(rows[-1]['reason'],'STOP_FILTERED_ODOM_JUMP')

if __name__=='__main__':unittest.main()
