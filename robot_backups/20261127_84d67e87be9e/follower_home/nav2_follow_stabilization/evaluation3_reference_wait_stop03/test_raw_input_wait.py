import unittest
from test_reference_wait_operator import OperatorTests


class RawWaitTests(unittest.TestCase):
    flow=OperatorTests.flow

    def test_isolated_status_entrypoint_still_drives_and_closes_display_process(self):
        f,rows=self.flow(loss_end=0.,isolated_status=True)
        self.assertTrue(rows[-1]['isolated_status'])
        self.assertTrue(f.status_process_closed)
        self.assertTrue(any(e[0]=='envelope' and e[1]['v']>0 for e in f.events))

    def test_raw_delay_waits_zero_then_resumes_without_rearm(self):
        f,rows=self.flow(loss_end=0.,raw_loss_end=30.)
        start=next(r for r in rows if r['event']=='raw_input_wait_started')
        end=next(r for r in rows if r['event']=='raw_input_wait_resumed')
        self.assertGreater(end['waited_s'],19.)
        held=[e[1] for e in f.events if e[0]=='envelope' and start['ros_ns']<=e[1]['stamp_ns']<end['ros_ns']]
        self.assertTrue(held)
        self.assertTrue(all(r['motion']=='STOP' and r['v']==r['w']==0 for r in held))
        self.assertTrue(any(e[0]=='envelope' and e[1]['stamp_ns']>end['ros_ns'] and e[1]['v']>0 for e in f.events))
        self.assertEqual(sum(e[0]=='arm' for e in f.events),1)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')

    def test_permanent_loss_never_releases_hold(self):
        f,rows=self.flow(loss_end=0.,raw_loss_end=1000.)
        self.assertEqual(rows[-1]['reason'],'ROS_SHUTDOWN')
        self.assertFalse(any(r['event']=='raw_input_wait_resumed' for r in rows))
        self.assertTrue(rows[-1]['follower_zero_confirmed'])

    def test_recovered_raw_cannot_bypass_filtered_wait(self):
        f,rows=self.flow(loss_end=0.,raw_loss_end=20.,filtered_loss_end=30.)
        raw_end=next(r for r in rows if r['event']=='raw_input_wait_resumed')
        filtered_end=next(r for r in rows if r['event']=='filtered_input_wait_resumed')
        self.assertGreater(filtered_end['ros_ns'],raw_end['ros_ns'])
        held=[e[1] for e in f.events if e[0]=='envelope' and raw_end['ros_ns']<=e[1]['stamp_ns']<filtered_end['ros_ns']]
        self.assertTrue(all(r['motion']=='STOP' for r in held))


if __name__=='__main__':unittest.main()
