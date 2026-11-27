import json,unittest
from test_local_avoidance_drive import Tests as Setup

class PolicyTests(unittest.TestCase):
    def test_three_cm_is_strict_in_both_measured_and_swept_clearance(self):
        setup=Setup()
        for field in ('clearance_m','swept_clearance_m'):
            for gap in (.029,.03,.030001,.04641841448321633,.075):
                with self.subTest(field=field,gap=gap):
                    clock,device,port,c=setup.contract();c.continuous20=True
                    obj=setup.envelope(clock,c,'DRIVE',.02,0.);obj[field]=gap
                    self.assertEqual(c.command(json.dumps(obj),clock.ros),gap>.03)
                    if gap<=.03:self.assertFalse(c.core.armed)
    def test_profile_reports_three_cm_and_rejects_old_mode_confusion(self):
        clock,device,port,c=Setup().contract();c.continuous20=True
        s=c.status();self.assertEqual(s['continuous_stop_clearance_m'],.03)
        self.assertEqual(s['continuous_clearance_profile'],'CONTINUOUS10_STOP03_WAIT_V1')
    def test_fresh_seven_cm_commands_can_drive_and_disarm_with_feedback(self):
        setup=Setup();clock,device,port,c=setup.contract();c.continuous20=True
        for _ in range(60):
            obj=setup.envelope(clock,c,'DRIVE',.02,0.)
            obj.update(clearance_m=.07,swept_clearance_m=.065)
            self.assertTrue(c.command(json.dumps(obj),clock.ros))
            c.tick(clock.ros);clock.now+=.02
            self.assertIsNone(c.core.fault)
        self.assertNotEqual(device.rpm,(0,0))
        c.disarm();c.tick(clock.ros)
        self.assertEqual(device.rpm,(0,0))
    def test_standard_profile_retains_thirty_cm(self):
        clock,device,port,c=Setup().contract()
        obj=Setup().envelope(clock,c,'DRIVE',.04,0.);obj['clearance_m']=.075
        self.assertFalse(c.command(json.dumps(obj),clock.ros))

if __name__=='__main__':unittest.main()
