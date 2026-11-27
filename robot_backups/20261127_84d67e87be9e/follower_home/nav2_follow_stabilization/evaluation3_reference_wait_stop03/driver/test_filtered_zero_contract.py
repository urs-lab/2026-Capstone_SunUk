import json,sys,unittest
from pathlib import Path
from types import SimpleNamespace as S
from test_local_avoidance_drive import Tests as Setup
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'candidate/follower_diagnostics'))
from filtered_input_wait import fresh_zero_sources

class Tests(unittest.TestCase):
    def test_fresh_independent_stop_sources_hold_actual_contract_then_same_epoch_resumes(self):
        clock,device,port,contract=Setup().contract(seconds=0);contract.continuous20=True
        epoch=contract.core.epoch;filtered_original=clock.ros
        for _ in range(400):
            source=clock.ros
            core=S(inputs={'raw':(source,clock.now,(0,0,0,0,0)),
                           'imu':(source,clock.now,(0,0,0,0,0)),
                           'filtered':(filtered_original,0.,(0,0,0,0,0))},
                   scan=(source,clock.now,[(5.,5.)]))
            zero=fresh_zero_sources(core,clock.ros,clock.now)
            obj=Setup().envelope(clock,contract,'STOP',0.,0.)
            obj.update(source_stamp_ns=zero['source'],scan_stamp_ns=zero['scan_stamp'],clearance_m=0.,swept_clearance_m=0.)
            self.assertTrue(contract.command(json.dumps(obj),clock.ros),contract.last_command_rejection)
            contract.tick(clock.ros);clock.now+=.05
            self.assertEqual(device.rpm,(0,0));self.assertIsNone(contract.core.fault)
            self.assertEqual(core.inputs['filtered'][0],filtered_original)
        self.assertEqual(contract.core.epoch,epoch);self.assertTrue(contract.core.armed)
        for _ in range(20):
            obj=Setup().envelope(clock,contract,'DRIVE',.02,0.)
            self.assertTrue(contract.command(json.dumps(obj),clock.ros));contract.tick(clock.ros);clock.now+=.02
        self.assertNotEqual(device.rpm,(0,0));contract.disarm();contract.tick(clock.ros);self.assertEqual(device.rpm,(0,0))

if __name__=='__main__':unittest.main()
