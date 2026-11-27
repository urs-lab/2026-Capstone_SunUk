import json,sys,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'candidate/follower_diagnostics'))
from continuous_local_core import ContinuousCore
from clearance_reobserve import ClearanceWait
from temporary_detour_candidate import Pose

class Tests(unittest.TestCase):
    def test_actual_recorded_cloud_still_stops_and_cannot_clear_wait(self):
        d=json.loads((ROOT/'fixtures/clearance_reobserve_snapshot_1794896827363546723.json').read_text());v=d['control']
        c=ContinuousCore(pose_source='filtered');c.install(d['plan'])
        c.scene=lambda *a:dict(pose=Pose(*v['pose']),points=np.array(d['points']),uncertainty=v['uncertainty_m'],source=v['source_stamp_ns'],scan_stamp=v['scan_stamp_ns'],source_stamps=v['input_source_stamps'])
        c.inputs['filtered']=(1,1.,(*v['pose'],v['wheel_diagnostic']['quantized_v'],v['wheel_diagnostic']['quantized_w']))
        c.last_curve=(.9,.025,0.)
        wait=ClearanceWait();wait.enter(1.)
        for i in range(100):
            if i:c.inputs['filtered']=(1,1.,(*v['pose'],0.,0.))
            row=c.command(2_000_000_000,1.+i*.1,{})
            if i==0:
                self.assertEqual(row['reason'],'STOP_SWEPT_CLEARANCE')
                self.assertAlmostEqual(row['swept_clearance_m'],v['swept_clearance_m'])
                self.assertEqual((row['v'],row['w']),(0.,0.))
            # Later shadow probes may be slower after a ramp timeout. None may
            # release the hold while the actual recorded obstacle points stay.
            self.assertFalse(wait.observe(row,1.+i*.1,dict(armed=True,feedback_fresh=True,measured_rpm=[0,0],zero_feedback_samples=5)))
        self.assertTrue(wait.active)

if __name__=='__main__':unittest.main()
