import json,sys,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'candidate/follower_diagnostics'))
import continuous_local_core as core_module
from temporary_detour_candidate import Pose

class Tests(unittest.TestCase):
    def test_recorded_snapshot_stops_at_five_cm_and_passes_three_cm(self):
        data=json.loads((ROOT/'fixtures/stop03_recorded_clearance_snapshot_1794895061004562056.json').read_text());v=data['control']
        for threshold,reason in ((.05,'STOP_SWEPT_CLEARANCE'),(.03,'CONTINUOUS_READY')):
            c=core_module.ContinuousCore(pose_source='filtered');c.install(data['plan'])
            c.scene=lambda *a:dict(pose=Pose(*v['pose']),points=np.array(data['points']),uncertainty=v['uncertainty_m'],source=v['source_stamp_ns'],scan_stamp=v['scan_stamp_ns'],source_stamps=v['input_source_stamps'])
            c.inputs['filtered']=(1,1.,(*v['pose'],v['wheel_diagnostic']['quantized_v'],v['wheel_diagnostic']['quantized_w']))
            c.last_curve=(.9,.025,.0105)
            with patch.object(core_module,'STOP_CLEARANCE_M',threshold):row=c.command(2_000_000_000,1.,{})
            self.assertEqual(row['reason'],reason)
            self.assertAlmostEqual(row['swept_clearance_m'],v['swept_clearance_m'])
            if threshold==.03:self.assertGreater(row['v'],0)

if __name__=='__main__':unittest.main()
