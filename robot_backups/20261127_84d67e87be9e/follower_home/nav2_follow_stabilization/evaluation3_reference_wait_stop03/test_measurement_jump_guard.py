import json
import math
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'candidate/follower_diagnostics'))
from continuous_local_core import ContinuousCore
from temporary_detour_candidate import Pose


class Tests(unittest.TestCase):
    def core(self,pose=(0.,0.,0.)):
        c=ContinuousCore(pose_source='filtered')
        c.filtered(1_000_000_000,1.,(*pose,.025,0.))
        c.last_pose=pose
        c.scene=lambda *args:dict(pose=Pose(*c.inputs['filtered'][2][:3]),
            points=np.array([[10.,10.]]),uncertainty=.01,
            source=c.inputs['filtered'][0],scan_stamp=c.inputs['filtered'][0],source_stamps={})
        return c

    def command(self,c):
        c.last_curve=(9.9,.025,0.)
        return c.command(10_000_000_000,10.,{},follow_command=(.025,0.))

    def test_wait_accumulated_rotation_is_not_a_single_measurement_jump(self):
        evidence=json.loads((ROOT/'measurement_jump_evidence_20261117.json').read_text())
        a=evidence['last_control']['pose'];b=evidence['stop_control']['pose']
        self.assertGreater(abs(b[2]-a[2]),.12)
        c=self.core(tuple(a))
        # Intermediate measurements are synthetic: the original log contains
        # only control endpoints. This reproduces the control-gap defect,
        # not an assertion about unrecorded live EKF samples.
        for i in range(1,41):
            pose=tuple(x+(y-x)*i/40 for x,y in zip(a,b))
            c.filtered(1_000_000_000+i*100_000_000,1+i*.1,(*pose,.025,0.))
        self.assertIsNone(c.filtered_jump)
        self.assertEqual(c.last_pose,tuple(a))
        self.assertEqual(self.command(c)['reason'],'CONTINUOUS_READY')

    def test_abrupt_translation_and_rotation_still_stop(self):
        for pose in ((.101,0.,0.),(0.,0.,.121)):
            with self.subTest(pose=pose):
                c=self.core();c.filtered(1_100_000_000,1.1,(*pose,.025,0.))
                row=self.command(c)
                self.assertEqual(row['reason'],'STOP_FILTERED_ODOM_JUMP')
                self.assertEqual((row['v'],row['w']),(0.,0.))
                self.assertEqual(row['odom_jump_diagnostic']['stamp_ns'],1_100_000_000)

    def test_jump_then_revert_between_commands_is_latched(self):
        c=self.core();c.filtered(1_100_000_000,1.1,(0.,0.,.2,.025,0.))
        c.filtered(1_200_000_000,1.2,(0.,0.,0.,0.,0.))
        self.assertEqual(self.command(c)['reason'],'STOP_FILTERED_ODOM_JUMP')

    def test_heading_wrap_is_not_a_jump(self):
        c=self.core((0.,0.,math.pi-.01))
        c.filtered(1_100_000_000,1.1,(0.,0.,-math.pi+.01,0.,0.))
        self.assertIsNone(c.filtered_jump)

    def test_large_step_after_receive_gap_is_still_rejected(self):
        c=self.core();c.filtered(5_000_000_000,5.,(0.,0.,.2,0.,0.))
        self.assertEqual(self.command(c)['reason'],'STOP_FILTERED_ODOM_JUMP')

    def test_invalid_and_repeated_samples_cannot_replace_guard_baseline(self):
        c=self.core();before=c.previous_filtered_measurement
        c.filtered(1_000_000_000,2.,(9.,9.,9.,0.,0.))
        self.assertEqual(c.previous_filtered_measurement,before)
        c.filtered(2_000_000_000,2.,(float('nan'),0.,0.,0.,0.))
        self.assertEqual(c.previous_filtered_measurement,before)


if __name__=='__main__':unittest.main()
