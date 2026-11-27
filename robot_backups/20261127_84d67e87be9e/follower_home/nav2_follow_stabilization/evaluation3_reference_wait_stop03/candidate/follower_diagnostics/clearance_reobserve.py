"""Preserve immediate 3cm STOP; resume only after fresh clear sweeps and zeros."""
import math
from reference_wait import ReferenceWait
from evaluation3_clearance_policy import STOP_CLEARANCE_M

class ClearanceWait(ReferenceWait):
    triggers={'STOP_SWEPT_CLEARANCE','STOP_FOOTPRINT_CLEARANCE'}
    probe_reasons={'CONTINUOUS_READY','CONTINUOUS_RAMP_WARMUP','FOLLOW_WAITING_LEADER'}
    resume_clearance_m=STOP_CLEARANCE_M+.02
    def accepts(self,row):return row['reason'] in self.triggers|self.probe_reasons
    def observe(self,row,mono,state):
        safe=(row['reason'] in self.probe_reasons
              and all(math.isfinite(row[k]) and row[k]>self.resume_clearance_m for k in ('clearance_m','swept_clearance_m'))
              and state.get('armed') is True and state.get('feedback_fresh') is True
              and state.get('measured_rpm')==[0,0] and state.get('zero_feedback_samples',0)>=3)
        if not safe:self.enter(mono,invalid=True);return False
        return self.ready(mono,{'scan':row['scan_stamp_ns']},True)
