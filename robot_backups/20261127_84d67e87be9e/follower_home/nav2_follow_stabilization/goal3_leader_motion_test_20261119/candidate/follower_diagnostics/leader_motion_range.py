"""Diagnostic for the original leader velocity guard; no ROS/hardware."""
import math

def range_diagnostic(data):
    v,w=float(data[3]),float(data[4])
    if not all(math.isfinite(x) for x in (v,w)):
        raise ValueError('NONFINITE_LEADER_VELOCITY')
    reasons=[]
    if v<-.01:reasons.append('REVERSE_BELOW_MINUS_0_01')
    if v>.1:reasons.append('FORWARD_ABOVE_0_1')
    if abs(w)>.1:reasons.append('ABS_YAW_ABOVE_0_1')
    return dict(linear_x_mps=v,angular_z_radps=w,outside_original_range=bool(reasons),
                exceeded=reasons,original_min_v_mps=-.01,original_max_v_mps=.1,original_max_abs_w_radps=.1)
