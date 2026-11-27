"""Offline-only command alternatives. Not imported by robot runtime modules.

The floors are comparison hypotheses, not physically validated motor settings.
No ROS, serial, sockets, publishers, or service clients are used.
"""
import math


def alternative(v,w,reason,method,*,cap=.06,floor=.04,deadband=.023):
    if not all(math.isfinite(x) for x in (v,w,cap,floor,deadband)):
        raise ValueError('NONFINITE_COMMAND')
    if method not in ('recorded','floor','deadband'):
        raise ValueError('UNKNOWN_POLICY')
    if reason!='TRACKED_PURSUIT' or v<=0:
        return 0.,0.
    if v<abs(w)*.28-1e-9:
        raise ValueError('NON_FORWARD_COMMAND')
    if method=='deadband' and v<deadband:
        return 0.,0.
    requested=min(cap,max(v,floor) if method=='floor' else v)
    scale=requested/v
    v,w=v*scale,w*scale
    rpm=max(abs(v-w*.28),abs(v+w*.28))/.065*60/(2*math.pi)
    scale=min(1.,10./rpm) if rpm else 1.
    return v*scale,w*scale


def wheel_rpm(v,w):
    return tuple(round(x/.065*60/(2*math.pi)) for x in (v-w*.28,v+w*.28))
