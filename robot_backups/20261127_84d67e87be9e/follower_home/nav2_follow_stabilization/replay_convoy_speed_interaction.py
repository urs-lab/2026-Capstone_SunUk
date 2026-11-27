#!/usr/bin/env python3
"""Deterministic fixed-gap counterexample using real preview and gate cores.

No ROS, serial, network or motor I/O. The leader measurement equals the prior
gate output, and distance is held fixed to isolate the control interaction.
This is not a physical closed-loop model or an encoder-fault reproduction.
"""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as S

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'candidate/follower_diagnostics'))
from follower_pursuit_preview import PreviewCore
from nav2_convoy_gate_core import Nav2ConvoyGateCore
from offline_low_speed_policy import alternative,wheel_rpm
from nav2_follow_safety import bounded_pursuit_result
from temporary_detour_candidate import Pose
import numpy as np


def run(gap,method='recorded',steps=20):
    follower=PreviewCore();gate=Nav2ConvoyGateCore(max_linear=.06)
    gate.graph_ok=True
    measured_leader_v=0.;rows=[]
    for i in range(steps):
        ns=100_000_000_000+i*50_000_000;mono=10.+i*.05
        stamp=S(sec=ns//10**9,nanosec=ns%10**9)
        zero=S(x=0.,y=0.,z=0.)
        msg=S(header=S(frame_id='map',stamp=stamp),child_frame_id='base_link',
              pose=S(pose=S(position=S(x=gap,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))),
              twist=S(twist=S(linear=S(x=measured_leader_v,y=0.,z=0.),angular=zero)))
        follower.inputs['follower']=(stamp,mono,(0.,0.,0.,0.,0.))
        follower.inputs['path']=(stamp,mono,((0.,0.),(3.,0.)))
        follower.odom_callback('leader',msg,ns,mono)
        v,_,reason=follower.speed_gate(ns,mono)
        v=min(v,.06)
        if method=='floor':
            scene=dict(pose=Pose(0.,0.,0.),points=np.array([[3.,0.]]),
                source=ns,scan_stamp=ns,uncertainty=0.,clearance=2.,
                source_stamps=dict(follower_map=ns,scan=ns),pose_scan_delta_s=0.)
            output=bounded_pursuit_result(scene,v,0.,'TRACKED_PURSUIT' if v>0 else reason,
                min_linear=.04,max_linear=.06)
            v,w=output['v'],output['w']
        else:
            v,w=alternative(v,0.,'TRACKED_PURSUIT' if v>0 else reason,method)
        gate.update_nav(.15,0.,mono)
        gate.update_follower(dict(trial_phase='FOLLOW',reason='TRACKED_PURSUIT' if v>0 else 'FOLLOW_WAITING_LEADER',
            follower_follow_reason=reason,follower_requested_v=v),mono)
        gate.update_motor(dict(armed=True,fault=None,io_failed=False,feedback_fresh=True),mono)
        gate.update_leader_pose(gap,0.,mono);gate.update_follower_pose(0.,0.,mono)
        output=gate.evaluate(mono)
        rows.append(dict(step=i,leader_measured_v=measured_leader_v,
                         follower_v=v,follower_reason=reason,follower_rpm=wheel_rpm(v,w),
                         leader_gate_v=output.linear_x,leader_gate_reason=output.reason))
        measured_leader_v=output.linear_x
    active=[r['follower_v']>0 for r in rows]
    return dict(gap=gap,method=method,steps=steps,
        follower_drive_stop_edges=sum(a!=b for a,b in zip(active,active[1:])),
        rows=rows)


def main():
    report=dict(mode='fixed_gap_ideal_measurement_counterexample',
        assumptions='gap fixed; ideal leader velocity measurement; no inertia, obstacles, transport jitter or hardware',
        results=[run(1.51),run(1.70),run(1.51,'floor')],
        runtime_controller_changed=False,encoder_alarm_cause_proven=False)
    p=ROOT/'reports/convoy_speed_interaction_counterexample.json'
    p.write_text(json.dumps(report,indent=2)+'\n')
    for r in report['results']:
        print(json.dumps({k:v for k,v in r.items() if k!='rows'}))
        print(json.dumps(r['rows'][:4]))
    print('REPORT='+str(p))


if __name__=='__main__':main()
