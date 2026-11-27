"""Fault injection through actual candidate gate callbacks; no ROS or hardware."""
import json
from pathlib import Path
import random
import time
from types import SimpleNamespace as S
from test_gate_source_adapter import adapter
from nav2_convoy_gate_core import Nav2ConvoyGateCore


def main():
    gate_class=type(adapter());rng=random.Random(20261111)
    categories=['motor','status','measurement','leader_pose','follower_pose']
    counts=dict(passed_valid=0,blocked_invalid=0)
    for _ in range(20000):
        target=rng.choice(categories)
        age=rng.choice((-.2,0.,.1,.299,.301,.5,1.2))
        bad_stamp=2_000_000_000-round(age*1e9)
        stamp=lambda name:bad_stamp if target==name else 2_000_000_000
        gate=gate_class.__new__(gate_class);gate.core=Nav2ConvoyGateCore()
        gate.source_rejections={};gate.get_clock=lambda:S(now=lambda:S(nanoseconds=2_000_000_000))
        gate.core.graph_ok=True;gate.core.update_nav(.15,.08,time.monotonic())
        gate.motor_cb(S(data=json.dumps(dict(status_ros_ns=stamp('motor'),armed=True,
            fault=None,io_failed=False,feedback_fresh=True,feedback_age_s=0.))))
        gate.follower_cb(S(data=json.dumps(dict(status_ros_ns=stamp('status'),
            source_stamp_ns=stamp('measurement'),trial_phase='FOLLOW',reason='TRACKED_PURSUIT',
            follower_follow_reason='TRACKED_PURSUIT',follower_requested_v=.04))))
        for name,child,x in [('leader_pose','base_link',1.5),('follower_pose','follower/base_link',0.)]:
            value=stamp(name)
            msg=S(header=S(frame_id='map',stamp=S(sec=value//10**9,nanosec=value%10**9)),
                  child_frame_id=child,pose=S(pose=S(position=S(x=x,y=0.))))
            (gate.leader_cb if name=='leader_pose' else gate.follower_pose_cb)(msg)
        output=gate.core.evaluate(time.monotonic())
        invalid=age<-.05 or age>.3
        if invalid:
            assert output.linear_x==0. and output.angular_z==0., (target,age,output)
            counts['blocked_invalid']+=1
        else:
            assert output.linear_x>0., (target,age,output)
            counts['passed_valid']+=1
    report=dict(seed=20261111,trials=20000,counts=counts,
                method='Actual gate callback bodies; fake clocks and message objects; no ROS transport',
                live_transport_tested=False,hardware_opened=False)
    out=Path(__file__).parent/'reports';out.mkdir(exist_ok=True)
    (out/'source_fault_replay.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
