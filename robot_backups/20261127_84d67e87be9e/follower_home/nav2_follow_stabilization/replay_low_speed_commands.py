#!/usr/bin/env python3
"""Replay saved command samples only. Does not model new trajectories or faults."""
import json
from pathlib import Path
from offline_low_speed_policy import alternative,wheel_rpm


def summarize(commands,method):
    values=[]
    for row in commands:
        v,w=alternative(row['v'],row['w'],row['reason'],method)
        values.append(dict(v=v,w=w,rpm=wheel_rpm(v,w)))
    active=[r['v']>0 for r in values]
    return dict(samples=len(values),drive_samples=sum(active),
        drive_stop_edges=sum(a!=b for a,b in zip(active,active[1:])),
        samples_with_1_or_2rpm=sum(any(0<abs(x)<=2 for x in r['rpm']) for r in values),
        max_wheel_rpm=max((max(abs(x) for x in r['rpm']) for r in values),default=0),
        max_v=max((r['v'] for r in values),default=0),
        changes_original_stops=any((r['reason']!='TRACKED_PURSUIT' or r['v']<=0)
                                  and q['v']!=0 for r,q in zip(commands,values)))


def main():
    root=Path(__file__).resolve().parent
    files={'previous_114s':'local_avoidance_trial_1794385866172303503.jsonl',
           'incident':'local_avoidance_trial_1794465862905625282.jsonl'}
    results={}
    for label,name in files.items():
        path=root/'candidate/follower_diagnostics/logs'/name
        rows=[json.loads(s) for s in path.read_text().splitlines()]
        commands=[r for r in rows if r.get('event')=='local_control']
        results[label]={method:summarize(commands,method) for method in ('recorded','floor','deadband')}
    report=dict(mode='offline_open_loop_command_replay',results=results,
        caveats=['Original poses, path, leader state and stop events are held fixed.',
                 'No physical motor, new motion trajectory, sensor failure or closed-loop response is simulated.',
                 'A minimum speed is not established as safe or stable by these results.'],
        runtime_controller_changed=False,motion_authorized=False)
    out=root/'reports/low_speed_open_loop_comparison.json'
    out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
