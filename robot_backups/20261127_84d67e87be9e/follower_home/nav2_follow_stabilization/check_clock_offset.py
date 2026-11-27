#!/usr/bin/env python3
"""Bound remote-minus-local wall-clock offset over one read-only SSH session.

No clock, ROS, service, motor or system configuration is changed.
"""
import argparse
import json
from pathlib import Path
import select
import shlex
import subprocess
import time


def bounds(before_ns, remote_ns, after_ns):
    if after_ns<before_ns:raise ValueError('LOCAL_CLOCK_BACKWARDS')
    return ((remote_ns-after_ns)/1e9,(remote_ns-before_ns)/1e9)


def line_with_timeout(stream,seconds):
    if not select.select([stream],[],[],seconds)[0]:raise TimeoutError('SSH_REPLY_TIMEOUT')
    line=stream.readline()
    if not line:raise RuntimeError('SSH_CLOSED')
    return line.strip()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='sunuk@10.253.245.81')
    args=parser.parse_args()
    remote='import sys,time\nprint("READY",flush=True)\nfor line in sys.stdin:\n if line.strip()!="PING":break\n print(time.time_ns(),flush=True)\n'
    cmd=['ssh','-T','-o','ConnectTimeout=5','-o','ServerAliveInterval=3',
         '-o','ServerAliveCountMax=2',args.host,'python3 -u -c '+shlex.quote(remote)]
    print('조회 전용: 원격 시각을 읽어 비교합니다. 시계·ROS 설정 변경 없음.',flush=True)
    process=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
    rows=[];rejected=[]
    try:
        if line_with_timeout(process.stdout,45)!='READY':raise RuntimeError('INVALID_SSH_HANDSHAKE')
        for _ in range(20):
            mono_before=time.monotonic_ns();before=time.time_ns()
            process.stdin.write('PING\n');process.stdin.flush()
            remote_ns=int(line_with_timeout(process.stdout,3))
            after=time.time_ns();mono_after=time.monotonic_ns()
            rtt=(mono_after-mono_before)/1e9
            if abs((after-before)-(mono_after-mono_before))>5_000_000:
                rejected.append('LOCAL_WALL_CLOCK_CHANGED');continue
            lo,hi=bounds(before,remote_ns,after)
            rows.append(dict(rtt_s=rtt,remote_minus_local_lower_s=lo,
                             remote_minus_local_upper_s=hi))
            time.sleep(.05)
        if not rows:raise RuntimeError('NO_VALID_CLOCK_SAMPLES')
        best=min(rows,key=lambda r:r['rtt_s'])
        # An intersection captures a constant offset only if clocks remained
        # sufficiently consistent throughout this short exchange.
        lo=max(r['remote_minus_local_lower_s'] for r in rows)
        hi=min(r['remote_minus_local_upper_s'] for r in rows)
        consistent=lo<=hi
        within=consistent and best['remote_minus_local_lower_s']>=-.05 and best['remote_minus_local_upper_s']<=.05
        report=dict(host=args.host,meaning='remote leader clock minus local follower clock',
            best_round_trip_sample=best,constant_offset_consistent=consistent,
            constant_offset_interval_s=[lo,hi] if consistent else None,
            bounded_within_50ms=within,rejected=rejected,samples=rows)
        out=Path(__file__).parent/'reports';out.mkdir(exist_ok=True)
        path=out/f'clock_offset_{time.time_ns()}.json'
        path.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2),flush=True)
        print('REPORT='+str(path),flush=True)
    finally:
        if process.stdin:process.stdin.close()
        try:process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:process.wait(timeout=2)
            except subprocess.TimeoutExpired:process.kill();process.wait()


if __name__=='__main__':main()
