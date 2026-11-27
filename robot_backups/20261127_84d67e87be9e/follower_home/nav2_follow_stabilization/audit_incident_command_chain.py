#!/usr/bin/env python3
"""Offline inspection of saved commands using production RPM/packet functions.

No port is opened: MotorCore stays in its default dry-run mode. Integration
below assumes perfect tracking and sample-and-hold; it is not measured motion.
"""
import json
import math
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'candidate/follower_diagnostics'))
from follower_motor_candidate import Config, MotorCore, velocity_packet


def inspect_command(core, v, w):
    left, right = core.rpm(v, w)
    packet = velocity_packet(left, right)
    decoded = (struct.unpack_from('<h', packet, 6)[0],
               struct.unpack_from('<h', packet, 9)[0])
    if decoded != (left, right) or sum(packet) & 255:
        raise AssertionError('PACKET_ROUNDTRIP_OR_CHECKSUM')
    c = core.config
    physical_left = -left if c.invert_left else left
    physical_right = -right if c.invert_right else right
    factor = 2 * math.pi * c.radius / 60
    return dict(raw_rpm=[left, right],
                equivalent_v=(physical_left + physical_right) * factor / 2,
                equivalent_w=(physical_right - physical_left) * factor / c.track,
                packet_hex=packet.hex(' '))


def audit_log(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    controls = [r for r in rows if r.get('event') == 'local_control']
    end_ns = next(r['ros_ns'] for r in rows if r.get('event') == 'end')
    core = MotorCore(config=Config(max_rpm=10))
    converted = [dict(ros_ns=r['ros_ns'], v=r['v'], w=r['w'], reason=r['reason'],
                      **inspect_command(core, r['v'], r['w'])) for r in controls]
    requested_yaw = equivalent_yaw = 0.
    max_hold = 0.
    for i, r in enumerate(converted):
        next_ns = converted[i + 1]['ros_ns'] if i + 1 < len(converted) else end_ns
        dt = (next_ns - r['ros_ns']) / 1e9
        if dt < 0:
            raise ValueError('NONMONOTONIC_LOG')
        max_hold = max(max_hold, dt)
        requested_yaw += r['w'] * dt
        equivalent_yaw += r['equivalent_w'] * dt
    poses = [r for r in rows if r.get('event') == 'map_pose_trace'
             and r.get('robot') == 'follower']
    fault_ns = next(r['ros_ns'] for r in rows if r.get('event') == 'motor'
                    and r['status'].get('fault') == 'DEVICE_STATUS_NONZERO')
    before_fault = [r for r in poses if r['ros_ns'] <= fault_ns]
    pose_yaw = sum(math.atan2(math.sin(b['pose'][2]-a['pose'][2]),
                              math.cos(b['pose'][2]-a['pose'][2]))
                   for a,b in zip(poses, poses[1:]))
    motor_rows = [r for r in rows if r.get('event') == 'motor']
    logged_rpm = [r['status']['sent_rpm'] for r in motor_rows]
    return dict(log=str(path), samples=len(converted),
        max_logged_hold_s=max_hold,
        requested_yaw_integral_deg=math.degrees(requested_yaw),
        rounded_command_equivalent_yaw_integral_deg=math.degrees(equivalent_yaw),
        map_estimate_yaw_change_through_log_end_deg=math.degrees(pose_yaw),
        map_estimate_yaw_change_before_first_fault_observation_deg=math.degrees(
            before_fault[-1]['pose'][2] - poses[0]['pose'][2]),
        map_pose_first_receive_ns=poses[0]['ros_ns'],
        map_pose_last_receive_ns=poses[-1]['ros_ns'],
        first_fault_receive_ns=fault_ns,
        max_rounded_command_abs_rpm=max(abs(x) for r in converted for x in r['raw_rpm']),
        max_logged_sent_abs_rpm=max(abs(x) for pair in logged_rpm for x in pair),
        asymmetric_rounded_commands=sum(-r['raw_rpm'][0] != r['raw_rpm'][1] for r in converted),
        converted_commands=converted)


def sweep():
    core = MotorCore(config=Config(max_rpm=10))
    count = 0
    for vi in range(601):
        v = vi / 10000
        for wi in range(-200, 201):
            w = wi / 1000
            if v + 1e-12 < abs(w) * core.config.track / 2:
                continue
            r = inspect_command(core, v, w)
            left, right = r['raw_rpm']
            if not (-10 <= left <= 0 <= right <= 10):
                raise AssertionError((v, w, r))
            count += 1
    return dict(command_pairs_checked=count, sign_limit_packet_failures=0)


def main():
    report = dict(mode='offline_command_chain_audit',
        incident=audit_log(ROOT/'candidate/follower_diagnostics/logs/'
                           'local_avoidance_trial_1794465862905625282.jsonl'),
        sweep=sweep(),
        limitations=[
            'Saved samples are not a continuous serial-wire capture.',
            'Equivalent yaw assumes ideal wheel tracking and logged command hold.',
            'Map yaw is a localization estimate, not independent physical measurement.',
            'No motor dynamics, encoder failure or physical stopping is simulated.'],
        hardware_io_performed=False, motion_authorized=False)
    output = ROOT/'reports/incident_command_chain_audit.json'
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({**report, 'incident': {k:v for k,v in report['incident'].items()
                                           if k != 'converted_commands'}}, indent=2))
    print('REPORT='+str(output))


if __name__ == '__main__':
    main()
