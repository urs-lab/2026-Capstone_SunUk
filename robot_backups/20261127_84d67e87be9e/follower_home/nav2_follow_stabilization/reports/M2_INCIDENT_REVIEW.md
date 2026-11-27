# M2 unintended motion and encoder alarm — motion hold

The operator reported abrupt leftward motion several seconds after a Nav2 goal,
and eight red ALARM2 flashes. Do not repeat ground motion or clear alarms as a
workaround. Actual physical stopping must be checked independently of encoder
RPM when encoder feedback is faulty.

## Evidence

- Incident log: `candidate/follower_diagnostics/logs/local_avoidance_trial_1794465862905625282.jsonl`.
- First M2 alarm was logged 6.939 s after arm, status M1=0/M2=1.
- Logged follower command maximum: v=0.027980 m/s; w range -0.003200 to
  +0.000025 rad/s. Logged transmitted wheel command maximum magnitude: 4 RPM.
- Map/IMU-derived follower heading changed approximately 0.74 to 38.96 degrees.
  Pose estimates are not independent physical speed measurements.
- Several zero command/zero reported wheel RPM samples coexist with changing
  heading. Zero reported RPM therefore does not establish physical stopping.
- The MD200T manufacturer manual V1.6, section 3.2, lists eight alarm flashes
  as encoder signal error. This agrees with M2 alarm and recorded encoder-fail
  history, but does not establish the electrical/mechanical/software root cause.
- PID99 returned six bytes; it has not been declared compatible with the
  documented eight-byte dual-motor layout. Preserve raw bytes.

## Software review

`follower_hardware.yaml`, `follower_motor_candidate.py`, and the odometry
calibration JSON match the original follower copies byte-for-byte. The changes
to `follower_drive_candidate.py` since its saved pre-diagnostic copy only add
status diagnostics. This does not exclude effects of changed higher-level
commands, speed caps, update timing, or the disabled clearance policy.

The separate `KeyError: filtered` was fixed by excluding unused legacy leader
control calculations from Nav2 mode. It was not the termination reason of this
latest incident.

Stop-confirmation reporting has been corrected: faulted or nonzero device-status
feedback may not report `follower_zero_confirmed=true` merely because RPM=0.
This is a reporting fix, not a remedy for unintended physical motion.

## Before further motion

Keep motor power off for connector handling. Inspect the complete M2 encoder
signal path, supply/ground, connection integrity and interference, and compare
driver encoder/motor settings against the proven setup without writing defaults.
Persistent/recurrent encoder alarms and unintended motion require qualified
hardware/driver investigation. Do not disable encoder/alarm detection or make
ground motion the first verification. The requested 0.06 m/s/clearance-disabled
profile is not suitable for further trials during this incident investigation.

## Subsequent read-only comparison

Nine readable settings (PID1/21/65/79/122/156/170/185/221) match the older saved record. This excludes drift in those registers only. Current PID34/39 returned zero after power cycling, while the encoder error counter in PID229 increased from 1 to 2; HALL_FAIL stayed at 7. Power-on status alone is not evidence of repaired sensor feedback. PID99 remains an unconfirmed six-byte layout. No additional motor commands or setting writes were issued by this review.
