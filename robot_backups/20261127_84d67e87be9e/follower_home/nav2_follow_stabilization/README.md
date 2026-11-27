# Isolated Nav2 convoy stabilization candidate

Status: offline regression verified; NOT deployed or physically validated.
No program was started on either robot by this work. No motor, sensor, map,
lifecycle, initial pose, firmware, or network setting was changed at runtime.

## Protected state

`protected_baseline_sha256.json` fingerprints 818 files in `~/final_robot`.
`protected_success_backup.tar.gz` is an independent copy. Both are checked
against the manifest. The pre-existing `~/nav2_follow_integration` remains
separate from this candidate. Never reset or overwrite the successful repo.

## What is changed in this candidate

* Driver status contains `status_ros_ns`, captured before publication.
* Follower status contains both original measurement and publication stamps.
* Motor feedback age is increased by time spent in status transport; a fresh
  receipt cannot hide an old feedback sample.
* The gate rejects old/missing/future publication stamps and old/future source
  measurements, even when receipt is recent. Invalid input clears the previous
  cached sample. The 0.3s timeouts remain unchanged; remote future allowance is
  bounded to 50ms, requiring clocks to be synchronized within that margin.
* Joint v/w scaling preserves the Nav2 curvature. Default remains 0.04m/s.
* Read-only stationary audit checks original source ages, receipt ages,
  backward timestamps, AMCL lifecycle activity, TF edges, graph exclusivity,
  stationary twists and healthy disarmed zero-RPM motor status. It requires
  five continuous seconds of valid observations and immediately clears its
  readiness result when an input fails. A ready report does not arm anything
  or authorize movement; it is a data-readiness check only.

An AMCL map->odom TF may legitimately be future-dated by transform_tolerance;
this is handled separately from measurements. We never restamp stale data as
current, extend watchdogs, delete internal LiDAR hits, or automatically rearm.

## Offline checks

* 97 existing motor tests: passed.
* 39 existing gate/follower/launch/preflight tests: passed.
* 11 new timing/readiness/actual-callback tests: passed.
* 20,000 injected source-delay/future-time cases through actual gate callback
  bodies: 8,646 valid accepted, 11,354 invalid stopped; no unexpected outputs.
* Python syntax compilation: passed for modified runtime files.

Fake clocks, byte transports and message objects are used. The prior 40,000
publication-stall comparisons modeled async behavior; they did NOT execute or
certify DDS. There is no claim of globally optimal behavior or a hardware
worst-case timing/stopping-distance guarantee.

## Read-only live evidence (first next step)

Stop the current follower trial and do not send new Nav2 goals. Leave the
stationary observation stack running so evidence is not erased by restarts.
On follower:

```bash
source ~/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash
python3 -B ~/nav2_follow_stabilization/observe_stationary.py --seconds 30
```

This only subscribes, reads graph metadata and calls GetState lifecycle READ
services; it never calls activate/deactivate, arm/disarm or motion services.
Reports go to `reports/stationary_*.json`. The currently deployed legacy driver
does not include status_ros_ns, so `motor:MISSING_SOURCE_STAMP` is EXPECTED in
this baseline audit. It must not be bypassed: coordinated candidate deployment
supplies this field. Other issues identify actual missing/stale pipeline stages.

The launch of a legacy driver with the candidate gate is intentionally rejected
by the timing contract. Never deploy just the new gate into the old session.

## Coordinated deployment plan (not executed)

1. Record the read-only baseline; resolve missing/stale scan, TF and AMCL
   stages and verify host clock agreement before changing control behavior.
2. Stop both goal session and integration gate/Nav2 before replacing files.
   Stop follower localization/bridge/sensors/driver in dependency order. No
   automatic restarts or fault clearing while armed are permitted.
3. Leader gate, gate core, launch wrapper, launch helper and
   convoy_source_time.py MUST be deployed together. Use the prepared leader
   bundle; copy it to a new leader directory, not into success backup paths.
4. Follower driver and trial MUST both come from candidate/follower_diagnostics,
   with the existing verified hardware config. Source async environment before
   starting them. Sensor launch can use the existing verified
   ~/follower_diagnostics/follower_position_fixed_lidar.launch.py (20Hz), since
   the candidate reference copy excludes LiDAR build artifacts.
5. Start sensors/EKF, then localization, then bridges, then leader integration
   Nav2 gate. After any odometry restart, re-establish BOTH real map poses via
   separate /initialpose and /follower/initialpose tools. Do not globally remap
   all pose inputs to follower. No arbitrary assumed coordinates are sent.
6. Repeat 30s and 60s stationary audits; inspect full source-age distributions
   and readiness losses, not just the last sample. RViz remains disabled for
   the baseline and is then reintroduced with laser/TF displays off, frame
   rate 10 and one map display. Preserve notebook's existing Nav2 Goal tool.
7. Only after stable stationary measurements, compare short straight, stop,
   gentle curve, goal cancel, input-loss and operator-stop cases at 0.04m/s.
   Physical response must be observed; never deliberately interrupt a moving
   hardware link as a first test. 0.05m/s comparison is deferred.

## Unresolved constraints

The repeating scan hit inside the follower footprint remains a real stop
condition until the mounting/body reflection/packet evidence is understood.
Raw checksum errors and TF subscriber freezes have not been reproduced in the
network-restricted tool sandbox. Async queues can still back up. Plain upstream
Nav2 Twist has no original source timestamp. Hardware stopping during a whole
host freeze requires independent hardware protection; software tests cannot
prove it. Existing emergency power-cut readiness is still required.

## Rollback

Do not unpack the archive over a running robot. Stop the integration session
first. Before deployment, current integration files remain untouched, so rollback
is simply to use the prior launch commands after a clean stationary restart and
pose reinitialization. Any future deployment must retain its own file manifest
and backup directory. Protected success backup is never a deployment target.
