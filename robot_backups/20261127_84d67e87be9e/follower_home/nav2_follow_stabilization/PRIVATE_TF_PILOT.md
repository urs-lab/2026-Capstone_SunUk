# Stationary follower TF-isolation pilot

This profile isolates TF TOPICS, not DDS domains or access permissions. It is
not a security boundary and does not establish hard real-time behavior.

Evidence: adding follower sensor/EKF nodes left leader odometry and scans
reasonably fresh but reliable TF reception lagged by 5–7 seconds, with up to
8.18s receive gaps. Best-effort reception of the same edges was much fresher.
We therefore test removing irrelevant cross-robot reliable TF subscriptions,
before adding AMCL or increasing speeds. This does not prove a specific DDS bug.

Sensors use the existing verified launch and binary. All follower dynamic TF
is scoped to `/follower/tf_internal`, static mounts to
`/follower/tf_static_internal`. Frame IDs and sensor/odometry topics are intact.
No relay, modified timestamp, dropped obstacle, timeout increase or new motor
owner is introduced. Tests check both launch wrappers and remap precedence using
the installed Humble launch API. Live data-path behavior still needs measurement.

For this pilot, leader remains sensor-only, Domain 10, asynchronous and
allow_motion=false. Follower motor remains running with motion input disabled.
AMCL, bridges, RViz and follower trial remain STOPPED on both hosts.

Stop ONLY the follower sensor/EKF launch. Start its alternative wrapper:

```bash
source ~/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash
ros2 launch ~/nav2_follow_stabilization/follower_private_tf_sensors.launch.py \
  enable_position_profile:=true ekf_frequency:=20.0
```

Do not run both sensor launches together. On the unchanged leader run the same
host input audit. The follower TF should no longer appear on PUBLIC /tf. The
leader reliable and best-effort TF should be compared over the same interval.

Rollback: stop only the private sensor launch and restart the original
follower_position_fixed_lidar.launch.py with the same flags. Motor remains
disarmed throughout. Localization poses must be re-established before a later
control session because odometry restarts reset its origin.

This pilot is NOT a complete navigation rollout. Later follower AMCL must use
follower_private_tf_localization.launch.py; bridge TF inputs and trial static
TF subscription must also use the private topics. Do not start the old AMCL or
trial against the private sensor stack. Fleet pose output remains a map-frame
message and does not require broadcasting raw follower TF to the leader.
