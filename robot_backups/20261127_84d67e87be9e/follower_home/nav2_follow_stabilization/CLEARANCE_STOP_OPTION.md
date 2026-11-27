# Explicit follower LiDAR clearance stop option

The candidate driver and Nav2 follow-only operator both accept
`--disable-clearance-stop`. Defaults retain clearance stopping. A mismatch
between the operator option and the driver's reported policy prevents start.

When explicitly disabled, current and swept LiDAR clearance do not stop motion.
Actual clearance values remain in logs and command envelopes; no fake clear
distance is substituted. Collision stopping by follower LiDAR distance is then
unavailable. Leader Nav2 obstacle avoidance is unchanged.

The independent 0.9 m legacy follow-trial obstacle termination is excluded from
Nav2 goal-follow mode. Other legacy and avoidance modes retain their behavior.
Measurement validity, source/receive age checks, command/feedback watchdogs,
motor faults, RPM bounds, convoy separation checks, and time limits remain.

After changing driver options, stop the old owner before starting a new one.
Since the driver supplies odometry, restart the follower sensors/EKF,
localization and bridge in dependency order and re-establish the actual follower
map pose before sending a goal. Relays and the leader stack need no code update.

To restore clearance stopping, restart both driver and operator without the
disable flag and re-establish follower localization after the odometry restart.

Offline regression: 93 tests passed. Protected final_robot manifest: all 818
files unchanged. No robot runtime was launched by the agent. These checks do
not establish physical collision avoidance while distance stopping is disabled.
