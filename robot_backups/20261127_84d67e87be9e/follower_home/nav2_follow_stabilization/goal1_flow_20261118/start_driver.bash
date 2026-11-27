#!/usr/bin/env bash
set -e
python3 -B "$HOME/nav2_follow_stabilization/goal1_flow_20261118/verify_sources.py"
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
exec python3 -B "$HOME/nav2_follow_stabilization/goal1_flow_20261118/driver/follower_drive_node.py" \
 --run-hardware --ack-hardware-zero-rpm-tx --isolated-telemetry \
 --input-kind local_avoidance --ack-bounded-pivot-candidate \
 --enable-motion-input --disable-clearance-stop --ack-extended-follow-speed \
 --trial-max-rpm 15 --trial-seconds 0 --wait-expired-reference \
 --fault-snapshot-dir "$HOME/nav2_follow_stabilization/goal1_flow_20261118/motor_faults"
