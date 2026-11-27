#!/usr/bin/env bash
set -e
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
exec python3 -B "$HOME/nav2_follow_stabilization/reverse5m006_20261118/follower/follower_drive_node.py" \
  --run-hardware --ack-hardware-zero-rpm-tx --isolated-telemetry \
  --input-kind local_avoidance --ack-bounded-pivot-candidate \
  --ack-distance-test-ignore-clearance --enable-motion-input \
  --trial-max-rpm 10 --trial-seconds 360
