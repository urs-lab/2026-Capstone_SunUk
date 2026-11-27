#!/usr/bin/env bash
set -e
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
# Keep the current device-status/encoder fault guards; use finite evaluation3 mode.
exec python3 -B "$HOME/nav2_follow_stabilization/smooth010/candidate/follower_diagnostics/follower_drive_node.py" \
  --run-hardware --ack-hardware-zero-rpm-tx \
  --input-kind local_avoidance --ack-bounded-pivot-candidate \
  --ack-continuous20-candidate --enable-motion-input \
  --trial-max-rpm 10 --trial-seconds 360
