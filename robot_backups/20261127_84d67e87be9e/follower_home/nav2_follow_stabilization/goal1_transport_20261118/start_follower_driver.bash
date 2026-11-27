#!/usr/bin/env bash
set -e
driver_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$driver_dir/verify_driver.py"
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
exec python3 -B "$driver_dir/driver/follower_drive_node.py" \
  --run-hardware --ack-hardware-zero-rpm-tx --isolated-telemetry \
  --input-kind local_avoidance --ack-bounded-pivot-candidate \
  --enable-motion-input --disable-clearance-stop \
  --ack-extended-follow-speed --trial-max-rpm 15 --trial-seconds 0 \
  --fault-snapshot-dir "$HOME/nav2_follow_stabilization/smooth010/motor_faults"
