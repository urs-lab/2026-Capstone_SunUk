#!/usr/bin/env bash
set -e
stack_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$stack_dir/verify_sources.py"
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
exec ros2 launch "$stack_dir/localization/follower_success_tf_stack.launch.py" \
  map:="${1:-$HOME/robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml}" \
  ekf_frequency:=20.0
