#!/usr/bin/env bash
set -eo pipefail
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
set -u
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
fixed_tf_dir="$HOME/nav2_follow_stabilization/goal3_comparison_20261117/tf2_deadlock_fix"
python3 -B "$fixed_tf_dir/verify_overlay.py"
success_tf_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 launch "$success_tf_dir/follower_success_tf_stack_fixed.launch.py" \
  map:="${1:-$HOME/robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml}" \
  ekf_frequency:=20.0
