#!/usr/bin/env bash
set -eo pipefail
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
set -u
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
convoy_tf_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
printf '%s\n' 'Follower public TF stack: AMCL dynamic /tf subscription BEST_EFFORT depth10; static TF and TF publishers unchanged.'
exec ros2 launch "$convoy_tf_dir/follower_success_tf_stack_be.launch.py" \
  map:="${1:-$HOME/robot_ws/maps/leader_mapping_20261107_143101_9R24Wc/leader_map.yaml}" \
  ekf_frequency:=20.0
