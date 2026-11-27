#!/usr/bin/env bash
set -e
stack_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$stack_dir/verify_sources.py"
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
exec python3 -B "$stack_dir/localization/real_follower_bridge_tf_candidate.py"
