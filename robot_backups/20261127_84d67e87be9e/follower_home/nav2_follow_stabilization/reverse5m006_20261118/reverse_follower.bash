#!/usr/bin/env bash
set -e
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
exec python3 -B "$HOME/nav2_follow_stabilization/reverse5m006_20261118/follower/run_follower_distance_trial.py" --run --ack-clear-path-power-cut-ready --distance-m 5
