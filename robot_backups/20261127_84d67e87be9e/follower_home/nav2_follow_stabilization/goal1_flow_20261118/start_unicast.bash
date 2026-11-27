#!/usr/bin/env bash
set -e
python3 -B "$HOME/nav2_follow_stabilization/goal1_flow_20261118/verify_sources.py"
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
exec python3 -B "$HOME/nav2_follow_stabilization/goal1_flow_20261118/relay/convoy_unicast_relay.py" --role follower --peer 10.253.245.81
