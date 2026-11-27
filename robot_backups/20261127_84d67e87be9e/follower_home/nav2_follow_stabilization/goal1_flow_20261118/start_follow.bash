#!/usr/bin/env bash
set -e
python3 -B "$HOME/nav2_follow_stabilization/goal1_flow_20261118/verify_sources.py"
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
export CONVOY_TF_STATIC_TOPIC=/tf_static
exec python3 -B "$HOME/nav2_follow_stabilization/goal1_flow_20261118/operator/run_local_avoidance_trial.py" \
 --run --ack-clear-both-power-cut-ready --nav2-leader --follow-only \
 --auto-start-on-goal --direct-unicast --exclude-body-internal-scan \
 --disable-clearance-stop --follow-max-linear 0.1 --ack-extended-follow-speed \
 --smooth-follow --pose-source filtered --tf-static-topic /tf_static --seconds 0
