#!/usr/bin/env bash
set -e
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
goal1_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$goal1_dir/verify_operator.py"
speed="${1:-0.1}"
case "$speed" in
  0.08|0.1) ;;
  *) echo 'Allowed speed: 0.08 or 0.1' >&2; exit 2 ;;
esac
export CONVOY_TF_STATIC_TOPIC=/tf_static
exec python3 -B "$goal1_dir/candidate/follower_diagnostics/run_local_avoidance_trial.py" \
  --run --ack-clear-both-power-cut-ready --nav2-leader --follow-only \
  --auto-start-on-goal --direct-unicast --exclude-body-internal-scan \
  --disable-clearance-stop --follow-max-linear "$speed" \
  --ack-extended-follow-speed --smooth-follow --pose-source filtered \
  --tf-static-topic /tf_static --seconds 0
