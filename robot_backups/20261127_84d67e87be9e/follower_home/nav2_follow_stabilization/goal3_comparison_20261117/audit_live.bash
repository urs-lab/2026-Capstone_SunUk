#!/usr/bin/env bash
set -e
source "$HOME/nav2_follow_integration/baseline/follower_diagnostics/integration_async_env.bash"
audit_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
audit_report="$audit_dir/live_$(date +%Y%m%d_%H%M%S).txt"
python3 -B "$audit_dir/audit_goal3_connection.py" --tf-scope public --seconds "${1:-15}" | tee "$audit_report"
printf 'REPORT=%s\n' "$audit_report"
