#!/usr/bin/env bash
set -euo pipefail
convoy_success_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Reuse the GitHub-success public-TF stack; remove the later UDP-only override.
# All changes are process-local. No motor commands, arm or global config writes.
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
printf '%s\n' 'Follower success-stack environment: default Fast DDS transport; public TF; no UDP-only profile override.'
exec bash "$convoy_success_dir/start_localization.bash" "$@"
