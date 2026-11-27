#!/usr/bin/env bash
# Run on follower after stopping the trial and the leader integration launch.
set -euo pipefail
gate_source_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
test -f "$gate_source_dir/nav2_convoy_gate.py"
test -f "$gate_source_dir/nav2_convoy_gate_core.py"
tar -C "$gate_source_dir" -cf - nav2_convoy_gate.py nav2_convoy_gate_core.py leader_nav2_convoy.launch.py |
  ssh sunuk@10.253.245.81 '
    set -eu
    cd ~/leader_diagnostics/nav2_follow_integration
    gate_backup_dir=$(mktemp -d backup_gate_XXXXXXXX)
    cp nav2_convoy_gate.py nav2_convoy_gate_core.py leader_nav2_convoy.launch.py "$gate_backup_dir/"
    tar -xf -
    echo "BACKUP=$gate_backup_dir"
    sha256sum nav2_convoy_gate.py nav2_convoy_gate_core.py leader_nav2_convoy.launch.py
  '
echo 'Transfer complete. Restart the leader integration launch to load the patch.'
