#!/usr/bin/env bash
set -e
# Preserve the goal1 command while using the verified input-recovery copy.
exec bash "$HOME/nav2_follow_stabilization/goal1_operator_20261118/start_follow.bash" "${1:-0.1}"
