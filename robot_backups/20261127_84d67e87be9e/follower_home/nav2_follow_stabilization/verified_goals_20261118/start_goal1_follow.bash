#!/usr/bin/env bash
set -e
exec bash "$HOME/nav2_follow_stabilization/goal1_operator_20261118/start_follow.bash" "${1:-0.1}"
