#!/usr/bin/env bash
set -e
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ssh sunuk@10.253.245.81 'mkdir -p /home/sunuk/leader_diagnostics/reverse5m006_20261118'
scp "$here/leader/run_leader_distance_trial.py" "$here/leader/leader_short_forward_trial.py" "$here/leader/follower_yaw_diagnostics.py" "$here/leader/reverse_leader.bash" sunuk@10.253.245.81:/home/sunuk/leader_diagnostics/reverse5m006_20261118/
echo '선두 코드 복사 완료. 주행/arm 요청 없음.'
