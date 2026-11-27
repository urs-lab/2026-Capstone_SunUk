#!/usr/bin/env bash
set -e
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ssh sunuk@10.253.245.81 'mkdir -p /home/sunuk/leader_diagnostics/reverse9m050_20261118'
scp "$here/run_leader_distance_trial.py" "$here/leader_short_forward_trial.py" "$here/follower_yaw_diagnostics.py" "$here/leader_eval2_speed050.launch.py" "$here/reverse_leader.bash" sunuk@10.253.245.81:/home/sunuk/leader_diagnostics/reverse9m050_20261118/
echo '선두 복사 완료. 주행/arm 요청 없음.'
