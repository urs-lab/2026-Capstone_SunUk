#!/usr/bin/env bash
set -e
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ssh sunuk@10.253.245.81 'python3 -' <<'CHECK_LEADER_SOURCE'
from pathlib import Path
import hashlib
p=Path.home()/'leader_diagnostics/nav2_follow_stability_da0ab05848da'
expected={'nav2_convoy_gate_core.py': '32b52bfd8e05413a8f15efad62f4edb2653637cf79fc660da85e5f6b579d4321', 'nav2_convoy_gate.py': 'ce2b9735e3fb9ad0272c8edacd62cfb2c66567d0a0372b93dc62d606339faa23', 'convoy_source_time.py': 'ff87d0a82cb40826eaf6c56633c95be49b283e742a55be6497075d557234830d', 'leader_nav2_convoy.launch.py': '5635b3ab6b422be92796f18a42c01b20e4e174f445171f1b0b4a85f1933fed00', 'nav2_convoy_launch_core.py': 'f822a69fcd90aa917705ea3f934dae455d089ff823555a533f07f6429371df72'}
bad=[n for n,h in expected.items() if not (p/n).is_file() or hashlib.sha256((p/n).read_bytes()).hexdigest()!=h]
if bad:raise SystemExit('LEADER_SOURCE_DIFFERS_FROM_REVIEWED_BACKUP: '+','.join(bad))
print('Leader live source matches reviewed backup.')
CHECK_LEADER_SOURCE
ssh sunuk@10.253.245.81 'mkdir -p /home/sunuk/leader_diagnostics/goal1_flow_20261118'
python3 -B "$here/verify_sources.py"
scp "$here/leader/SOURCE_MANIFEST.json" "$here/leader/"*.py "$here/leader/start_leader_nav2.bash" sunuk@10.253.245.81:/home/sunuk/leader_diagnostics/goal1_flow_20261118/
echo '선두 정량1 별도 실행본 복사 완료. ROS 실행/설정 변경 없음.'
