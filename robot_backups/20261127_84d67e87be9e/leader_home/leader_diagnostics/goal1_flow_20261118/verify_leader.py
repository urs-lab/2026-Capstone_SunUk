from pathlib import Path
import hashlib,json
p=Path(__file__).resolve().parent
expected=json.loads((p/'SOURCE_MANIFEST.json').read_text())
bad=[n for n,h in expected.items() if not (p/n).is_file() or hashlib.sha256((p/n).read_bytes()).hexdigest()!=h]
if bad:raise SystemExit('GOAL1_FLOW_LEADER_CHANGED: '+','.join(bad))
print('정량1 전용: 최대0.1m/s 유지, 출발0.06m/s 상한·간격1.5m 이상에서 후발속도+0.02 상한. 기존 시간/정지 검사 유지.')
