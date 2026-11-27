from pathlib import Path
import hashlib,json
p=Path(__file__).resolve().parent
expected=json.loads((p/'SOURCE_MANIFEST.json').read_text())['files']
bad=[n for n,h in expected.items() if not (p/n).is_file() or hashlib.sha256((p/n).read_bytes()).hexdigest()!=h]
if bad:raise SystemExit('GOAL1_FLOW_SOURCE_CHANGED: '+','.join(bad))
print('정량1 상태전달·속도연계 전용본 확인. 최고속도0.1m/s·15RPM·기존 신선도 검사 유지.')
