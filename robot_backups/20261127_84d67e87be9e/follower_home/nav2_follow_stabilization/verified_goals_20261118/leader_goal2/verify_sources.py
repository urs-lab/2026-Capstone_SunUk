from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parent
data=json.loads((root/'SOURCE_MANIFEST.json').read_text())
bad=[name for name,expected in data['files'].items()
     if not (root/name).is_file() or hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected]
if bad:raise SystemExit('GOAL2_SOURCE_CHANGED: '+','.join(bad))
print('정량2: 기존3m 전진·0.1m/s·보정계수0.935594233291917 소스 확인.',flush=True)
