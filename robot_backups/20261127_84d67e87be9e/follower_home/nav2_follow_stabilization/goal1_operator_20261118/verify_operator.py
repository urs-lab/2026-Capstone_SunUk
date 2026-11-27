from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parent
manifest=json.loads((root/'OPERATOR_MANIFEST.json').read_text())
bad=[name for name,expected in manifest['files'].items()
     if not (root/name).is_file() or hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected]
if bad:raise SystemExit('GOAL1_OPERATOR_FILE_CHANGED: '+','.join(bad))
print('정량1: 0.1m/s·15RPM 기존 추종 + 고정 콜백 처리·상태 발행 분리·지도/스캔 지연시 속도0 대기.',flush=True)
