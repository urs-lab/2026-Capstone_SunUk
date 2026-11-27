"""Verify the isolated goal1 driver copy without ROS or serial access."""
from pathlib import Path
import hashlib
import json

root=Path(__file__).resolve().parent
manifest=json.loads((root/'DRIVER_MANIFEST.json').read_text())
bad=[name for name,sha in manifest['files'].items()
     if not (root/name).is_file() or hashlib.sha256((root/name).read_bytes()).hexdigest()!=sha]
if bad:raise SystemExit('GOAL1_DRIVER_FILE_CHANGED: '+','.join(bad))
print('정량1: 기존 15RPM·시간제한 없음 설정 유지. 모터 DDS 텔레메트리 발행만 별도 프로세스로 분리.',flush=True)
