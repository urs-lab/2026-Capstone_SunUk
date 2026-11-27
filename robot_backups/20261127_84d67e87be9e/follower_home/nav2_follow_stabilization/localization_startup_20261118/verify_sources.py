from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parent
data=json.loads((root/'SOURCE_MANIFEST.json').read_text())
bad=[name for name,sha in data['files'].items() if not (root/name).is_file() or hashlib.sha256((root/name).read_bytes()).hexdigest()!=sha]
if bad:raise SystemExit('LOCALIZATION_SOURCE_CHANGED: '+','.join(bad))
print('후발 위치추정: 성공본 TF0.25.24 유지, 센서 후 기동·bond20초·active 상태 확인 적용. 모터/주행 명령 없음.',flush=True)
