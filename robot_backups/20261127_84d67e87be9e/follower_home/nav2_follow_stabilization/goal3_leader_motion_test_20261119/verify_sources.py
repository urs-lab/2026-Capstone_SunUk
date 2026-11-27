from pathlib import Path
import hashlib,json
p=Path(__file__).resolve().parent
d=json.loads((p/'SOURCE_MANIFEST.json').read_text())
bad=[n for n,h in d['files'].items() if not (p/n).is_file() or hashlib.sha256((p/n).read_bytes()).hexdigest()!=h]
if bad:raise SystemExit('GOAL3_RANGE_TEST_CHANGED: '+','.join(bad))
o=json.loads((p/'ORIGINAL_SOURCE_MANIFEST.json').read_text());original=Path(o['root'])
bad=[n for n,h in o['files'].items() if not (original/n).is_file() or hashlib.sha256((original/n).read_bytes()).hexdigest()!=h]
if bad:raise SystemExit('ORIGINAL_SOURCE_CHANGED: '+','.join(bad))
print('별도 테스트 소스 확인·기존 평가3 소스 변경 없음.')
