#!/usr/bin/env python3
"""확장 프로필 해시 목록(scenarios_extended/legacy_sha256.json)을 지금 파일 내용으로 갱신한다.

test_extended.py::test_legacy_manifest_hashes 는 '확장 프로필이 기본 파일을 건드리지 않았는지'를
해시로 확인한다. 기본 파일(robot_params.yaml, collector.py 등)을 일부러 고쳤다면 이 스크립트로
목록을 갱신하고 같이 커밋한다. 목록에 있는 파일만 갱신하며, 파일을 추가·삭제하지는 않는다.

  python3 tools/update_legacy_hashes.py          # 갱신
  python3 tools/update_legacy_hashes.py --check  # 바뀐 파일만 출력 (바뀐 게 있으면 종료 코드 1)
"""
import hashlib
import json
import pathlib
import sys

SRC = pathlib.Path(__file__).resolve().parent.parent / 'src'
MANIFEST = SRC / 'kickboard_sim' / 'config' / 'scenarios_extended' / 'legacy_sha256.json'


def main():
    check = '--check' in sys.argv[1:]
    raw = MANIFEST.read_text()
    manifest = json.loads(raw)
    changed = []
    for rel, expected in manifest.items():
        digest = hashlib.sha256((SRC / rel).read_bytes()).hexdigest()
        if digest != expected:
            changed.append(rel)
            manifest[rel] = digest
    for rel in changed:
        print(('바뀜: ' if check else '갱신: ') + rel)
    if check:
        return 1 if changed else 0
    if changed:
        MANIFEST.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + ('\n' if raw.endswith('\n') else ''))
    print(f'{len(changed)}개 갱신' if changed else '바뀐 파일 없음')
    return 0


if __name__ == '__main__':
    sys.exit(main())
