#!/usr/bin/env python3
"""운영 기록(ops_monitor 의 *.jsonl) -> 웹 리플레이 한 파일 (역할 ④).

  python3 scripts/build_ops_replay.py                       # ~/.ros/kickboard_ops 에서 시나리오별 최신 기록 전부
  python3 scripts/build_ops_replay.py 기록1.jsonl 기록2.jsonl -o 결과.html

만든 HTML 을 브라우저로 열면 캠퍼스 지도 위에서 로봇 이동·킥보드 상태 변화·정렬 결과를 재생한다 (인터넷 없이 동작, 글꼴만 온라인).
"""
import argparse
import glob
import json
import os

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAX_SNAPS = 500


def load_run(path):
    recs = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                try:
                    recs.append(json.loads(ln))
                except ValueError:
                    pass                      # 강제 종료로 마지막 줄이 잘린 경우
    meta = next((r for r in recs if r.get('type') == 'meta'), {'scenario': os.path.basename(path), 'id': 'run'})
    snaps = [r for r in recs if r.get('type') == 'snap']
    if len(snaps) > MAX_SNAPS:                  # 긴 기록은 스냅샷을 솎아 파일 크기를 줄인다 (이벤트는 전부 둔다)
        step = len(snaps) / MAX_SNAPS
        keep = {id(snaps[int(k * step)]) for k in range(MAX_SNAPS)} | {id(snaps[-1])}
        recs = [r for r in recs if r.get('type') != 'snap' or id(r) in keep]
    for r in recs:                              # 바뀌지 않는 큰 항목은 빼서 크기 줄이기
        if r.get('type') == 'snap':
            r.pop('occ', None) if not any(r.get('occ', {})) else None
    return {'meta': {'scenario': meta.get('scenario', ''), 'id': meta.get('id', 'run')},
            'records': [r for r in recs if r.get('type') in ('snap', 'event')], 'note': ''}


def campus_map():
    m = json.load(open(os.path.join(PKG, 'maps', 'knu_campus.json')))
    r1 = lambda p: [[round(x, 1), round(y, 1)] for x, y in p]   # noqa: E731
    return {'boundary': r1(m['boundary']),
            'buildings': [{'p': r1(b['poly']), 'n': b['number'], 'c': r1([b['c']])[0]} for b in m['buildings']],
            'roads': [{'k': 's' if r['kind'] in ('service', 'residential') else 'f', 'p': r1(r['pts'])}
                      for r in m['roads'] if r['kind'] != 'steps'],
            'green': [r1(g['poly']) for g in m['green']], 'water': [r1(w['poly']) for w in m['water']],
            'zones': {z['id']: {'type': z['type'], 'poly': r1(z['poly']), 'c': r1([z['rack']['center']])[0], 'n': len(z['slots'])}
                      for z in m['parking_zones'] if z['rack']},
            'slots': {s['id']: r1([s['xy']])[0] for z in m['parking_zones'] for s in z['slots']}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='*')
    ap.add_argument('-o', '--out', default=os.path.join(os.path.dirname(os.path.dirname(PKG)), 'results', 'ops_replay.html'))
    ap.add_argument('--note', default='', help='페이지 아래에 붙일 설명 (예: 테스트 속도)')
    a = ap.parse_args()
    logs = a.logs
    if not logs:                                  # 시나리오별 최신 기록
        import importlib.util
        spec = importlib.util.spec_from_file_location('chk', os.path.join(PKG, 'scripts', 'check_knu_runs.py'))
        chk = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(chk)
        logs = sorted(chk.latest_logs().values())       # 시나리오별 정상 종료된 최신 기록
    runs = [load_run(p) for p in logs]
    runs = [r for r in runs if r['records']]
    chk = {}
    cp = os.path.expanduser('~/.ros/kickboard_ops/check.json')
    if os.path.exists(cp):                      # check_knu_runs.py 결과 (예상 대비 판정)
        chk = json.load(open(cp))
    for r in runs:
        r['note'] = a.note
        r['check'] = chk.get(r['meta']['id'])
    runs.sort(key=lambda r: r['meta']['id'])
    if not runs:
        raise SystemExit('기록이 없습니다. 시뮬레이션을 먼저 돌리세요 (ops_monitor 가 ~/.ros/kickboard_ops 에 기록).')
    data = json.dumps({'map': campus_map(), 'runs': runs}, ensure_ascii=False, separators=(',', ':')).replace('</', '<\\/')
    page = open(os.path.join(PKG, 'scripts', 'ops_replay_template.html')).read().replace('/*DATA*/', data)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w') as f:
        f.write('<!doctype html><html lang="ko"><head><meta charset="utf-8">'
                '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover"></head><body>\n'
                + page + '\n</body></html>\n')
    print(f'{a.out}  (기록 {len(runs)}개: {", ".join(r["meta"]["id"] for r in runs)})')


if __name__ == '__main__':
    main()
