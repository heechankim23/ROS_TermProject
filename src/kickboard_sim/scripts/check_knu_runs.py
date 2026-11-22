#!/usr/bin/env python3
"""운영 기록(~/.ros/kickboard_ops/<시나리오>_*.jsonl, 시나리오별 최신)을 시나리오 YAML 의 expected 와 비교해 통과 여부를 낸다.

  python3 scripts/check_knu_runs.py               # 기록이 있는 시나리오 전부
  python3 scripts/check_knu_runs.py -o summary.txt

킥보드별 판정 (expected.dest -> 기록에서 확인하는 것)
  general      일반 슬롯에 정렬 완료 (slot_filled, kind normal)
  charging     충전 스탠드에 정렬 완료 (kind charge)
  broken       고장 분류 칸에 정렬 완료 (kind broken)
  charge_wait  일반 슬롯에 충전 대기로 정렬 (kind charge_wait 또는 charge 칸이 남아 있었으면 charge 도 인정)
  none         수거하지 않음
  cancelled    수거하지 않음 (요청 취소)
  held         정렬되지 않음 (로봇 고장으로 보류)
  unreachable  수거하지 않고 실패·관리자 알림
구역(zone)은 로봇 위치에 따라 달라질 수 있어 참고로만 센다.
"""
import argparse
import glob
import json
import os

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SC_DIR = os.path.join(PKG, 'config', 'scenarios_knu')
LOG_DIR = os.path.expanduser('~/.ros/kickboard_ops')
KIND_OK = {'general': {'normal'}, 'charging': {'charge'}, 'broken': {'broken'}, 'charge_wait': {'charge_wait', 'charge'}}


def finished(path):
    """끝까지 돈 기록인가: 정상 종료('end' 줄)했고, 마지막 상태가 대기(IDLE) — 로봇이 일을 다 마치고 쉬는 중에 끝남.
    중간에 Ctrl+C 로 끈 실행은 제외된다"""
    last_to, ended = None, False
    try:
        for ln in open(path, errors='ignore'):
            if '"ev": "state"' in ln:
                try:
                    last_to = json.loads(ln)['e'].get('to')
                except (ValueError, KeyError):
                    pass
            elif '"type": "end"' in ln:
                ended = True
    except OSError:
        return False
    return ended and last_to == 'IDLE'


def latest_logs():
    """시나리오별 기록: 정상 종료된 것 중 최신, 없으면 그냥 최신"""
    out, done = {}, {}
    for p in sorted(glob.glob(os.path.join(LOG_DIR, '*.jsonl')), key=os.path.getmtime):
        k = os.path.basename(p).rsplit('_', 2)[0]
        out[k] = p
        if finished(p):
            done[k] = p
    out.update(done)
    return out


def check(sc_id, log):
    sc = yaml.safe_load(open(os.path.join(SC_DIR, sc_id + '.yaml')))
    filled, collected, failed, cancelled, alerts = {}, set(), set(), set(), []
    for ln in open(log, errors='ignore'):
        try:
            r = json.loads(ln)
        except ValueError:
            continue
        if r.get('type') != 'event':
            continue
        e = r['e']
        sid = str(e.get('id', ''))
        ev = e.get('ev')
        if ev == 'slot_filled':
            filled[sid] = (e.get('kind') or '', e.get('slot') or '', e.get('zone') or '')
        elif ev == 'collected':
            collected.add(sid)
        elif ev == 'collect_failed':
            failed.add(sid)
        elif ev == 'item_status' and e.get('status') in ('FAILED', 'NEEDS_HUMAN'):
            failed.add(sid)
        elif ev == 'item_status' and e.get('status') == 'CANCELLED':
            cancelled.add(sid)
        elif ev == 'alert':
            alerts.append(sid)
    rows, ok, zone_same = [], 0, 0
    for ex in sc.get('expected') or []:
        sid, dest = str(ex['id']), ex['dest']
        got = filled.get(sid)
        if dest in KIND_OK:
            good = got is not None and got[0] in KIND_OK[dest]
            if good and got[2] == ex.get('zone'):
                zone_same += 1
            what = f'{got[1]} ({got[0]})' if got else ('적재만 함' if sid in collected else '수거 안 됨')
        elif dest in ('none', 'cancelled'):
            good = sid not in collected
            what = '수거 안 함' if good else '수거함'
        elif dest == 'held':
            good = got is None
            what = '보류' if good else f'{got[1]} 정렬'
        else:                                  # unreachable
            good = sid not in collected and (sid in failed or sid in alerts)
            what = '실패·알림' if good else ('수거함' if sid in collected else '판정 없음')
        ok += good
        rows.append({'id': sid, 'dest': dest, 'got': what, 'ok': bool(good)})
    n = len(rows)
    return {'id': sc_id, 'name': sc.get('name', sc_id), 'ok': ok, 'n': n, 'pass': ok == n, 'zone_same': zone_same,
            'rows': rows, 'log': log}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('names', nargs='*')
    ap.add_argument('-o', '--out', default='')
    a = ap.parse_args()
    logs = latest_logs()
    names = a.names or sorted(k for k in logs if os.path.exists(os.path.join(SC_DIR, k + '.yaml')))
    res = [check(k, logs[k]) for k in names if k in logs]
    lines = [f'{"시나리오":<6} {"판정":<6} {"킥보드":>8}  제목']
    for r in res:
        lines.append(f'{r["id"]:<8} {"통과" if r["pass"] else "부분":<6} {r["ok"]:>3}/{r["n"]:<3}  {r["name"]}')
        for row in r['rows']:
            if not row['ok']:
                lines.append(f'            #{row["id"]} 예상 {row["dest"]} -> 실제 {row["got"]}')
    missing = sorted(set(os.path.basename(p)[:-5] for p in glob.glob(os.path.join(SC_DIR, 'k*.yaml'))) - set(logs))
    if missing and not a.names:
        lines.append(f'기록 없음: {", ".join(missing)}')
    lines.append(f'통과 {sum(r["pass"] for r in res)} / 기록 {len(res)}')
    text = '\n'.join(lines)
    print(text)
    if a.out:
        open(a.out, 'w').write(text + '\n')
    json.dump({r['id']: {k: r[k] for k in ('ok', 'n', 'pass', 'rows')} for r in res},
              open(os.path.join(LOG_DIR, 'check.json'), 'w'), ensure_ascii=False)


if __name__ == '__main__':
    main()
