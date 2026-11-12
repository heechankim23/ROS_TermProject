"""시나리오별 통과 기준 (ROS 없음). /mission_events 이벤트 목록으로 판정한다.

새 시나리오를 만들면 CHECKS 에 같은 이름으로 함수를 추가한다. 함수가 없으면 공통 기준만 본다.
각 함수는 [(통과 여부, 설명), ...] 를 돌려준다. 'info' 로 시작하는 설명은 참고용(판정에 안 들어감).
"""
import math
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARAMS = os.path.join(ROOT, 'src', 'kickboard_sim', 'config', 'robot_params.yaml')


class Ev:
    """이벤트 목록 조회 도우미"""

    def __init__(self, events):
        self.all = sorted(events, key=lambda e: e.get('t', 0.0))

    def of(self, kind, **match):
        return [e for e in self.all if e['ev'] == kind and all(e.get(k) == v for k, v in match.items())]

    def ids(self, kind, **match):
        return {e['id'] for e in self.of(kind, **match)}

    def final_status(self):
        st = {}
        for e in self.of('item_status'):
            st[e['id']] = e['status']
        return st

    def states(self):
        return self.of('state')


def queue_params():
    with open(PARAMS) as f:
        return yaml.safe_load(f)['queue_manager']['ros__parameters']


def starvation_bound(p):
    """queue_logic.starvation_bound 과 같은 식 (사유 가중치 3 - 1 = 2)"""
    return (p['w_r'] * 2.0 + p['w_d'] * p['d_max']) / p['w_t']


# ---------------- 공통 ----------------
def common(ev, sc):
    out = []
    stuck = [e for e in ev.states() if e['to'] == 'NEEDS_HUMAN']
    out.append((not stuck, '로봇이 NEEDS_HUMAN 으로 멈추지 않음' if not stuck else f'로봇 정지: {stuck[0].get("why")}'))
    for d in ev.of('docked'):
        ok = d['position_error_m'] <= 0.05 and abs(d['yaw_error_deg']) <= 5.0
        out.append((ok, f'도킹 오차 {d["position_error_m"] * 100:.1f} cm / {d["yaw_error_deg"]:.1f}° (≤ 5 cm, 5°)'))
    if ev.of('dock_failed'):
        out.append((False, f'도킹 실패 {len(ev.of("dock_failed"))}회'))
    return out


def expect_collected(ev, ids):
    got = ev.ids('collected')
    miss = sorted(set(ids) - got)
    return (not miss, f'수거 {sorted(got & set(ids))}' + (f', 못 함 {miss}' if miss else ''))


def model_ids(sc):
    return [str(s['id']) for s in sc.get('scooters', []) if s.get('model', True)]


# ---------------- 시나리오별 ----------------
def s1_single(ev, sc):
    reasons = {e['reason'] for e in ev.of('request_sent', id='1')}
    enq = ev.of('enqueued', id='1')
    return [expect_collected(ev, ['1']),
            (len(reasons) >= 2 and len(enq) == 1, f'같은 ID 병합 (사유 {sorted(reasons)}, 대기열 추가 {len(enq)}회)'),
            (bool([e for e in ev.states() if e['to'] == 'CHARGING']), '충전소 복귀·충전')]


def s2_new_request(ev, sc):
    out = [expect_collected(ev, ['1', '2', '3'])]
    enq3 = ev.of('enqueued', id='3')
    if enq3:
        t3 = enq3[0]['t']
        # 3번이 들어왔을 때 진행 중이던 대상이 끝나기 전에 다른 대상으로 바뀌지 않아야 함 (비선점)
        nav_before = [e for e in ev.states() if e['to'] == 'NAVIGATING' and e['t'] < t3]
        cur = nav_before[-1]['why'].split()[-1] if nav_before else None
        nxt = [e for e in ev.of('collected') if e['t'] > t3]
        ok = cur is None or (nxt and nxt[0]['id'] == cur)
        out.append((bool(ok), f'처리 중 새 요청: 현재 대상({cur}) 완료 후 재평가'))
    else:
        out.append((False, '3번 요청이 들어오지 않음'))
    return out


def s3_full_load(ev, sc):
    cap = sc.get('robot', {}).get('capacity', 4)
    full = [e for e in ev.states() if '(만재)' in e.get('why', '')]
    return [expect_collected(ev, model_ids(sc)),
            (bool(full), f'만재({cap}대) 후 하역·충전으로 전환' if full else '만재 전환 없음')]


def s4_low_battery(ev, sc):
    st = ev.states()
    ret = [e for e in st if e['to'] in ('RETURNING', 'CHARGING') and '배터리' in e.get('why', '')]
    charged = [e for e in st if e['frm'] == 'CHARGING' and e['to'] == 'SELECTING' and e['battery'] >= 80.0]
    after = [e for e in ev.of('collected') if charged and e['t'] > charged[0]['t']]
    requeue = [e for e in ev.of('item_status') if e.get('note') == 'battery low']
    return [expect_collected(ev, model_ids(sc)),
            (bool(ret), '배터리 20% 이하에서 복귀'),
            (bool(charged), '80% 충전 후 재개'),
            (bool(after), '충전 후 남은 대상 수거'),
            (True, 'info: 이동 중 대상 반납(REQUEUE) ' + ('있음' if requeue else '없음 — 적재 중에 20% 도달'))]


def s5_aging(ev, sc):
    p = queue_params()
    tmax = starvation_bound(p)
    enq = ev.of('enqueued', id='1')
    sel = [e for e in ev.states() if e['to'] == 'NAVIGATING' and e.get('why', '').endswith(' 1')]
    if not enq or not sel:
        return [(False, '1번이 선택되지 않음')]
    wait = sel[0]['t'] - enq[0]['t']
    return [(wait <= tmax, f'먼 1번이 대기 {wait:.0f}s 에 선택 (상한 t_max {tmax:.0f}s)'),
            expect_collected(ev, model_ids(sc))]


def s6_verify_fail(ev, sc):
    fails = [e for e in ev.of('collect_failed', id='1') if e['reason'].startswith('verify_failed')]
    return [(bool(fails), '실물 없는 1번: 회전 탐색 후 확인 실패'),
            (ev.final_status().get('1') == 'FAILED', '1번 대기열 상태 FAILED'),
            expect_collected(ev, ['2'])]


def s7_obstacle(ev, sc):
    out = [expect_collected(ev, model_ids(sc))]
    clr = [e for e in ev.of('runner_clearance')]
    if clr:
        c = clr[-1]['min_m']
        out.append((c > 0.0, f'장애물과 최소 간격 {c:.2f} m' + (' (충돌)' if c <= 0.0 else '')))
    else:
        out.append((True, 'info: 충돌 판정은 sim:=gz 에서만 (/sim/robot_pose 필요)'))
    return out


def s8_auto_cancel(ev, sc):
    cancelled = [e for e in ev.of('item_status', id='2', status='CANCELLED') if e.get('note') == 'auto cancel']
    return [(bool(cancelled), '다시 세워진 2번 자동 취소'), expect_collected(ev, ['1'])]


def s9_drop_sites(ev, sc):
    dropped = {e['id']: e for e in ev.of('scooter_dropped')}
    in_zone = all(dropped[i].get('in_zone') for i in ('1', '2') if i in dropped)
    re_req = [e for e in ev.of('request_sent') if e['id'] in dropped and e['t'] > dropped[e['id']]['t']]
    return [expect_collected(ev, ['1', '2', '3', '4']),
            (set(dropped) == {'1', '2'} and in_zone, f'주차구역 하역 {sorted(dropped)} (구역 안: {in_zone})'),
            (ev.ids('unloaded_at_charger') >= {'3', '4'}, f'충전소 하역 {sorted(ev.ids("unloaded_at_charger"))}'),
            (not re_req, '하역한 킥보드 재요청 없음' if not re_req else f'재요청 {sorted({e["id"] for e in re_req})}')]


CHECKS = {f.__name__: f for f in (s1_single, s2_new_request, s3_full_load, s4_low_battery, s5_aging,
                                  s6_verify_fail, s7_obstacle, s8_auto_cancel, s9_drop_sites)}


def check(name, events, scenario):
    """(통과 여부, [(ok, 설명)])"""
    ev = Ev(events)
    items = common(ev, scenario) + CHECKS.get(name, lambda e, s: [])(ev, scenario)
    ok = all(o for o, msg in items if not msg.startswith('info'))
    return ok, items


# ---------------- s7 충돌 판정용 기하 ----------------
ROBOT_L, ROBOT_W, ROBOT_CX = 1.6, 0.9, -0.05     # 차체 1.6 x 0.74 m (+바퀴), 모델 원점보다 5 cm 뒤가 중심


def _box_dist(px, py, ox, oy, hx, hy):
    return math.hypot(max(abs(px - ox) - hx, 0.0), max(abs(py - oy) - hy, 0.0))


def clearance(x, y, yaw, box, n=20):
    """로봇 차체 사각형과 축 정렬 상자 box = (cx, cy, sx, sy) 사이 최소 간격 (겹치면 0)."""
    ox, oy, sx, sy = box
    hx, hy = sx / 2, sy / 2
    c, s = math.cos(yaw), math.sin(yaw)
    best = float('inf')
    for i in range(n + 1):
        f = i / n
        for lx, ly in ((ROBOT_CX - ROBOT_L / 2 + f * ROBOT_L, -ROBOT_W / 2), (ROBOT_CX - ROBOT_L / 2 + f * ROBOT_L, ROBOT_W / 2),
                       (ROBOT_CX - ROBOT_L / 2, -ROBOT_W / 2 + f * ROBOT_W), (ROBOT_CX + ROBOT_L / 2, -ROBOT_W / 2 + f * ROBOT_W)):
            best = min(best, _box_dist(x + c * lx - s * ly, y + s * lx + c * ly, ox, oy, hx, hy))
    for cx, cy in ((ox + a * hx, oy + b * hy) for a in (-1, 1) for b in (-1, 1)):   # 상자 모서리가 차체 안
        lx, ly = c * (cx - x) + s * (cy - y), -s * (cx - x) + c * (cy - y)
        if abs(lx - ROBOT_CX) <= ROBOT_L / 2 and abs(ly) <= ROBOT_W / 2:
            return 0.0
    return best


def obstacle_boxes(scenario):
    return [(float(e['x']), float(e['y']), *[float(v) for v in e.get('size', (0.5, 0.5))[:2]])
            for e in scenario.get('events', []) if e.get('type') == 'obstacle']
