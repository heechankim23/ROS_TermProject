"""mission_manager 가공부: 미션 판단과 적재함·충전 자리 관리 (ROS 없음 -> 단위 테스트·오프라인 실험 가능).

mission_manager(통신부)는 0.2 s 마다 decide_tick 을 부르고, 결과에 따라 서비스·액션을 호출한다.
판단 규칙을 바꾸고 싶으면 이 파일만 고치면 된다 (노드 연결은 그대로).

  decide_tick        : 상태별 다음 행동 (SELECTING 이면 decide_select 로)
  decide_select      : 충전소 하역 / 복귀(주차구역 하역 먼저 시도) / 대기열에 다음 대상 요청
  decide_queue_empty : 대기열이 비었을 때 (주차구역 하역 -> 도크면 IDLE, 아니면 복귀)
  EnergyModel, collect_cost, drop_cost, must_head_home : 배터리로 '갔다가 충전소까지 돌아올 수 있나' 판단
  drop_next_points   : 주차구역 하역 후 갈 곳 후보 (하역 자리를 고를 때 그다음 이동 거리까지 보려고)
  classify_failure   : 수거 실패 사유 -> 대기열 상태(FAILED / NEEDS_HUMAN)와 알림 종류
  Cargo, ChargerSlots: 적재함 내용, 충전 자리 배정
"""
import math
from dataclasses import dataclass

IDLE, SELECTING, NAVIGATING, VERIFYING, LOADING = 'IDLE', 'SELECTING', 'NAVIGATING', 'VERIFYING', 'LOADING'
FAILED, RETURNING, DOCKING, CHARGING = 'FAILED', 'RETURNING', 'DOCKING', 'CHARGING'
DROPPING = 'DROPPING'
NEEDS_HUMAN = 'NEEDS_HUMAN'    # 도킹 반복 실패 등: 정지 후 관리자 처리 대기
STATES = (IDLE, SELECTING, NAVIGATING, VERIFYING, LOADING, FAILED, RETURNING, DOCKING, CHARGING, DROPPING,
          NEEDS_HUMAN)
PARKING, CHARGER = 'parking', 'charger'      # 킥보드별 하역 장소
COLLECT_PHASE_TO_STATE = {'NAVIGATING': NAVIGATING, 'VERIFYING': VERIFYING,
                          'ALIGNING': LOADING, 'LOADING': LOADING}
FAILED_RETRY_SEC = 1.0                       # FAILED 에서 다시 SELECTING 으로 가기까지
# 대기열 항목 중 아직 그 자리에 놓여 있을 킥보드 (하역 위치를 고를 때 피함). DONE·CANCELLED 는 이미 없음
LYING_STATUSES = ('WAITING', 'IN_PROGRESS', 'FAILED', 'NEEDS_HUMAN')


@dataclass
class MissionParams:
    capacity: int = 3
    return_battery_pct: float = 20.0
    charge_target_pct: float = 80.0
    scooter_battery_threshold_pct: float = 20.0
    admin_alerts: bool = True
    max_dock_retries: int = 3
    drop_next_weight: float = 1.0      # 하역 자리 고를 때 '하역 후 갈 곳' 거리 가중치 (0 = 가장 가까운 곳, 예전 방식)


@dataclass
class Decision:
    """action:
      SELECT          -> decide_select 실행 (SELECTING 상태)
      SET             -> state 로 전이 (why 를 로그에)
      ABORT_COLLECT   -> 진행 중인 수거를 취소하고 대상을 반납 (배터리 부족)
      UNLOAD_AT_DOCK  -> 충전소에서 충전소행(또는 전부) 킥보드 하역
      RETURN          -> try_drop 이면 주차구역 하역부터 시도, 안 되면 fallback (CHARGING | IDLE | DOCK)
      QUERY           -> 대기열에 다음 대상 요청
      NONE            -> 할 일 없음
    """
    action: str
    state: str = ''
    why: str = ''
    try_drop: bool = False
    fallback: str = ''


def should_go_charge(load, capacity, battery, return_pct):
    return load >= capacity or battery <= return_pct


def drop_destination(battery_pct, threshold_pct):
    """실은 킥보드의 하역 장소. 배터리를 모르면(보고 없음) 주차구역."""
    return CHARGER if battery_pct is not None and battery_pct < threshold_pct else PARKING


def decide_tick(state, battery, p, waiting_items, state_age, collect_active, abort_requested, must_return=False):
    """0.2 s 마다. state_age = 현재 상태에 머문 시간(s).
    must_return: 지금 위치에서 충전소까지 갈 배터리만 남음 (must_head_home) -> 20% 전이라도 수거를 끊고 복귀."""
    if state == SELECTING:
        return Decision('SELECT')
    if state == IDLE:
        if battery <= p.return_battery_pct:
            return Decision('SET', CHARGING, f'(대기 중 배터리 {battery:.0f}%)')
        if waiting_items > 0:
            return Decision('SET', SELECTING, '(새 요청)')
    elif state in (NAVIGATING, VERIFYING):          # 적재(LOADING) 중에는 끊지 않음
        if (battery <= p.return_battery_pct or must_return) and collect_active and not abort_requested:
            return Decision('ABORT_COLLECT')
    elif state == FAILED:
        if state_age > FAILED_RETRY_SEC:
            return Decision('SET', SELECTING)
    elif state == CHARGING:
        if battery >= p.charge_target_pct:
            return Decision('SET', SELECTING, f'(충전 {battery:.0f}%)')
    return Decision('NONE')


def decide_select(at_dock, cargo, battery, p, dock_unload_all=False, drop_when_low=False):
    """SELECTING 에서 할 일.
    drop_when_low: 배터리가 부족해도 (충전소 밖이면) 주차구역 하역을 먼저 시도한다.
    하역하고 충전소까지 갈 배터리가 되는지는 하역 자리를 고른 뒤 drop_cost 로 확인한다 (안 되면 그냥 복귀)."""
    if at_dock and (cargo.for_dest(CHARGER) or (dock_unload_all and cargo.load)):
        return Decision('UNLOAD_AT_DOCK')
    low = battery <= p.return_battery_pct
    if low or cargo.load >= p.capacity:
        why = f'(배터리 {battery:.0f}%)' if low else '(만재)'
        # drop_when_low 가 아니면: 배터리가 부족할 때 주차구역행 킥보드를 실은 채 먼저 충전하고, 충전 후 하역한다
        drop_ok = not low or (drop_when_low and not at_dock)
        return Decision('RETURN', why=why, try_drop=drop_ok and bool(cargo.for_dest(PARKING)),
                        fallback=CHARGING if at_dock else 'DOCK')
    return Decision('QUERY')


def decide_queue_empty(at_dock, cargo):
    """대기열에 고를 대상이 없을 때."""
    return Decision('RETURN', why='(대기열 빈)', try_drop=bool(cargo.for_dest(PARKING)),
                    fallback=IDLE if at_dock else 'DOCK')


def drop_next_points(queue_empty, waiting_xy, dock_xy):
    """주차구역 하역 후 갈 곳 후보 (하역 자리 고르기에 씀).
    대기열이 비어 하역하면 충전소로 돌아가고, 만재로 하역하면 대기 중인 킥보드 중 하나로 간다.
    대기 중인 킥보드 위치를 모르면 충전소. 충전소 위치도 모르면 [] (-> 로봇에서 가장 가까운 자리)."""
    if not queue_empty and waiting_xy:
        return list(waiting_xy)
    return [dock_xy] if dock_xy is not None else []


@dataclass
class EnergyModel:
    """배터리 소모 예측. battery_manager 와 같은 식: 이동거리 x k1 + 경과시간 x k2.
    거리는 직선거리 x detour (실제 경로는 돌아가므로). 경로 거리를 알게 되면 dist 함수만 바꿔 넣는다.
    kinematic s4 실측: 충전소 -> 6.2 m 떨어진 킥보드 수거 5.3 %, 복귀 5.5 % -> detour 1.2 이면 조금 넉넉한 쪽."""
    k1_pct_per_m: float = 1.0
    k2_pct_per_s: float = 0.02
    detour: float = 1.2
    speed_mps: float = 0.2          # 평균 이동 속도 (nav v_max 0.22)
    collect_sec: float = 40.0       # 확인·정렬·적재 (Gazebo 팔 약 25 s)
    unload_sec: float = 25.0        # 1대 하역 (Gazebo 팔 약 21 s)
    reserve_pct: float = 5.0        # 충전소에 도착했을 때 남겨 둘 배터리

    def travel(self, d_m):
        path = d_m * self.detour
        return self.k1_pct_per_m * path + self.k2_pct_per_s * path / self.speed_mps

    def work(self, sec):
        return self.k2_pct_per_s * sec


def straight(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def home_cost(m, xy, chargers, dist=straight):
    """xy 에서 가장 가까운 충전소까지 가는 데 드는 배터리(%). 충전소를 모르면 0."""
    return min(m.travel(dist(xy, c)) for c in chargers) if chargers else 0.0


def collect_cost(m, robot_xy, target_xy, chargers, dist=straight):
    """대상까지 가서 싣고, 거기서 가장 가까운 충전소까지 가는 데 드는 배터리(%)."""
    return m.travel(dist(robot_xy, target_xy)) + m.work(m.collect_sec) + home_cost(m, target_xy, chargers, dist)


def drop_cost(m, robot_xy, stand_xy, n, chargers, dist=straight):
    """하역 위치까지 가서 n 대 내리고, 거기서 가장 가까운 충전소까지 가는 데 드는 배터리(%)."""
    return m.travel(dist(robot_xy, stand_xy)) + m.work(m.unload_sec * n) + home_cost(m, stand_xy, chargers, dist)


def affordable(battery, cost, m):
    """cost 만큼 쓰고도 충전소 도착 때 reserve 이상 남는가."""
    return battery - cost >= m.reserve_pct


def must_head_home(battery, m, robot_xy, chargers, dist=straight):
    """지금 바로 충전소로 가야 reserve 를 지킬 수 있는가 (이동 중 확인)."""
    return bool(chargers) and battery - home_cost(m, robot_xy, chargers, dist) < m.reserve_pct


def charge_needed(cost, m):
    """cost 짜리 일을 하려면 충전소에서 몇 % 까지 채워야 하나. 100 을 넘으면 한 번 충전으로는 못 함."""
    return cost + m.reserve_pct


# collector 실패 사유 앞부분 -> 관리자 알림 종류 (앞에서부터 처음 맞는 것)
FAILURE_KINDS = (('nav', 'nav_failed'), ('verify', 'verify_failed'), ('align', 'align_failed'), ('load', 'load_failed'))


def classify_failure(reason, admin_alerts):
    """수거 실패 사유 -> (대기열 상태, 알림 종류). 이동 실패는 사람이 봐야 하는 NEEDS_HUMAN (알림 켜졌을 때).
    알림 종류는 사유 앞부분으로 정한다 (예전에는 이동 실패가 아니면 전부 verify_failed 로 보냈다:
    'align_failed: nav failed ...' 도 태그 확인 실패로 보고됐음). 모르는 사유는 collect_failed."""
    is_nav = reason.startswith('nav')
    status = NEEDS_HUMAN if (is_nav and admin_alerts) else FAILED
    kind = next((k for prefix, k in FAILURE_KINDS if reason.startswith(prefix)), 'collect_failed')
    return status, kind


def dock_should_stop(fail_count, max_retries):
    """도킹 실패 누적 횟수가 한계에 닿으면 정지(NEEDS_HUMAN), 아니면 재시도."""
    return fail_count >= max_retries


class WaitGuard:
    """서비스 응답·액션 결과를 기다리는 일 하나를 추적한다 (미션은 한 번에 하나만 기다린다).

    begin 으로 시작하면 token 을 준다. 제한 시간이 지나면 expired 가 그 종류를 한 번 돌려주고 대기를 지운다.
    그 뒤 늦게 도착한 응답은 end(token) 이 False 라서 버린다 -> 시간 초과로 이미 복구한 일을 다시 처리하지 않는다.
    """

    def __init__(self):
        self.token = 0
        self.kind = None
        self.deadline = None

    def begin(self, kind, now, timeout):
        self.token += 1
        self.kind, self.deadline = kind, now + timeout
        return self.token

    def is_current(self, token):
        return token == self.token and self.kind is not None

    def end(self, token):
        """이 token 의 대기를 끝낸다. 이미 시간 초과됐거나 다른 대기로 바뀐 token 이면 False (응답을 버려야 함)."""
        if not self.is_current(token):
            return False
        self.kind = self.deadline = None
        return True

    def expired(self, now):
        """제한 시간이 지난 대기의 종류 (한 번만). 없으면 None."""
        if self.kind is not None and now > self.deadline:
            kind, self.kind, self.deadline = self.kind, None, None
            return kind
        return None


class Cargo(list):
    """적재함: [(킥보드 id, 목적지)] 실은 순서대로.

    list 를 그대로 상속한다: mission_manager_campus / _station / extended_runtime 은 예전처럼
    self.cargo 를 리스트로 쓴다 (append, 반복, `if self.cargo`, c[0]).
    remove 는 list.remove(값) 와 달리 킥보드 id 로 지운다 (예전 mission_manager 동작).
    """

    @property
    def load(self):
        return len(self)

    def add(self, sid, dest):
        self.append((sid, dest))

    def for_dest(self, dest):
        return [sid for sid, d in self if d == dest]

    def ids_for_dock(self, unload_all):
        """충전소에서 내릴 킥보드: 충전소행, unload_all 이면 전부."""
        return [sid for sid, d in self if d == CHARGER or unload_all]

    def remove(self, sid):
        self[:] = [c for c in self if c[0] != sid]

    def redirect_to_charger(self, sid):
        """주차구역에 못 내린 킥보드는 충전소에서 내린다."""
        self[:] = [(s, CHARGER if s == sid else d) for s, d in self]


def parking_jobs(cargo, spots):
    """주차구역 하역 작업 [(id, (x, y, yaw), 'park')]. 나중에 실은 것(앞 칸)부터 -> 팔이 다른 킥보드 위로 안 지나감."""
    parking = list(reversed(cargo.for_dest(PARKING)))
    return [(sid, spot, 'park') for sid, spot in zip(parking, spots)]


class ChargerSlots:
    """충전소 충전 자리 배정. 자리가 다 차 있으면 가장 오래 맡긴 킥보드를 '충전 끝 -> 치움' 처리하고 그 자리를 쓴다."""

    def __init__(self, spots):
        self.spots = list(spots)                 # [(x, y, yaw)]
        self.occ = [None] * len(self.spots)      # 자리별로 맡긴 킥보드 id (None = 빈 자리)
        self.fifo = []                           # 맡긴 순서 (자리 index)

    def plan_unload(self, ids):
        """ids 를 내릴 작업 목록 [(id, 자리 또는 None, 'charger' | 'charged' | 'remove')]. 나중에 실은 것부터."""
        jobs = []
        for sid in reversed(ids):
            if not self.spots:
                jobs.append((sid, None, 'remove'))
                continue
            if None in self.occ:
                idx = self.occ.index(None)
            else:
                idx = self.fifo.pop(0)
                jobs.append((self.occ[idx], None, 'charged'))
            self.occ[idx] = sid
            self.fifo.append(idx)
            jobs.append((sid, self.spots[idx], 'charger'))
        return jobs

    def release(self, sid):
        """충전 자리에 못 세운 킥보드의 자리 배정을 취소."""
        if sid in self.occ:
            idx = self.occ.index(sid)
            self.occ[idx] = None
            self.fifo.remove(idx)


# ---------------- 출발한 요청의 사유 해소 (현장 해소) ----------------
class ResolveWatch:
    """수거하러 가는 동안 대상 킥보드의 펌웨어 보고(/scooter_states, 1 Hz)를 보고 "요청 사유가 사라졌다"를 판단한다.

    사라짐 = 대기열 자동 취소(B)와 같은 정상 조건: 기울기 <= tilt_th, 구역(슬롯) 안, 고장 없음, 배터리 >= batt_th.
    배터리가 기준보다 낮아도 충전 중(charge_window 초 동안 charge_rise % 넘게 꾸준히 오름)이면 정상으로 본다.
    보고 한 번이 튀어 취소하지 않게 연속 confirm 번 정상이어야 확정 (사용자가 잠깐 들었다 놓는 경우, 센서 잡음)."""

    def __init__(self, tilt_th=45.0, batt_th=20.0, confirm=2, charge_window=20.0, charge_rise=0.3):
        self.tilt_th, self.batt_th, self.confirm = tilt_th, batt_th, max(1, int(confirm))
        self.charge_window, self.charge_rise = charge_window, charge_rise
        self.streak = 0
        self.batt = []                  # [(t, %)] charge_window 동안
        self.why = ''

    def charging(self):
        """충전 중인가: 창 길이의 80 % 이상 쌓인 기록에서 처음 -> 마지막이 charge_rise 넘게 오르고, 중간에 내려간 적 없음."""
        if len(self.batt) < 3 or self.batt[-1][0] - self.batt[0][0] < 0.8 * self.charge_window:
            return False
        drops = any(b2 < b1 - 0.05 for (_, b1), (_, b2) in zip(self.batt, self.batt[1:]))
        return not drops and self.batt[-1][1] - self.batt[0][1] >= self.charge_rise

    def update(self, t, tilt_deg, in_zone, battery_pct, fault=''):
        """보고 하나를 넣고, 사유가 사라졌다고 확정됐는지 돌려준다."""
        self.batt = [(tt, b) for tt, b in self.batt if t - tt <= self.charge_window] + [(t, battery_pct)]
        if fault:
            ok, why = False, ''
        elif tilt_deg > self.tilt_th or not in_zone:
            ok, why = False, ''
        elif battery_pct >= self.batt_th:
            ok, why = True, '바르게 서 있고 구역 안'
        elif self.charging():
            ok, why = True, f'충전 중 (배터리 {self.batt[0][1]:.1f} -> {battery_pct:.1f} %)'
        else:
            ok, why = False, ''
        self.streak = self.streak + 1 if ok else 0
        if ok:
            self.why = why
        return self.resolved

    @property
    def resolved(self):
        return self.streak >= self.confirm


# ---------------- 도크에서 출발: 확인하며 후진 ----------------
UNDOCK_BODY_X0 = -0.98          # 차체 뒤끝 (base_footprint 기준, collector_logic.BODY_X 와 같게)
UNDOCK_HALF_W = 0.55            # 차체 반폭 0.525 + 여유 (차고 문 반폭 0.70, 지도 격자 번짐 고려)


def scan_to_body_points(ranges, angle_min, angle_inc, range_min, range_max, off_x=0.80,
                        body=(-0.98, 0.96, 0.58)):
    """LaserScan 거리값 -> 로봇 기준(base_footprint) 점 [(x, y)]. 측정 끝값·차체 안 점(자기 몸)은 뺀다."""
    pts = []
    for k, r in enumerate(ranges):
        if not (range_min <= r < range_max - 0.05) or r != r:
            continue
        a = angle_min + angle_inc * k
        x, y = off_x + r * math.cos(a), r * math.sin(a)
        if body[0] < x < body[1] and abs(y) < body[2]:
            continue
        pts.append((x, y))
    return pts


def rear_clear(points, dist, margin=0.3, body_x0=UNDOCK_BODY_X0, half_w=UNDOCK_HALF_W):
    """뒤로 dist m 물러날 통로(차체 뒤끝에서 dist + margin, 폭 ±half_w)에 점이 하나도 없으면 True."""
    x_lo = body_x0 - max(0.0, dist) - margin
    return not any(x_lo <= x < body_x0 and abs(y) < half_w for x, y in points)


# ---------------- 충전소(차고) 안인가 / 똑바로 후진 ----------------
STATION_REGION = (-0.45, 2.62, 0.88)   # 차고 영역 (스테이션 좌표: 램프 앞끝 x, 뒷벽 x, 반폭). +x = 안쪽
BODY = (-0.98, 0.96, 0.525)            # 로봇 차체 (x 최소, x 최대, 반폭) base_footprint 기준 (램프 포함)


def to_station(station, x, y):
    """world 점 -> 스테이션 좌표 (sx, sy)"""
    c, s = math.cos(station[2]), math.sin(station[2])
    dx, dy = x - station[0], y - station[1]
    return c * dx + s * dy, -s * dx + c * dy


def body_in_station(pose, station, margin=0.2, region=STATION_REGION, body=BODY):
    """로봇 차체 네 모서리·중심 중 하나라도 차고 영역(+ margin) 안이면 True. 다 나와야 False"""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x0, x1, hw = region
    for bx, by in ((body[0], -body[2]), (body[0], body[2]), (body[1], -body[2]), (body[1], body[2]), (0.0, 0.0)):
        sx, sy = to_station(station, pose[0] + c * bx - s * by, pose[1] + s * bx + c * by)
        if x0 - margin <= sx <= x1 + margin and abs(sy) <= hw + margin:
            return True
    return False


def drop_station_walls(points, pose, station, region=STATION_REGION, inner_half=0.30, margin=0.25):
    """로봇 기준 점 [(x, y)] 중 충전소(차고) 자기 옆벽에 찍힌 점을 뺀다 (도크 후진 판단용).
    지도 격자(0.25 m)가 비스듬한 차고 벽을 계단처럼 그려 벽 칸이 후진 통로(±0.55 m) 안으로 들어오면
    rear_clear 가 늘 '뒤에 장애물'로 보고 출발하지 못했다 (k04, 차고 방향 168.7°).
    차고 영역(앞끝 - margin ~ 뒷벽 + margin) 안에서 축에서 inner_half 이상 떨어진 점 = 옆벽으로 본다.
    번진 벽 칸은 축에서 0.40 m 까지 들어온다 (k08, k21. 차체 반폭 0.525 보다 안쪽이라 실제 물체일 수 없음).
    차고 문 가운데(|sy| < inner_half)와 차고 밖의 점은 그대로 둔다."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x0, x1, _ = region
    out = []
    for bx, by in points:
        sx, sy = to_station(station, pose[0] + c * bx - s * by, pose[1] + s * bx + c * by)
        if x0 - margin <= sx <= x1 + margin and abs(sy) >= inner_half:
            continue
        out.append((bx, by))
    return out


def undock_twist(pose, station, speed, k_y=0.6, k_yaw=1.5, max_tilt=0.2, w_max=0.4):
    """차고 축(스테이션 +x)을 따라 똑바로 후진하는 (v, w). 방향은 안쪽(스테이션 yaw)을 본 채로,
    옆으로 벗어나면(sy) 살짝 틀어 축으로 돌아온다 (후진이라 sy 를 줄이려면 차체를 sy 쪽으로 기울임)."""
    v = -abs(speed)
    _, sy = to_station(station, pose[0], pose[1])
    err_yaw = math.atan2(math.sin(pose[2] - station[2]), math.cos(pose[2] - station[2]))
    want = max(-max_tilt, min(max_tilt, k_y * sy))
    w = max(-w_max, min(w_max, k_yaw * (want - err_yaw)))
    return v, w
