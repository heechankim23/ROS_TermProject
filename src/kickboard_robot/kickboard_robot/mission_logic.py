"""mission_manager 가공부: 미션 판단과 적재함·충전 자리 관리 (ROS 없음 -> 단위 테스트·오프라인 실험 가능).

mission_manager(통신부)는 0.2 s 마다 decide_tick 을 부르고, 결과에 따라 서비스·액션을 호출한다.
판단 규칙을 바꾸고 싶으면 이 파일만 고치면 된다 (노드 연결은 그대로).

  decide_tick        : 상태별 다음 행동 (SELECTING 이면 decide_select 로)
  decide_select      : 충전소 하역 / 복귀(주차구역 하역 먼저 시도) / 대기열에 다음 대상 요청
  decide_queue_empty : 대기열이 비었을 때 (주차구역 하역 -> 도크면 IDLE, 아니면 복귀)
  classify_failure   : 수거 실패 사유 -> 대기열 상태(FAILED / NEEDS_HUMAN)와 알림 종류
  Cargo, ChargerSlots: 적재함 내용, 충전 자리 배정
"""
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


@dataclass
class MissionParams:
    capacity: int = 3
    return_battery_pct: float = 20.0
    charge_target_pct: float = 80.0
    scooter_battery_threshold_pct: float = 15.0
    admin_alerts: bool = True
    max_dock_retries: int = 3


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


def decide_tick(state, battery, p, waiting_items, state_age, collect_active, abort_requested):
    """0.2 s 마다. state_age = 현재 상태에 머문 시간(s)."""
    if state == SELECTING:
        return Decision('SELECT')
    if state == IDLE:
        if battery <= p.return_battery_pct:
            return Decision('SET', CHARGING, f'(대기 중 배터리 {battery:.0f}%)')
        if waiting_items > 0:
            return Decision('SET', SELECTING, '(새 요청)')
    elif state in (NAVIGATING, VERIFYING):          # 적재(LOADING) 중에는 끊지 않음
        if battery <= p.return_battery_pct and collect_active and not abort_requested:
            return Decision('ABORT_COLLECT')
    elif state == FAILED:
        if state_age > FAILED_RETRY_SEC:
            return Decision('SET', SELECTING)
    elif state == CHARGING:
        if battery >= p.charge_target_pct:
            return Decision('SET', SELECTING, f'(충전 {battery:.0f}%)')
    return Decision('NONE')


def decide_select(at_dock, cargo, battery, p, dock_unload_all=False):
    """SELECTING 에서 할 일."""
    if at_dock and (cargo.for_dest(CHARGER) or (dock_unload_all and cargo.load)):
        return Decision('UNLOAD_AT_DOCK')
    low = battery <= p.return_battery_pct
    if low or cargo.load >= p.capacity:
        why = f'(배터리 {battery:.0f}%)' if low else '(만재)'
        # 배터리가 부족하면 주차구역행 킥보드를 실은 채 먼저 충전하고, 충전 후 하역한다
        return Decision('RETURN', why=why, try_drop=not low and bool(cargo.for_dest(PARKING)),
                        fallback=CHARGING if at_dock else 'DOCK')
    return Decision('QUERY')


def decide_queue_empty(at_dock, cargo):
    """대기열에 고를 대상이 없을 때."""
    return Decision('RETURN', why='(대기열 빈)', try_drop=bool(cargo.for_dest(PARKING)),
                    fallback=IDLE if at_dock else 'DOCK')


def classify_failure(reason, admin_alerts):
    """수거 실패 사유 -> (대기열 상태, 알림 종류). 이동 실패는 사람이 봐야 하는 NEEDS_HUMAN (알림 켜졌을 때)."""
    is_nav = reason.startswith('nav')
    status = NEEDS_HUMAN if (is_nav and admin_alerts) else FAILED
    return status, ('nav_failed' if is_nav else 'verify_failed')


def dock_should_stop(fail_count, max_retries):
    """도킹 실패 누적 횟수가 한계에 닿으면 정지(NEEDS_HUMAN), 아니면 재시도."""
    return fail_count >= max_retries


class Cargo:
    """적재함: [(킥보드 id, PARKING | CHARGER)] 실은 순서대로."""

    def __init__(self):
        self.items = []

    @property
    def load(self):
        return len(self.items)

    def add(self, sid, dest):
        self.items.append((sid, dest))

    def for_dest(self, dest):
        return [sid for sid, d in self.items if d == dest]

    def ids_for_dock(self, unload_all):
        """충전소에서 내릴 킥보드: 충전소행, unload_all 이면 전부."""
        return [sid for sid, d in self.items if d == CHARGER or unload_all]

    def remove(self, sid):
        self.items = [c for c in self.items if c[0] != sid]

    def redirect_to_charger(self, sid):
        """주차구역에 못 내린 킥보드는 충전소에서 내린다."""
        self.items = [(s, CHARGER if s == sid else d) for s, d in self.items]


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
