"""F4: 대기열 점수식과 선택 로직 (ROS 의존성 없음 -> 단위 테스트 가능).

S_i = w_r * R_i + w_t * t_i - w_d * d_i
d_i 는 dist(robot_xy, item) 가 주어지면 실제 주행 경로 길이, 없으면 직선거리.
"""
import math
from dataclasses import dataclass, field

FALLEN, OUT_OF_ZONE, LOW_BATTERY = 1, 2, 3
BROKEN, OBSTRUCTION, REPORTED, MISALIGNED = 4, 5, 6, 7          # 캠퍼스 맵 (고장 · 사용자 신고 · 정렬 불량)
REASON_NAMES = {FALLEN: 'FALLEN', OUT_OF_ZONE: 'OUT_OF_ZONE', LOW_BATTERY: 'LOW_BATTERY', BROKEN: 'BROKEN',
                OBSTRUCTION: 'OBSTRUCTION', REPORTED: 'REPORTED', MISALIGNED: 'MISALIGNED'}
REASON_WEIGHT = {FALLEN: 3.0, OUT_OF_ZONE: 2.0, LOW_BATTERY: 1.0, BROKEN: 2.5, OBSTRUCTION: 4.0, REPORTED: 2.0,
                 MISALIGNED: 1.5}
ALL_REASONS = tuple(REASON_NAMES)
USER_REPORTS = (OBSTRUCTION, REPORTED)                          # 사용자 신고로만 들어오는 사유

WAITING, IN_PROGRESS, DONE = 'WAITING', 'IN_PROGRESS', 'DONE'
FAILED, NEEDS_HUMAN, CANCELLED = 'FAILED', 'NEEDS_HUMAN', 'CANCELLED'
ACTIVE = (WAITING, IN_PROGRESS)


@dataclass
class Item:
    id: str
    reason: int
    x: float
    y: float
    lat: float
    lon: float
    t_enqueued: float               # 최초 접수 시각(s). 병합·재대기 시에도 유지 -> aging 이 이어짐
    status: str = WAITING
    nav_failures: int = 0
    history: list = field(default_factory=list)


@dataclass
class Weights:
    w_r: float = 10.0
    w_t: float = 1.0
    w_d: float = 5.0


def merge_reason(old, new):
    """같은 킥보드 요청을 병합할 때 남길 사유.

    킥보드 판정끼리는 최신 상태를 따른다 (구역 밖 -> 넘어짐, 다시 세워지면 배터리 부족으로 내려감).
    사용자 신고가 끼면 가중치가 높은 쪽을 남긴다: 신고가 넘어짐 판정을 덮어 순위를 내리거나,
    킥보드 판정이 통행 방해 신고를 지우지 않게.
    """
    if old in USER_REPORTS or new in USER_REPORTS:
        return max((old, new), key=lambda r: REASON_WEIGHT.get(r, 0.0))
    return new


def score(item, now, robot_xy, w, dist=None):
    wait = max(0.0, now - item.t_enqueued)
    d = math.hypot(item.x - robot_xy[0], item.y - robot_xy[1]) if dist is None else dist(robot_xy, item)
    return w.w_r * REASON_WEIGHT.get(item.reason, 1.0) + w.w_t * wait - w.w_d * d


def starvation_bound(w, d_max):
    """이 값(초)보다 더 늦게 들어온 항목에는 밀리지 않는다 (PRD 4장).

    먼저 또는 이 시간 안에 들어온 항목에는 밀릴 수 있으므로 '대기 <= t_max 안에 선택' 을 보장하지는 않는다.
    """
    dr_max = max(REASON_WEIGHT.values()) - min(REASON_WEIGHT.values())
    return (w.w_r * dr_max + w.w_d * d_max) / w.w_t if w.w_t > 0 else float('inf')


def ranked(items, now, robot_xy, w, dist=None):
    """WAITING 항목을 점수 내림차순으로. 동점이면 먼저 들어온 순."""
    cand = [i for i in items if i.status == WAITING]
    return sorted(cand, key=lambda i: (-score(i, now, robot_xy, w, dist), i.t_enqueued))


def queue_position(items, item_id, now, robot_xy, w, dist=None):
    order = ranked(items, now, robot_xy, w, dist)
    for k, it in enumerate(order):
        if it.id == item_id:
            return k + 1
    return 0
