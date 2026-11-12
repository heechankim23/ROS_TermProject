"""F4: 대기열 점수식과 선택 로직 (ROS 의존성 없음 -> 단위 테스트 가능).

S_i = w_r * R_i + w_t * t_i - w_d * d_i
"""
import math
from dataclasses import dataclass, field

FALLEN, OUT_OF_ZONE, LOW_BATTERY = 1, 2, 3
REASON_NAMES = {FALLEN: 'FALLEN', OUT_OF_ZONE: 'OUT_OF_ZONE', LOW_BATTERY: 'LOW_BATTERY'}
REASON_WEIGHT = {FALLEN: 3.0, OUT_OF_ZONE: 2.0, LOW_BATTERY: 1.0}

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


def score(item, now, robot_xy, w):
    wait = max(0.0, now - item.t_enqueued)
    d = math.hypot(item.x - robot_xy[0], item.y - robot_xy[1])
    return w.w_r * REASON_WEIGHT.get(item.reason, 1.0) + w.w_t * wait - w.w_d * d


def starvation_bound(w, d_max):
    """대기 시간이 이 값을 넘으면 나중에 들어온 어떤 항목에도 밀리지 않는다 (PRD 4장)."""
    dr_max = max(REASON_WEIGHT.values()) - min(REASON_WEIGHT.values())
    return (w.w_r * dr_max + w.w_d * d_max) / w.w_t if w.w_t > 0 else float('inf')


def ranked(items, now, robot_xy, w):
    """WAITING 항목을 점수 내림차순으로. 동점이면 먼저 들어온 순."""
    cand = [i for i in items if i.status == WAITING]
    return sorted(cand, key=lambda i: (-score(i, now, robot_xy, w), i.t_enqueued))


def queue_position(items, item_id, now, robot_xy, w):
    order = ranked(items, now, robot_xy, w)
    for k, it in enumerate(order):
        if it.id == item_id:
            return k + 1
    return 0
