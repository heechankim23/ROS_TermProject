"""queue_manager 가공부 제안: 대기열 항목과 그 위의 연산 (병합·선택·상태 변경·취소). ROS 없음.

queue_logic.py(B 담당)는 고치지 않고 그 함수들을 가져다 쓴다. queue_manager 가 아직 이 파일을 쓰지는 않는다
(팀 합의 후 연결 — CONTRIBUTING.md, docs/PIPELINE.md).
"""
from .queue_logic import (ACTIVE, CANCELLED, DONE, FAILED, IN_PROGRESS, NEEDS_HUMAN, WAITING, Item,  # noqa: F401
                          queue_position, ranked, score)


def is_normal(tilt_deg, in_zone, battery_pct, tilt_threshold_deg, battery_threshold_pct):
    """F13 자동 취소 조건: 서 있고, 주차구역 안이고, 배터리가 충분하면 정상 킥보드."""
    return tilt_deg <= tilt_threshold_deg and in_zone and battery_pct >= battery_threshold_pct


class QueueBook:
    """queue_manager 가공부: 대기열 항목과 그 위의 연산 (병합·선택·상태 변경·취소). ROS 없음.

    가중치(w)는 SetConfig 로 바뀔 수 있어서 호출할 때마다 받는다.
    """

    def __init__(self):
        self.items = {}            # id -> Item

    def enqueue(self, sid, reason, x, y, lat, lon, now):
        """새 요청. 같은 ID 가 대기·처리 중이면 병합(사유만 갱신, 대기 중이면 위치도 갱신). 병합했으면 True."""
        it = self.items.get(sid)
        if it is not None and it.status in ACTIVE:
            it.reason = reason
            if it.status == WAITING:                     # 처리 중인 항목은 위치를 바꾸지 않음
                it.x, it.y, it.lat, it.lon = x, y, lat, lon
            return True
        self.items[sid] = Item(sid, reason, x, y, lat, lon, now)
        return False

    def position(self, sid, now, robot_xy, w):
        return queue_position(self.items.values(), sid, now, robot_xy, w)

    def select(self, now, robot_xy, w):
        """점수 최고 WAITING 항목을 IN_PROGRESS 로. (항목, 점수, 후보 수) 또는 None"""
        order = ranked(self.items.values(), now, robot_xy, w)
        if not order:
            return None
        it = order[0]
        it.status = IN_PROGRESS
        return it, score(it, now, robot_xy, w), len(order)

    def update(self, sid, status, note='', corrected_xy=None):
        """mission_manager 의 결과 반영. status = REQUEUE | DONE | FAILED | NEEDS_HUMAN | CANCELLED.
        (성공 여부, 항목). REQUEUE 는 t_enqueued 를 유지해 aging 이 이어진다."""
        it = self.items.get(sid)
        if it is None:
            return False, None
        if corrected_xy is not None:
            it.x, it.y = corrected_xy
        if 'nav' in note:
            it.nav_failures += 1
        if status == 'REQUEUE':
            it.status = WAITING
        elif status in (DONE, FAILED, NEEDS_HUMAN, CANCELLED):
            it.status = status
        else:
            return False, it
        return True, it

    def cancel(self, sid):
        """수동 취소. (성공 여부, 메시지, 항목). 처리 중인 항목은 취소하지 않는다(비선점)."""
        it = self.items.get(sid)
        if it is None or it.status not in ACTIVE:
            return False, '대기 중인 항목 없음', None
        if it.status == IN_PROGRESS:
            return False, '처리 중인 항목은 취소할 수 없음(비선점)', it
        it.status = CANCELLED
        return True, '취소됨', it

    def auto_cancel(self, sid, tilt_deg, in_zone, battery_pct, tilt_threshold_deg, battery_threshold_pct):
        """대기 중인 킥보드가 정상으로 돌아왔으면 CANCELLED 로 바꾸고 항목을, 아니면 None."""
        it = self.items.get(sid)
        if it is None or it.status != WAITING:
            return None
        if is_normal(tilt_deg, in_zone, battery_pct, tilt_threshold_deg, battery_threshold_pct):
            it.status = CANCELLED
            return it
        return None

    def rows(self, now, robot_xy, w):
        """발행용 (항목, 대기 s, 점수) 목록. 대기·처리 중을 먼저, 점수 높은 순."""
        out = [(it, now - it.t_enqueued, score(it, now, robot_xy, w)) for it in self.items.values()]
        out.sort(key=lambda r: (r[0].status not in ACTIVE, -r[2]))
        return out
