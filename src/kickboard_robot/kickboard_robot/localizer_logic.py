"""sim_localizer 가공부: 정답 위치 + odom -> map->odom 보정값 (ROS 없음)."""
import math

JUMP_M = 0.5        # 추정값과 이보다 크게 어긋나면 필터를 거치지 않고 바로 맞춤


def compose_map_odom(truth, odom):
    """truth = map 에서 base (x, y, yaw), odom = odom 에서 base (x, y, yaw) -> map 에서 odom (x, y, yaw)"""
    tx, ty, tth = truth
    ox, oy, oth = odom
    yaw = tth - oth
    c, s = math.cos(yaw), math.sin(yaw)
    return tx - (c * ox - s * oy), ty - (s * ox + c * oy), yaw


class MapOdomFilter:
    """map->odom 은 미끄러질 때만 천천히 변한다. 정답·odom 메시지 시각 차이로 생기는 떨림은 저역 통과로 없앤다."""

    def __init__(self, smoothing=0.1):
        self.smoothing = smoothing
        self.est = None

    def update(self, truth, odom):
        raw = compose_map_odom(truth, odom)
        e = self.est
        if e is None or math.hypot(raw[0] - e[0], raw[1] - e[1]) > JUMP_M:
            self.est = raw                       # 처음 / 크게 어긋났으면 바로 맞춤
        else:
            a = self.smoothing
            dyaw = math.atan2(math.sin(raw[2] - e[2]), math.cos(raw[2] - e[2]))
            self.est = (e[0] + a * (raw[0] - e[0]), e[1] + a * (raw[1] - e[1]), e[2] + a * dyaw)
        return self.est
