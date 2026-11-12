"""간단한 평면 이동 제어기 (nav_bridge simple 백엔드, collector 정렬, dock_controller 공용)."""
"test2"
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    """(x, y, z, w)"""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def standoff_point(target_xy, from_xy, dist):
    """target 에서 from 쪽으로 dist 만큼 떨어진 점과, 그 점에서 target 을 바라보는 yaw."""
    dx, dy = from_xy[0] - target_xy[0], from_xy[1] - target_xy[1]
    n = math.hypot(dx, dy)
    if n < 1e-6:
        dx, dy, n = 1.0, 0.0, 1.0
    gx, gy = target_xy[0] + dx / n * dist, target_xy[1] + dy / n * dist
    return gx, gy, math.atan2(target_xy[1] - gy, target_xy[0] - gx)


class GoToController:
    """회전 -> 직진 -> 최종 방향 맞춤. step() 은 (v, w, done) 반환."""

    def __init__(self, k_lin=0.8, k_ang=1.8, v_max=0.22, w_max=1.2,
                 pos_tol=0.05, yaw_tol=math.radians(5), heading_gate=math.radians(25),
                 allow_reverse=False):
        self.k_lin, self.k_ang = k_lin, k_ang
        self.v_max, self.w_max = v_max, w_max
        self.pos_tol, self.yaw_tol = pos_tol, yaw_tol
        self.heading_gate = heading_gate
        self.allow_reverse = allow_reverse

    def step(self, pose, goal, final_yaw=True):
        x, y, th = pose
        gx, gy, gth = goal
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist > self.pos_tol:
            heading = math.atan2(dy, dx)
            err = wrap(heading - th)
            sign = 1.0
            if self.allow_reverse and abs(err) > math.pi / 2 and dist < 0.5:
                err, sign = wrap(err + math.pi), -1.0       # 짧은 거리는 후진으로
            w = _clip(self.k_ang * err, self.w_max)
            v = 0.0 if abs(err) > self.heading_gate else sign * min(self.v_max, self.k_lin * dist)
            v *= max(0.0, math.cos(err))
            return v, w, False
        if final_yaw:
            err = wrap(gth - th)
            if abs(err) > self.yaw_tol:
                return 0.0, _clip(self.k_ang * err, self.w_max), False
        return 0.0, 0.0, True


def _clip(v, lim):
    return max(-lim, min(lim, v))
