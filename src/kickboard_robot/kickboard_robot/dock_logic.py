"""dock_controller 가공부: 충전소 기하, 도킹 오차, 태그 기반 목표 필터 (ROS 없음).

pose·goal 은 모두 map 기준 (x, y, yaw).
"""
import math

from .motion import wrap


def charger_poses(c):
    """map_config 의 charger 항목 -> (tag, dock, pre_dock) 각각 (x, y, yaw). yaw 는 로봇이 바라볼 방향."""
    tx, ty, tyaw = c['tag_pose']['x'], c['tag_pose']['y'], math.radians(c['tag_pose']['yaw_deg'])
    face = tyaw + math.pi
    dock = (tx + c['dock_distance'] * math.cos(tyaw), ty + c['dock_distance'] * math.sin(tyaw), face)
    pre = (tx + c['pre_dock_distance'] * math.cos(tyaw), ty + c['pre_dock_distance'] * math.sin(tyaw), face)
    return (tx, ty, tyaw), dock, pre


def dock_errors(pose, dock):
    """설정된 도킹 pose 대비 (위치 오차 m, 측면 오차 m, 방향 오차 rad)."""
    dx, dy = pose[0] - dock[0], pose[1] - dock[1]
    lateral = -dx * math.sin(dock[2]) + dy * math.cos(dock[2])
    return math.hypot(dx, dy), lateral, wrap(pose[2] - dock[2])


def within_accept(pose, goal, pos_tol=0.05, yaw_tol_deg=5.0):
    """시간 초과 시 PRD 기준(5 cm, 5°) 안이면 성공으로 친다. (통과 여부, 위치 오차 m, 방향 오차 deg)"""
    pe = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
    ye = math.degrees(wrap(goal[2] - pose[2]))
    return pe <= pos_tol and abs(ye) <= yaw_tol_deg, pe, ye


class DockGoalFilter:
    """충전소 태그 검출로 도킹 목표를 갱신한다.

    검출 노이즈(σ 2 cm)는 저역 통과(alpha)로 줄이고, 목표 lock_dist 안에 들어오면 목표를 고정한다
    (노이즈로 목표가 도는 것 방지). 검출이 없으면 설정된 dock pose 가 목표.
    """

    def __init__(self, default_goal, dock_distance, alpha=0.15, lock_dist=0.10):
        self.goal = default_goal
        self.dist, self.alpha, self.lock_dist = dock_distance, alpha, lock_dist
        self.smooth = None
        self.locked = False

    def check_lock(self, pose):
        """pose 가 목표 lock_dist 안이면 고정. 이번 호출에서 처음 고정됐으면 True."""
        if self.locked or pose is None:
            return False
        if math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1]) < self.lock_dist:
            self.locked = True
            return True
        return False

    def add_detection(self, tx, ty, tyaw):
        """태그 면 pose 1개 반영 (고정된 뒤에는 무시). 갱신된 목표를 돌려준다."""
        if self.locked:
            return self.goal
        raw = (tx + self.dist * math.cos(tyaw), ty + self.dist * math.sin(tyaw), tyaw)
        if self.smooth is None:
            self.smooth = raw
        else:
            a, s = self.alpha, self.smooth
            self.smooth = (s[0] + a * (raw[0] - s[0]), s[1] + a * (raw[1] - s[1]), s[2] + a * wrap(raw[2] - s[2]))
        self.goal = (self.smooth[0], self.smooth[1], self.smooth[2] + math.pi)
        return self.goal
