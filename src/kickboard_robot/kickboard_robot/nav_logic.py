"""nav_bridge 가공부: 라이다 전처리, 목표 지점 선택, 경로 추종 1 스텝 (ROS 없음).

nav_bridge(통신부)는 /scan 을 받아 filter_scan -> Costmap.update_scan 으로 넘기고,
GoToPose 목표를 받으면 choose_goal 로 목표를 정한 뒤 PathFollower.step 을 주기적으로 불러
나온 속도를 /cmd_vel 로 발행한다. 경로 계획 자체는 planner.Costmap, 추종 제어는 motion.GoToController.
"""
import contextlib
import math
from dataclasses import dataclass

import numpy as np

from .motion import GoToController, standoff_point
from .occupancy import free_standoff
from .planner import free_spot

FRONT_HALF_ANGLE_DEG = 20.0      # 안전장치가 보는 전방 범위 (±)


def filter_scan(ranges, angle_min, angle_inc, range_min, offset_x, footprint):
    """라이다는 앞 범퍼 위(x offset_x)에 있어서 뒤쪽 빔은 자기 차체에 맞는다 -> 차체 안에 찍힌 점을 골라낸다.

    footprint = (차체 x 최소, x 최대, |y| 최대) [base_footprint 기준]
    반환: (전방 ±20° 최소 거리, 차체 점 마스크, 거리 배열)
    """
    fx0, fx1, fy = footprint
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    with np.errstate(invalid='ignore'):
        px, py = offset_x + r * np.cos(a), r * np.sin(a)
        own = np.isfinite(r) & (px > fx0) & (px < fx1) & (np.abs(py) < fy)
        front = (np.abs(np.arctan2(np.sin(a), np.cos(a))) < math.radians(FRONT_HALF_ANGLE_DEG)) & ~own \
            & np.isfinite(r) & (r > range_min)
    front_min = float(r[front].min()) if front.any() else float('inf')
    return front_min, own, r


def sensor_pose(robot_pose, offset_x):
    x, y, yaw = robot_pose
    return x + offset_x * math.cos(yaw), y + offset_x * math.sin(yaw), yaw


def choose_goal(target, target_yaw, robot_xy, standoff, clearance, now, cm=None, omap=None):
    """GoToPose 목표 -> 실제로 갈 (x, y, yaw), 대체 지점 사용 여부.

    standoff > 0 이면 target 에서 standoff 떨어진 원 위의 '빈 자리' (로봇 쪽부터 좌우로 넓혀 찾음).
    코스트맵(정적 지도 + 라이다)이 있으면 그걸로, 없으면 정적 지도로 판정. 못 찾으면 직선 offset.
    """
    if standoff <= 0.0:
        return (target[0], target[1], target_yaw), False
    if cm is not None:
        goal = free_spot(cm, target, robot_xy, standoff, clearance, now)
    else:
        goal = free_standoff(omap, target, robot_xy, standoff, clearance)
    if goal is None:
        return standoff_point(target, robot_xy, standoff), True
    return goal, False


@dataclass
class FollowParams:
    v_max: float = 0.22
    w_max: float = 1.2
    pos_tol: float = 0.08
    yaw_tol_deg: float = 8.0
    obstacle_stop_dist: float = 0.15
    blocked_timeout_sec: float = 8.0
    replan_sec: float = 1.0
    lookahead: float = 0.5
    goal_clearance: float = 1.1


@dataclass
class StepResult:
    kind: str            # 'drive' | 'wait' | 'again' | 'arrived' | 'fail'
    v: float = 0.0
    w: float = 0.0
    dist: float = 0.0    # 목표까지 직선거리 (피드백용)
    msg: str = ''        # fail 사유 또는 로그로 남길 메모 (목표 변경 등)


class PathFollower:
    """목표 하나를 향한 경로 추종. step() 을 약 20 Hz 로 부른다.

    kind 별로 통신부가 할 일:
      drive   -> (v, w) 발행 후 0.05 s 대기
      wait    -> 정지 발행 후 0.2 s 대기 (경로 없음, blocked_timeout 까지 기다림)
      again   -> 중간 경로점 도착. 대기 없이 바로 다시 step
      arrived -> 정지, 성공
      fail    -> 정지, 실패 (msg = 'no path' | 'blocked')
    """

    def __init__(self, goal, params, cm=None, lock=None, repick=None):
        """goal = (x, y, yaw). cm = planner.Costmap (없으면 직선 주행). lock = cm 을 다른 스레드와 공유할 때의 락.
        repick = (target(x, y), standoff) — standoff 목표면 주변에 새 장애물이 붙었을 때 자리를 다시 고른다."""
        p = params
        self.goal, self.p, self.cm = goal, p, cm
        self.lock = lock if lock is not None else contextlib.nullcontext()
        self.repick = repick
        self.ctrl = GoToController(v_max=p.v_max, w_max=p.w_max, pos_tol=p.pos_tol, yaw_tol=math.radians(p.yaw_tol_deg))
        self.path, self.planned_at, self.blocked_since = None, -1e9, None

    def step(self, pose, now, front_min=float('inf')):
        p, goal = self.p, self.goal
        dist = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
        note = ''
        if self.cm is not None and dist > p.pos_tol:
            # 목표가 '빈 자리'로 고른 것(standoff)인데 그사이 새 장애물(방금 내려놓은 킥보드 등)이 붙었으면 다시 고른다
            if self.repick is not None and now - self.planned_at > p.replan_sec:
                with self.lock:
                    tight = self.cm.dist_at(goal[0], goal[1], now) < p.goal_clearance - 0.05
                    if tight:
                        target, standoff = self.repick
                        new, _ = choose_goal(target, 0.0, pose[:2], standoff, p.goal_clearance, now, cm=self.cm)
                if tight:
                    note = (f'목표 주변에 새 장애물 -> 목표 변경 ({goal[0]:.2f}, {goal[1]:.2f}) -> '
                            f'({new[0]:.2f}, {new[1]:.2f})')
                    self.goal = goal = new
                    self.path = None
                    dist = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
            with self.lock:
                need = (self.path is None or now - self.planned_at > p.replan_sec
                        or self.cm.path_blocked(self.path, now, pose[:2]))
                if need:
                    self.path = self.cm.plan(pose[:2], goal[:2], now)
                    self.planned_at = now
            if self.path is None:
                self.blocked_since = now if self.blocked_since is None else self.blocked_since
                if now - self.blocked_since > p.blocked_timeout_sec:
                    return StepResult('fail', dist=dist, msg='no path')
                return StepResult('wait', dist=dist, msg=note)
            if not self.path:
                self.path = [goal[:2]]
            while len(self.path) > 1 and math.hypot(self.path[0][0] - pose[0], self.path[0][1] - pose[1]) < p.lookahead:
                self.path.pop(0)
            wp = self.path[0]
        else:
            wp = goal
        last = self.cm is None or len(self.path or []) <= 1
        v, w, done = self.ctrl.step(pose, (wp[0], wp[1], goal[2]), final_yaw=last)
        if done and last:
            return StepResult('arrived', dist=dist, msg=note)
        if done:                                       # 중간 경로점 도착 -> 다음 점
            self.path.pop(0)
            return StepResult('again', dist=dist, msg=note)
        if v > 0 and front_min < p.obstacle_stop_dist:  # 안전장치: 계획에 없던 가까운 장애물
            v = 0.0
            self.blocked_since = now if self.blocked_since is None else self.blocked_since
            if now - self.blocked_since > p.blocked_timeout_sec:
                return StepResult('fail', dist=dist, msg='blocked')
        else:
            self.blocked_since = None
        return StepResult('drive', v=v, w=w, dist=dist, msg=note)
