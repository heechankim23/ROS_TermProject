"""battery_manager 가공부: 가상 배터리 모델 (ROS 없음 -> 단위 테스트·벤치마크 가능).

소모 = 이동거리 x k1 + 경과시간 x k2. CHARGING 에서만 충전, IDLE·NEEDS_HUMAN 은 변화 없음.
"""
import math

# CHARGING 에서만 충전. IDLE(충전소 대기)은 소모도 충전도 없음 -> 시나리오 시작 배터리가 그대로 유지된다
NO_DRAIN_STATES = ('CHARGING', 'IDLE', 'NEEDS_HUMAN')   # NEEDS_HUMAN: 정지 상태
TELEPORT_M = 1.0                                         # 한 번에 이보다 많이 움직이면 리셋으로 보고 무시


class BatteryModel:
    def __init__(self, initial_pct=100.0, k1_pct_per_m=1.0, k2_pct_per_s=0.02, charge_rate_pct_per_s=2.0):
        self.pct = float(initial_pct)
        self.k1, self.k2, self.charge_rate = k1_pct_per_m, k2_pct_per_s, charge_rate_pct_per_s
        self.last_xy = None
        self.dist_acc = 0.0
        self.last_t = None

    def add_odom(self, x, y):
        """오도메트리 위치 1개 반영 (이동거리 누적)."""
        if self.last_xy is not None:
            d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            if d < TELEPORT_M:
                self.dist_acc += d
        self.last_xy = (x, y)

    def update(self, t, state):
        """시각 t(초)에서 로봇 상태 state 로 잔량 갱신. 갱신된 잔량(%)을 돌려준다."""
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        if state == 'CHARGING':
            self.pct += self.charge_rate * dt
        elif state not in NO_DRAIN_STATES:
            self.pct -= self.k1 * self.dist_acc + self.k2 * dt
        self.dist_acc = 0.0
        self.pct = min(100.0, max(0.0, self.pct))
        return self.pct

    def is_low(self, return_pct):
        return self.pct <= return_pct
