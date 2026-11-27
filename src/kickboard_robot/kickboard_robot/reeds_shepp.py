"""Reeds-Shepp 곡선: 앞·뒤로 가는 자동차(최소 회전 반경 R)가 두 자세를 잇는 경로들 (ROS 무관).

J. Reeds, L. Shepp, "Optimal paths for a car that goes both forwards and backwards" (1990) 의 식 8.1 ~ 8.11 과
좌우·시간 대칭. 경로 = [(조향, 길이), ...]: 조향 +1 좌 / 0 직진 / -1 우, 길이 > 0 전진, < 0 후진.
paths() 는 모든 후보를 돌려준다 (충돌 검사와 비용 선택은 계획기가 함).
"""
import math

PI = math.pi
HALF_PI = PI / 2
EPS = 1e-9


def _mod(a):
    a = math.fmod(a, 2 * PI)
    if a < -PI:
        a += 2 * PI
    elif a > PI:
        a -= 2 * PI
    return a


def _polar(x, y):
    return math.hypot(x, y), math.atan2(y, x)


def _tau_omega(u, v, xi, eta, phi):
    delta = _mod(u - v)
    a = math.sin(u) - math.sin(delta)
    b = math.cos(u) - math.cos(delta) - 1
    t1 = math.atan2(eta * a - xi * b, xi * a + eta * b)
    t2 = 2 * (math.cos(delta) - math.cos(v) - math.cos(u)) + 3
    tau = _mod(t1 + PI) if t2 < 0 else _mod(t1)
    return tau, _mod(tau - u + v - phi)


# 아래 함수들: 단위 반경, 출발 (0, 0, 0) -> (x, y, phi). 해가 있으면 [(조향, 길이)] 를, 없으면 None
def _lsl(x, y, phi):
    u, t = _polar(x - math.sin(phi), y - 1 + math.cos(phi))
    if t >= -EPS:
        v = _mod(phi - t)
        if v >= -EPS:
            return [(1, t), (0, u), (1, v)]
    return None


def _lsr(x, y, phi):
    u1, t1 = _polar(x + math.sin(phi), y - 1 - math.cos(phi))
    if u1 * u1 >= 4:
        u = math.sqrt(u1 * u1 - 4)
        t = _mod(t1 + math.atan2(2, u))
        v = _mod(t - phi)
        if t >= -EPS and v >= -EPS:
            return [(1, t), (0, u), (-1, v)]
    return None


def _lrl(x, y, phi):
    u1, theta = _polar(x - math.sin(phi), y - 1 + math.cos(phi))
    if u1 <= 4:
        u = -2 * math.asin(u1 / 4)
        t = _mod(theta + u / 2 + PI)
        v = _mod(phi - t + u)
        if t >= -EPS and u <= EPS:
            return [(1, t), (-1, u), (1, v)]
    return None


def _lrlr_up_um(x, y, phi):
    xi, eta = x + math.sin(phi), y - 1 - math.cos(phi)
    rho = (2 + math.hypot(xi, eta)) / 4
    if rho <= 1:
        u = math.acos(rho)
        t, v = _tau_omega(u, -u, xi, eta, phi)
        if t >= -EPS and v <= EPS:
            return [(1, t), (-1, u), (1, -u), (-1, v)]
    return None


def _lrlr_um_um(x, y, phi):
    xi, eta = x + math.sin(phi), y - 1 - math.cos(phi)
    rho = (20 - xi * xi - eta * eta) / 16
    if 0 <= rho <= 1:
        u = -math.acos(rho)
        if u >= -HALF_PI:
            t, v = _tau_omega(u, u, xi, eta, phi)
            if t >= -EPS and v >= -EPS:
                return [(1, t), (-1, u), (1, u), (-1, v)]
    return None


def _lrsl(x, y, phi):
    xi, eta = x - math.sin(phi), y - 1 + math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2:
        r = math.sqrt(rho * rho - 4)
        u = 2 - r
        t = _mod(theta + math.atan2(r, -2))
        v = _mod(phi - HALF_PI - t)
        if t >= -EPS and u <= EPS and v <= EPS:
            return [(1, t), (-1, -HALF_PI), (0, u), (1, v)]
    return None


def _lrsr(x, y, phi):
    xi, eta = x + math.sin(phi), y - 1 - math.cos(phi)
    rho, theta = _polar(-eta, xi)
    if rho >= 2:
        t = theta
        u = 2 - rho
        v = _mod(t + HALF_PI - phi)
        if t >= -EPS and u <= EPS and v <= EPS:
            return [(1, t), (-1, -HALF_PI), (0, u), (-1, v)]
    return None


def _lrslr(x, y, phi):
    xi, eta = x + math.sin(phi), y - 1 - math.cos(phi)
    rho = math.hypot(xi, eta)
    if rho >= 2:
        u = 4 - math.sqrt(rho * rho - 4)
        if u <= EPS:
            t = _mod(math.atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta))
            v = _mod(t - phi)
            if t >= -EPS and v >= -EPS:
                return [(1, t), (-1, -HALF_PI), (0, u), (1, -HALF_PI), (-1, v)]
    return None


# (식, 거꾸로(도착에서 출발 쪽으로 푼) 변형도 쓰는가)
_WORDS = ((_lsl, False), (_lsr, False), (_lrl, True), (_lrlr_up_um, False), (_lrlr_um_um, False),
          (_lrsl, True), (_lrsr, True), (_lrslr, False))


def _unit_paths(x, y, phi):
    xb = x * math.cos(phi) + y * math.sin(phi)
    yb = x * math.sin(phi) - y * math.cos(phi)
    for f, backwards in _WORDS:
        for bx, by, rev in ((x, y, False), (xb, yb, True)):
            if rev and not backwards:
                continue
            for flip, mirror in ((1, 1), (-1, 1), (1, -1), (-1, -1)):     # 시간 뒤집기(후진), 좌우 거울
                segs = f(flip * bx, mirror * by, flip * mirror * phi)
                if segs is None:
                    continue
                segs = [(s * mirror, length * flip) for s, length in segs]
                yield segs[::-1] if rev else segs


def end_pose(q0, segs, R):
    """q0 에서 경로 segs 를 따라간 끝 자세"""
    x, y, th = q0
    for s, length in segs:
        if s == 0:
            x += length * math.cos(th)
            y += length * math.sin(th)
        else:
            k = s / R
            nth = th + k * length
            x += (math.sin(nth) - math.sin(th)) / k
            y -= (math.cos(nth) - math.cos(th)) / k
            th = nth
    return x, y, math.atan2(math.sin(th), math.cos(th))


def paths(q0, q1, R):
    """q0 = (x, y, yaw) -> q1 의 Reeds-Shepp 후보들 [[(조향, 길이 m)], ...] (끝이 실제로 q1 인 것만)"""
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]
    c, s = math.cos(q0[2]), math.sin(q0[2])
    x, y = (c * dx + s * dy) / R, (-s * dx + c * dy) / R
    phi = _mod(q1[2] - q0[2])
    out = []
    for unit in _unit_paths(x, y, phi):
        segs = [(st, length * R) for st, length in unit if abs(length) > 1e-6]
        ex, ey, eth = end_pose(q0, segs, R)
        if math.hypot(ex - q1[0], ey - q1[1]) < 1e-3 * R and abs(_mod(eth - q1[2])) < 1e-3:
            out.append(segs)
    return out


def cost(segs, reverse_penalty=1.0, switch_penalty=0.0):
    """길이 + 후진 배수 + 전·후진 전환 벌점"""
    total, prev = 0.0, 0
    for _, length in segs:
        d = 1 if length > 0 else -1
        total += abs(length) * (reverse_penalty if d < 0 else 1.0) + (switch_penalty if prev and d != prev else 0.0)
        prev = d
    return total
