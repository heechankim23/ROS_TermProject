"""ROS-independent policies for the opt-in extended scenario profile."""
import math


def admission(scooter, cargo_ids, weights, limit):
    if scooter.get('damaged', False):
        return 'NEEDS_HUMAN'
    mass = float(scooter.get('weight_kg', 18.0))
    if not math.isfinite(mass) or mass <= 0 or mass > limit:
        return 'NEEDS_HUMAN'
    loaded = sum(weights.get(sid, 18.0) for sid in cargo_ids)
    return 'UNLOAD_FIRST' if loaded + mass > limit else 'COLLECT'


def ramp(previous, target, accel, dt):
    return previous + max(-accel * dt, min(accel * dt, target - previous))


def footprint_free(omap, pose, margin=0.04):
    """Check the entire 1.7 x 0.9 m rectangle, including interior grid cells."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    nx = math.ceil((1.7 + 2 * margin) / (omap.res / 2))
    ny = math.ceil((0.9 + 2 * margin) / (omap.res / 2))
    for i in range(nx + 1):
        lx = -0.85 - margin + (1.7 + 2 * margin) * i / nx
        for j in range(ny + 1):
            ly = -0.45 - margin + (0.9 + 2 * margin) * j / ny
            wx, wy = x + c * lx - s * ly, y + s * lx + c * ly
            if not (omap.ox <= wx < omap.ox + omap.w * omap.res and
                    omap.oy <= wy < omap.oy + omap.h * omap.res):
                return False
            if not omap.is_free(wx, wy, 0):
                return False
    return True


def integrate(pose, v, w, dt):
    x, y, a = pose
    if abs(w) < 1e-8:
        return x + v * math.cos(a) * dt, y + v * math.sin(a) * dt, a
    b = a + w * dt
    return x + v / w * (math.sin(b) - math.sin(a)), y - v / w * (math.cos(b) - math.cos(a)), b


def scan_collision(points, v, w, horizon=0.8):
    """Sweep padded footprint in scan/base frame; points inside the body are self returns."""
    points = [(x, y) for x, y in points if not (-0.92 < x < 0.89 and abs(y) < 0.50)]
    for k in range(1, 9):
        x, y, a = integrate((0, 0, 0), v, w, horizon * k / 8)
        c, s = math.cos(a), math.sin(a)
        for px, py in points:
            dx, dy = px - x, py - y
            if abs(c * dx + s * dy) <= 1.00 and abs(-s * dx + c * dy) <= 0.62:
                return True
    return False
