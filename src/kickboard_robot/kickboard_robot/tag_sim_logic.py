"""sim_tag_detector 가공부: 카메라 시야 안의 태그를 골라 노이즈 섞인 검출값을 만든다 (ROS 없음).

킥보드는 (id, x, y, yaw) 튜플, 카메라는 (x, y, yaw). 결과 검출값은 (tag_id, x, y, yaw, distance).
"""
import math
import random
from dataclasses import dataclass

from .collector_logic import face_tag_id, tag_id_of
from .gz_utils import SCOOTER_L, SCOOTER_W
from .motion import wrap


@dataclass
class CameraModel:
    hfov_deg: float = 62.2              # Waffle Pi 카메라
    max_range: float = 3.0
    min_range: float = 0.12
    max_view_angle_deg: float = 70.0    # 태그 면을 비스듬히 볼 수 있는 한계
    noise_pos: float = 0.02
    noise_yaw_deg: float = 2.0


def camera_pose(robot_pose, offset_x):
    x, y, yaw = robot_pose
    return x + offset_x * math.cos(yaw), y + offset_x * math.sin(yaw), yaw


def scooter_faces(x, y, yaw):
    """킥보드 네 면의 태그 자리 (x, y, 바깥 법선 yaw)."""
    out = []
    c, s = math.cos(yaw), math.sin(yaw)
    for lx, ly, ny in ((SCOOTER_L / 2, 0, 0), (-SCOOTER_L / 2, 0, math.pi),
                       (0, SCOOTER_W / 2, math.pi / 2), (0, -SCOOTER_W / 2, -math.pi / 2)):
        out.append((x + c * lx - s * ly, y + s * lx + c * ly, yaw + ny))
    return out


def visible(cam, face, model):
    """보이면 (거리, 비스듬한 각 rad), 아니면 None."""
    fx, fy, fyaw = face
    vx, vy = cam[0] - fx, cam[1] - fy
    d = math.hypot(vx, vy)
    if not (model.min_range <= d <= model.max_range):
        return None
    view = abs(wrap(math.atan2(vy, vx) - fyaw))                       # 태그 법선 vs 카메라 방향
    bearing = abs(wrap(math.atan2(fy - cam[1], fx - cam[0]) - cam[2]))  # 카메라 광축 vs 태그
    if view > math.radians(model.max_view_angle_deg) or bearing > math.radians(model.hfov_deg / 2):
        return None
    return d, view


def noisy(tag, face, d, model, rng=random):
    return (tag, face[0] + rng.gauss(0, model.noise_pos), face[1] + rng.gauss(0, model.noise_pos),
            face[2] + rng.gauss(0, math.radians(model.noise_yaw_deg)), d)


def detect(cam, scooters, charger_face, charger_id, model, rng=random):
    """보이는 태그 검출값 목록. 킥보드마다 가장 정면으로 보이는 면 하나만."""
    out = []
    for sid, x, y, yaw in scooters:
        best = None
        for k, f in enumerate(scooter_faces(x, y, yaw)):
            v = visible(cam, f, model)
            if v and (best is None or v[1] < best[1][1]):
                best = (f, v, k)
        if best:     # 면마다 다른 ID (collector 가 킥보드 방향을 앎)
            out.append(noisy(face_tag_id(tag_id_of(sid), best[2]), best[0], best[1][0], model, rng))
    if charger_face is not None:
        v = visible(cam, charger_face, model)
        if v:
            out.append(noisy(charger_id, charger_face, v[0], model, rng))
    return out
