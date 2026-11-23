"""collector 가공부: 태그 검출값 -> 킥보드 중심 추정, 도착 판정 (ROS 없음).

검출값은 (x, y, yaw) 튜플로 받는다. yaw = 태그 면의 바깥쪽 법선 방향.
"""
import math


def tag_id_of(scooter_id):
    """킥보드 ID -> AprilTag ID. 숫자 ID 는 그대로, 아니면 1~90 으로 해시."""
    try:
        return int(scooter_id)
    except ValueError:
        return abs(hash(scooter_id)) % 90 + 1


def arrived_by_tag(tag_distance, standoff, margin):
    """이동 중 태그가 standoff + margin 안에서 보이면 접근 완료로 본다 (GPS 오차로 목표가 너무 붙었을 때)."""
    return tag_distance <= standoff + margin


def center_from_tag(tx, ty, tyaw, half_depth):
    """태그 면 pose -> 킥보드 중심 (태그 면에서 법선 반대쪽으로 half_depth 안쪽). (x, y, yaw)"""
    return tx - half_depth * math.cos(tyaw), ty - half_depth * math.sin(tyaw), tyaw


def wrap_angle(a):
    """각도를 -pi ~ pi 로."""
    return math.atan2(math.sin(a), math.cos(a))


def retry_angles(base, offsets_deg, tried, min_sep_deg):
    """정렬 재시도 후보 각도(rad, 킥보드 중심에서 본 로봇 자리의 방향).
    base(처음 실패한 자리의 각도)에 offsets_deg 를 차례로 더한 순서를 지키고,
    이미 시도한 각도(tried)나 앞선 후보와 min_sep_deg 보다 가까운 것은 뺀다 (같은 자리 반복 방지)."""
    sep = math.radians(min_sep_deg) - 1e-9
    out = []
    for off in offsets_deg:
        a = wrap_angle(base + math.radians(off))
        if all(abs(wrap_angle(a - t)) >= sep for t in list(tried) + out):
            out.append(a)
    return out


def align_pose(cx, cy, r, angle):
    """킥보드 중심 (cx, cy) 둘레 반경 r 원 위, 방향 angle 인 자리와 중심을 바라보는 yaw. (x, y, yaw)"""
    return cx + r * math.cos(angle), cy + r * math.sin(angle), wrap_angle(angle + math.pi)


ARM_REACH_NEAR = 0.25      # align_distance 보다 이만큼 가까워도 팔이 닿음
ARM_REACH_SKIP_FAR = 0.15  # 정렬 주행을 생략할 때 허용하는 바깥쪽 여유 (적재 허용 여유 arm_reach_margin 보다 좁게)


def within_reach(r, dist, near, far):
    """킥보드 중심까지 거리 r 이 팔이 닿는 범위 [dist - near, dist + far] 안인가."""
    return dist - near <= r <= dist + far


def mean_center(centers, last_n=None):
    """중심 추정값들의 평균 (x, y)와 마지막 yaw. last_n 이면 최근 n 개만. 비어 있으면 None."""
    pts = centers[-last_n:] if last_n else centers
    if not pts:
        return None
    return (sum(c[0] for c in pts) / len(pts), sum(c[1] for c in pts) / len(pts), pts[-1][2])
