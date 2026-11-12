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


def mean_center(centers, last_n=None):
    """중심 추정값들의 평균 (x, y)와 마지막 yaw. last_n 이면 최근 n 개만. 비어 있으면 None."""
    pts = centers[-last_n:] if last_n else centers
    if not pts:
        return None
    return (sum(c[0] for c in pts) / len(pts), sum(c[1] for c in pts) / len(pts), pts[-1][2])
