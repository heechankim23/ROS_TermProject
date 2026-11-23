"""request_gateway 가공부: 수거 요청 검증과 GPS -> map 변환 (ROS 없음)."""
import math

from .queue_logic import FALLEN, LOW_BATTERY, OUT_OF_ZONE

VALID_REASONS = (FALLEN, OUT_OF_ZONE, LOW_BATTERY)


def check_request(scooter_id, reason, lat, lon):
    """형식 검증. 문제가 없으면 '', 있으면 거절 사유."""
    if not scooter_id:
        return 'scooter_id 비어 있음'
    if reason not in VALID_REASONS:
        return f'알 수 없는 reason {reason}'
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        return '위경도 범위 오류'
    return ''


def to_map_checked(geo, lat, lon, max_range_m):
    """위경도 -> map (x, y). 서비스 지역 밖이면 (x, y, 거절 사유), 안이면 (x, y, '')."""
    x, y = geo.to_map(lat, lon)
    if math.hypot(x, y) > max_range_m:
        return x, y, f'서비스 지역 밖 ({x:.0f}, {y:.0f})'
    return x, y, ''
