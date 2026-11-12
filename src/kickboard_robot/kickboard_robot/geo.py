"""F3: 위경도 <-> map 좌표(ENU) 변환. 노드 대신 공용 라이브러리로 둔다 (PRD 6장 허용).

WGS84 -> ECEF -> 기준점(origin) 기준 ENU. 수 km 범위에서 오차는 mm 수준.
"""
import math

_A = 6378137.0                 # WGS84 장반경
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


def _geodetic_to_ecef(lat, lon, h=0.0):
    la, lo = math.radians(lat), math.radians(lon)
    n = _A / math.sqrt(1.0 - _E2 * math.sin(la) ** 2)
    return ((n + h) * math.cos(la) * math.cos(lo),
            (n + h) * math.cos(la) * math.sin(lo),
            (n * (1.0 - _E2) + h) * math.sin(la))


def _ecef_to_geodetic(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - _E2))
    for _ in range(6):
        n = _A / math.sqrt(1.0 - _E2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - _E2 * n / (n + h)))
    n = _A / math.sqrt(1.0 - _E2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), h


class GeoConverter:
    """origin_lat/lon 이 map 원점, map +x = 동쪽(E), +y = 북쪽(N) 으로 가정."""

    def __init__(self, origin_lat, origin_lon, origin_alt=0.0):
        self.lat0, self.lon0, self.h0 = origin_lat, origin_lon, origin_alt
        self._ox, self._oy, self._oz = _geodetic_to_ecef(origin_lat, origin_lon, origin_alt)
        la, lo = math.radians(origin_lat), math.radians(origin_lon)
        self._sl, self._cl = math.sin(la), math.cos(la)
        self._so, self._co = math.sin(lo), math.cos(lo)

    def to_map(self, lat, lon):
        x, y, z = _geodetic_to_ecef(lat, lon, self.h0)
        dx, dy, dz = x - self._ox, y - self._oy, z - self._oz
        e = -self._so * dx + self._co * dy
        n = -self._sl * self._co * dx - self._sl * self._so * dy + self._cl * dz
        return e, n

    def to_latlon(self, x, y):
        e, n, u = x, y, 0.0
        dx = -self._so * e - self._sl * self._co * n + self._cl * self._co * u
        dy = self._co * e - self._sl * self._so * n + self._cl * self._so * u
        dz = self._cl * n + self._sl * u
        lat, lon, _ = _ecef_to_geodetic(self._ox + dx, self._oy + dy, self._oz + dz)
        return lat, lon
