"""지형 높이 (캠퍼스 지형 world). maps/<맵>.json 의 elevation 격자(16 m)를 쌍선형 보간 -> world z (로봇 도크 = 0)."""
import json


class Terrain:
    def __init__(self, map_json):
        with open(map_json) as f:
            e = json.load(f)['elevation']
        self.x0, self.y0, self.step = e['xs0'], e['ys0'], e['step']
        self.nx, self.ny, self.datum, self.E = e['nx'], e['ny'], e['datum'], e['elev']

    def z(self, x, y):
        fx = min(max((x - self.x0) / self.step, 0.0), self.nx - 1.001)
        fy = min(max((y - self.y0) / self.step, 0.0), self.ny - 1.001)
        i, j = int(fx), int(fy)
        u, v = fx - i, fy - j
        E = self.E
        e = (E[j][i] * (1 - u) * (1 - v) + E[j][i + 1] * u * (1 - v) + E[j + 1][i] * (1 - u) * v
             + E[j + 1][i + 1] * u * v)
        return e - self.datum
