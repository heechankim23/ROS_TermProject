#!/usr/bin/env python3
"""world SDF 의 고정 구조물(벽·선반·기둥·충전소)로 Nav2 용 점유 격자 지도를 바로 만든다.
SLAM 없이도 nav:=nav2 를 쓸 수 있게 하기 위함. world 를 바꾸면 OBSTACLES 를 맞춰 다시 실행.

  python3 scripts/generate_map.py maps/kickboard_world
"""
import math
import sys

RES = 0.05
X0, Y0, X1, Y1 = -5.5, -4.5, 5.5, 4.5        # 지도 범위 (map 좌표)
# (종류, cx, cy, sx|r, sy)
OBSTACLES = [
    ('box', 0.0, 4.05, 10.2, 0.1), ('box', 0.0, -4.05, 10.2, 0.1),
    ('box', 5.05, 0.0, 0.1, 8.2), ('box', -5.05, 0.0, 0.1, 8.2),
    ('box', 0.0, 3.6, 2.0, 0.6),               # shelf
    ('cyl', 4.0, 3.0, 0.3, 0),                  # pillar
    ('box', -4.7, 0.0, 0.3, 0.6),              # charging_station
]


def occupied(x, y):
    if not (-5.0 <= x <= 5.0 and -4.0 <= y <= 4.0):
        return True                             # 방 밖은 미지 대신 점유로
    for kind, cx, cy, a, b in OBSTACLES:
        if kind == 'box' and abs(x - cx) <= a / 2 and abs(y - cy) <= b / 2:
            return True
        if kind == 'cyl' and math.hypot(x - cx, y - cy) <= a:
            return True
    return False


def main(prefix):
    w, h = round((X1 - X0) / RES), round((Y1 - Y0) / RES)
    rows = []
    for j in range(h):                          # PGM 은 위쪽 행부터
        y = Y1 - (j + 0.5) * RES
        rows.append(bytes(0 if occupied(X0 + (i + 0.5) * RES, y) else 254 for i in range(w)))
    with open(prefix + '.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        for r in rows:
            f.write(r)
    name = prefix.rsplit('/', 1)[-1]
    with open(prefix + '.yaml', 'w') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {RES}\norigin: [{X0}, {Y0}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'{prefix}.pgm ({w}x{h}) / .yaml 생성')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'maps/kickboard_world')
