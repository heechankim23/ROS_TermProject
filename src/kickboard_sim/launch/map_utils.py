"""맵 이름 -> 파일 경로. 맵 하나 = world SDF + 지도(pgm/yaml) + 맵 설정(yaml).

  kickboard_world (기본, 실내 10 x 8 m)  worlds/kickboard_world.sdf, maps/kickboard_world.yaml, config/map_config.yaml
  street / campus (scripts/build_env.py) worlds/<맵>.sdf, maps/<맵>.yaml, maps/<맵>_lidar.yaml, config/maps/<맵>.yaml

맵은 launch 인자 map:=<이름> 으로 고르거나, 시나리오 YAML 의 `map: <이름>` 으로 정한다 (인자가 우선).
맵 설정의 `params:` 는 노드 파라미터를 맵에 맞게 덮어쓴다 (주행 속도, 배터리 계수, d_max ...).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

DEFAULT_MAP = 'kickboard_world'


def scenario_path(scenario):
    share = get_package_share_directory('kickboard_sim')
    if os.path.isabs(scenario):
        return scenario
    name = scenario if scenario.endswith('.yaml') else scenario + '.yaml'
    for d in ('scenarios', 'scenarios_knu'):           # scenarios_knu = 경북대 캠퍼스 시나리오 (k01 ~)
        path = os.path.join(share, 'config', d, name)
        if os.path.exists(path):
            return path
    return os.path.join(share, 'config', 'scenarios', name)


def resolve_map(map_arg, scenario=None):
    """launch 인자 map 이 비어 있으면 시나리오 YAML 의 map 키, 그것도 없으면 기본 맵"""
    if map_arg == 'local' and scenario:           # 시나리오 전용 작은 캠퍼스: k08 -> knu_k08 (build_scenario_worlds.py)
        return 'knu_' + os.path.splitext(os.path.basename(scenario_path(scenario)))[0]
    if map_arg:
        return map_arg
    if scenario:
        with open(scenario_path(scenario)) as f:
            return (yaml.safe_load(f) or {}).get('map') or DEFAULT_MAP
    return DEFAULT_MAP


def map_files(name):
    share = get_package_share_directory('kickboard_sim')
    cfg = os.path.join(share, 'config', 'maps', name + '.yaml')
    if not os.path.exists(cfg):
        cfg = os.path.join(share, 'config', 'map_config.yaml')          # 기본 맵
    nav = os.path.join(share, 'maps', name + '.yaml')
    lidar = os.path.join(share, 'maps', name + '_lidar.yaml')           # 물리 장애물만 (진입 금지 구역 제외)
    out = {'name': name, 'world': os.path.join(share, 'worlds', name + '.sdf'), 'config': cfg,
           'nav_map': nav, 'lidar_map': lidar if os.path.exists(lidar) else nav}
    for k in ('world', 'config', 'nav_map'):
        if not os.path.exists(out[k]):
            raise FileNotFoundError(f'맵 "{name}" 의 {k} 파일이 없습니다: {out[k]}')
    with open(cfg) as f:
        cfg_all = yaml.safe_load(f) or {}
    out['params'] = cfg_all.get('params', {}) or {}
    out['mission_variant'] = cfg_all.get('mission_variant', '') or ''     # 'station' 킥보드 충전소 별도, 'campus' 거치대 슬롯
    # 큰 지도(캠퍼스): maps/<map_json> = 큰 경로·지형·구역 데이터. 지형 world 이면 terrain_json 도 넘긴다
    mj = cfg_all.get('map_json')
    out['map_json'] = os.path.join(share, 'maps', mj) if mj else ''
    terr = cfg_all.get('terrain') or {}
    out['terrain_json'] = out['map_json'] if (mj and terr and not terr.get('flat')) else ''
    return out
