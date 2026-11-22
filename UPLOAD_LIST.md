# 깃허브에 올릴 파일 목록 (2026-11-23 기준)

링크는 이 파일(`my_ws/UPLOAD_LIST.md`) 위치 기준이다. 폴더는 통째로 올린다.

## ① 맵 제작

- [src/kickboard_sim/scripts/build_knu_campus.py](src/kickboard_sim/scripts/build_knu_campus.py)
- [src/kickboard_sim/scripts/knu_campus_viewer_template.html](src/kickboard_sim/scripts/knu_campus_viewer_template.html)
- 폴더 `src/kickboard_sim/maps/knu_campus_src/` (원본: OSM · 주차구역 이미지 · 고도)
- [src/kickboard_sim/maps/knu_campus.png](src/kickboard_sim/maps/knu_campus.png), [knu_campus.yaml](src/kickboard_sim/maps/knu_campus.yaml),
  [knu_campus_lidar.png](src/kickboard_sim/maps/knu_campus_lidar.png), [knu_campus_lidar.yaml](src/kickboard_sim/maps/knu_campus_lidar.yaml),
  [knu_campus_flat.yaml](src/kickboard_sim/maps/knu_campus_flat.yaml), [knu_campus_flat_lidar.yaml](src/kickboard_sim/maps/knu_campus_flat_lidar.yaml)
- [src/kickboard_sim/maps/knu_campus.json](src/kickboard_sim/maps/knu_campus.json), [knu_campus_2d.png](src/kickboard_sim/maps/knu_campus_2d.png), [knu_campus_viewer.html](src/kickboard_sim/maps/knu_campus_viewer.html)
- [src/kickboard_sim/worlds/knu_campus.sdf](src/kickboard_sim/worlds/knu_campus.sdf), [knu_campus_flat.sdf](src/kickboard_sim/worlds/knu_campus_flat.sdf)
- [src/kickboard_sim/config/maps/knu_campus.yaml](src/kickboard_sim/config/maps/knu_campus.yaml), [knu_campus_flat.yaml](src/kickboard_sim/config/maps/knu_campus_flat.yaml)
- `src/kickboard_sim/models/knu_campus/meshes/` 중 이름 뒤에 `_k00`~`_k32` 가 **없는** 것만:
  `terrain.obj/.mtl`, `terrain_flat.obj/.mtl`, `buildings.obj/.mtl`, `buildings_flat.obj/.mtl`, `racks.obj/.mtl`, `racks_flat.obj/.mtl`, `campus_ground.png`

## ② 시뮬레이션 환경

새 파일
- [campus_nav.py](src/kickboard_robot/kickboard_robot/campus_nav.py), [mission_manager_campus.py](src/kickboard_robot/kickboard_robot/mission_manager_campus.py), [terrain.py](src/kickboard_robot/kickboard_robot/terrain.py)

고친 파일 (`src/kickboard_robot/kickboard_robot/`)
- [scooter_fleet_sim.py](src/kickboard_robot/kickboard_robot/scooter_fleet_sim.py), [gz_utils.py](src/kickboard_robot/kickboard_robot/gz_utils.py), [nav_bridge.py](src/kickboard_robot/kickboard_robot/nav_bridge.py),
  [occupancy.py](src/kickboard_robot/kickboard_robot/occupancy.py), [sim_lidar.py](src/kickboard_robot/kickboard_robot/sim_lidar.py), [collector.py](src/kickboard_robot/kickboard_robot/collector.py),
  [queue_logic.py](src/kickboard_robot/kickboard_robot/queue_logic.py), [queue_manager.py](src/kickboard_robot/kickboard_robot/queue_manager.py), [request_gateway.py](src/kickboard_robot/kickboard_robot/request_gateway.py),
  [arm_loader.py](src/kickboard_robot/kickboard_robot/arm_loader.py), [status_panel.py](src/kickboard_robot/kickboard_robot/status_panel.py)
- [src/kickboard_robot/setup.py](src/kickboard_robot/setup.py)

인터페이스
- [ScooterState.msg](src/kickboard_interfaces/msg/ScooterState.msg), [RequestCollection.srv](src/kickboard_interfaces/srv/RequestCollection.srv)

실행 파일 (`src/kickboard_sim/launch/`)
- [bringup.launch.py](src/kickboard_sim/launch/bringup.launch.py), [sim.launch.py](src/kickboard_sim/launch/sim.launch.py), [system.launch.py](src/kickboard_sim/launch/system.launch.py), [map_utils.py](src/kickboard_sim/launch/map_utils.py)

로봇 모델
- [src/kickboard_sim/models/kickboard_robot/model.sdf](src/kickboard_sim/models/kickboard_robot/model.sdf)

## ③ 시나리오

- [src/kickboard_sim/scripts/build_knu_scenarios.py](src/kickboard_sim/scripts/build_knu_scenarios.py), [knu_scenarios_template.html](src/kickboard_sim/scripts/knu_scenarios_template.html)
- 폴더 `src/kickboard_sim/config/scenarios_knu/` 전체 (k00~k32 yaml, [README.md](src/kickboard_sim/config/scenarios_knu/README.md), catalog.json, catalog.html)

## ④ 시각화 · 실행 도구

- [ops_monitor.py](src/kickboard_robot/kickboard_robot/ops_monitor.py)
- [build_ops_replay.py](src/kickboard_sim/scripts/build_ops_replay.py), [ops_replay_template.html](src/kickboard_sim/scripts/ops_replay_template.html),
  [check_knu_runs.py](src/kickboard_sim/scripts/check_knu_runs.py), [run_knu_scenarios.py](src/kickboard_sim/scripts/run_knu_scenarios.py), [build_scenario_worlds.py](src/kickboard_sim/scripts/build_scenario_worlds.py)
- [src/kickboard_sim/rviz/kickboard.rviz](src/kickboard_sim/rviz/kickboard.rviz)
- 워크스페이스 맨 위: [run_all_knu.sh](run_all_knu.sh), [KNU_COMMANDS.md](KNU_COMMANDS.md), [GAZEBO_COMMANDS.md](GAZEBO_COMMANDS.md), [patches_for_team/CAMPUS_SIM_CHANGES.md](patches_for_team/CAMPUS_SIM_CHANGES.md)

## 올리지 않는 것

- 시나리오 전용 작은 world 생성물 (약 80 MB, `python3 src/kickboard_sim/scripts/build_scenario_worlds.py` 로 50초면 다시 만듦):
  `models/knu_campus/meshes/*_k00`~`*_k32`, `worlds/knu_k*.sdf`, `config/maps/knu_k*.yaml`, `maps/knu_k*.yaml`
- `results/`, `build/`, `install/`, `log/`, `~/.ros/kickboard_ops/`
- 진행 중: 전체 시나리오 검증 결과와 최종 리플레이 (`results/ops_replay.html`)
