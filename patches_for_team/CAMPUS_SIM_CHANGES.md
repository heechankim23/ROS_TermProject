# 경북대 캠퍼스 시뮬레이션 (역할 ②) — 담당자별 변경 내역

캠퍼스 맵(`knu_campus`, 약 1.2 × 1.2 km)에서 킥보드 여러 대가 다양한 상태로 운영되고 로봇이 수거·정렬·충전·고장 분류를 하도록
노드를 확장했다. **기존 맵(실내 `kickboard_world`, 도로 `street`·`street_cs`)의 동작은 바꾸지 않았다.**
캠퍼스 기능은 맵 설정에 거치대 슬롯(`slots`)이나 `mission_variant: campus`, `map_json` 이 있을 때만 켜진다.

## E (인터페이스) — 확인 부탁

| 파일 | 변경 |
|---|---|
| `kickboard_interfaces/msg/ScooterState.msg` | 필드 추가: `string fault` (자가진단 고장 코드, 빈 문자열 = 정상), `bool in_slot_area` (슬롯 밖이지만 거치대 6 m 안 = 정렬 불량). `in_parking_zone` 은 캠퍼스 맵에서 "슬롯에 바르게 서 있음" 뜻 |
| `kickboard_interfaces/srv/RequestCollection.srv` | 사유 상수 추가: `BROKEN=4`, `OBSTRUCTION=5`(통행 방해 신고), `REPORTED=6`(방치·기타 신고), `MISALIGNED=7`(정렬 불량) |

## B (요청·대기열)

| 파일 | 변경 |
|---|---|
| `queue_logic.py` | 사유 4~7 과 가중치 추가: 통행 방해 신고 4.0 > 넘어짐 3.0 > 고장 2.5 > 구역 밖·신고 2.0 > 정렬 불량 1.5 > 배터리 1.0. `ALL_REASONS` |
| `request_gateway.py` | 허용 사유를 `ALL_REASONS` 로 (1~7) |
| `queue_manager.py` | 자동 취소 조건에 `not msg.fault` 추가 (고장 킥보드는 슬롯에 있어도 취소하지 않음) |

## C (주행·도킹)

| 파일 | 변경 |
|---|---|
| `campus_nav.py` (새 파일) | 큰 지도용 경로 계획. 빈 공간 위 1 m 격자 그래프(도로 1.0, 보행로 1.2, 그 외 1.8 비용)에서 scipy Dijkstra 로 큰 경로, 로봇 주변 70 m 창에서만 기존 `planner.Costmap` A*. 창은 로봇을 따라 옮기고 라이다 기억은 겹치는 부분만 옮김 |
| `nav_bridge.py` | 파라미터 `route_map_json`·`window_m` (주면 `campus_nav.RollingCostmap` 사용, 안 주면 기존과 같음). 창 안 중간 목표 8 m 전에 재계획. 경로점을 건너뛸 때 로봇에서 다음 점까지 직선이 막히면 건너뛰지 않음(모서리 끼임 방지). 앞이 막혔을 때 뒤가 비면 최대 `backoff_m`(0.6 m) 후진 후 재계획. 막혔을 때 원인 로그 |
| `occupancy.py` | numpy 로 한 번에 읽기 (캠퍼스 지도 2,580만 칸: 수십 초 → 1.8 초), PNG 지도 지원 |
| `sim_lidar.py` | 매 주기 지도 전체를 복사하지 않고 로봇 주변(라이다 거리 + 1 m)만 잘라 빔 계산 |

## D (미션)

| 파일 | 변경 |
|---|---|
| `mission_manager_campus.py` (새 파일) | `mission_manager.py` 서브클래스 (기존 파일 수정 없음). 실을 때 종류 결정: 고장 → 고장 분류 칸(반대 방향), 배터리 < 15 % → 충전 스탠드, 그 외 → 일반 슬롯. 하역할 때 거치대 센서(`/zone_occupancy`)로 빈 칸 배정, 가까운 구역부터 순회. 충전 칸이 없으면 일반 슬롯 + "충전 대기" 알림. 로봇팔이 닿도록 거치대를 마주 보고(슬롯에서 1.6 m) 좌우 0.75 m 안의 2~3칸씩 내림. `/sim/campus_events`: 충전 스탠드 고장·구역 폐쇄·충전 스탠드 수 제한·로봇팔 고장 처리 |
| `setup.py` | `mission_manager_campus` 실행 파일 등록 |

## A (시뮬레이션 환경) — 담당 파일

| 파일 | 변경 |
|---|---|
| `scooter_fleet_sim.py` | 캠퍼스 맵: 슬롯 기준 '구역 안' 판정, 정렬 불량, 고장 코드(숨은 고장은 로봇 4 m 안에서 드러남), 스스로 요청하지 않는 킥보드(신고로만), 충전 스탠드 충전(0.2 %/s), 이벤트 `report`·`fault`·`park`·`gps_drift`·`after_drop`, id 없는 이벤트(`stand_fault`·`zone_closed`·`robot_fault`)를 `/sim/campus_events` 로 전달, 거치대 센서 `/zone_occupancy`, 지형 높이 |
| `gz_utils.py` | 상태 색 추가: 고장(검은 발판·빨간 기둥), 정렬 불량(노란 발판), 충전 중(청록 기둥). 킥보드 생성을 gz 서비스로 바로 (대당 `ros2 run` 프로세스 없이) |
| `terrain.py` (새 파일) | 지형 world 의 높이 (maps/knu_campus.json elevation) |
| `arm_loader.py`, `status_panel.py` | 새 상태(고장·정렬 불량) 색·표시 |
| `launch/map_utils.py`, `launch/system.launch.py` | 맵 설정의 `map_json`·`terrain`·`mission_variant: campus` 를 노드에 전달. `scenario:=k01` 처럼 `config/scenarios_knu/` 도 찾음 |
| `scripts/build_knu_campus.py` | 거치대 고정부(뒤 바·키오스크)를 주행·라이다 지도에 장애물로 넣음, 맵 설정에 캠퍼스 키와 노드 파라미터 |
| `scripts/build_knu_scenarios.py` | k00 캠퍼스 운영 전경(킥보드 56대) 추가 |

## 알려진 점

- `kickboard_robot/test/test_extended.py::test_legacy_manifest_hashes` 는 이번 작업 전부터 실패한다 (`build_env.py`, `planner.py`, `street_layout.json` 등이 manifest 를 만든 뒤 이미 바뀜). 이번 작업으로 고정 파일이 더 바뀌었다. manifest 를 갱신할지는 그 프로필 담당이 정한다.
- 캠퍼스 맵 nav_bridge 는 시작할 때 큰 경로 격자를 만드느라 약 3~4 초, 메모리 최대 약 600 MB 를 쓴다.
