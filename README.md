# 공유 킥보드 자율 수거·충전 로봇 (ROS 2 Humble) — 초안

PRD「공유 킥보드 자율 수거·충전 로봇 (ROS 2)」를 코드로 옮긴 **초안**이다.
요청 접수 → 대기열 → 접근 → 카메라 확인 → 적재 → 하역(주차구역 또는 충전소)·충전까지 시뮬레이션에서 사람 개입 없이 돈다.

> ⚠️ **PRD와 다른 점 (팀 공유 필요)**
> PRD는 *Gazebo Classic + TurtleBot3 Waffle Pi* 를 가정했지만, 개발 VM(arm64, Ubuntu 22.04)에는
> Gazebo Classic·`turtlebot3_gazebo`·`gazebo_ros_pkgs` 의 arm64 바이너리가 **없다**.
> 그래서 이미 설치된 **Gazebo Harmonic(gz sim 8) + ros_gz(소스 빌드)** 기반으로 만들고,
> 로봇은 SDF 모델로 직접 정의했다. 처음엔 Waffle Pi 치수였지만, 킥보드 4대를 싣고 로봇팔로 적재하는 모습을 보이려고
> **1.7 x 0.9 m 수거 로봇(운전석 캡 + 캡 위 로봇팔 + 뒤 적재함 4칸)** 으로 키웠다 (차동 구동 + 360° 라이다 + 카메라).
> 노드·인터페이스는 시뮬레이터와 무관하므로 x86 PC에서 Classic으로 옮겨도 그대로 쓸 수 있다.

> 📌 **브랜치 통합 현황, 아직 올라오지 않은 파일 목록, git 작업 규칙은 [12장](#12-통합-현황과-협업-규칙-2026-11-17-병합-관리)** 에 있다.

---

## 1. 폴더 구조

```
ROS_TermProject/
├── README.md
└── src/
    ├── kickboard_interfaces/      # (E) msg / srv / action — 11주차에 동결
    │   ├── msg/   ScooterState, RobotStatus, QueueItem, QueueStatus, AdminAlert,
    │   │          TagDetection(Array), ScooterTruthArray(시뮬 전용)
    │   ├── srv/   RequestCollection, GetQueue, CancelRequest, SetConfig,
    │   │          EnqueueRequest·SelectNextTarget·UpdateRequestStatus (내부용)
    │   └── action/ CollectScooter, DockToCharger, GoToPose(nav_bridge 내부용), LoadScooter(arm_loader)
    ├── kickboard_robot/           # 직접 구현 노드 (rclpy)
    │   ├── kickboard_robot/
    │   │   ├── scooter_fleet_sim.py   F1  킥보드 펌웨어 흉내 (시나리오 재생, GPS 노이즈)
    │   │   ├── request_gateway.py     F2  요청 검증·좌표 변환·순번 응답
    │   │   ├── geo.py                 F3  위경도 ↔ map(ENU) 변환 (라이브러리)
    │   │   ├── queue_manager.py       F4 F12 F13 F15  대기열·점수식·취소·SetConfig·RViz 마커
    │   │   ├── queue_logic.py             점수식/기아 방지 상한 (ROS 무관, 단위 테스트)
    │   │   ├── mission_manager.py     F5 F11 F14  상태 머신, /robot_status, /admin_alerts
    │   │   ├── nav_bridge.py          F6  GoToPose ↔ Nav2(또는 간이 주행), standoff(접근 1.6 m·정렬 1.35 m), 2회 재시도
    │   │   ├── collector.py           F7 F8  CollectScooter: 이동→확인(회전 탐색)→정렬→적재
    │   │   ├── arm_loader.py              [시뮬] 로봇팔 적재·하역 애니메이션 (Load/UnloadScooter action, sim:=gz)
    │   │   ├── planner.py                     코스트맵 + A* (nav_bridge simple 백엔드, ROS 무관·단위 테스트)
    │   │   ├── drop_zones.py                  주차구역·충전소 하역 자리 계산 (ROS 무관)
    │   │   ├── sim_lidar.py               [시뮬] 2D 라이다 흉내 -> /scan (지도 + 킥보드 정답 위치)
    │   │   ├── sim_localizer.py           [시뮬] 정답 위치로 map->odom 보정 (simple + Gazebo)
    │   │   ├── status_panel.py            [시뮬] Gazebo 'Kickboard status' 패널 (/echo)
    │   │   ├── battery_manager.py     F9  가상 배터리 (거리×k1 + 시간×k2)
    │   │   ├── dock_controller.py     F10 DockToCharger: pre-dock → 태그 기반 P제어
    │   │   ├── sim_tag_detector.py        [시뮬] 카메라+AprilTag 검출 흉내 (FOV·거리·노이즈)
    │   │   ├── apriltag_adapter.py        apriltag_ros 출력 → /tag_detections
    │   │   ├── fake_base.py               [시뮬] Gazebo 없는 kinematic 베이스
    │   │   └── analyze.py                 PRD 7장 지표 계산 (rosbag)
    │   └── test/test_logic.py         GPS 변환·aging·제어기 단위 테스트
    └── kickboard_sim/             # (A) 시뮬레이션 환경
        ├── worlds/kickboard_world.sdf      방 10×8 m, 벽·선반·기둥·주차구역·충전소
        ├── models/kickboard_robot/         수거 로봇 (캡 + 로봇팔 + 적재함 4칸)
        ├── models/charging_station/        충전소 (태그 id 100)
        ├── config/map_config.yaml          ★ 맵 종속 값 (GPS 기준점, 충전소, 주차구역, 스폰)
        ├── config/robot_params.yaml        노드 파라미터 (가중치, 용량, 배터리 계수…)
        ├── config/scenarios/s1~s9.yaml     PRD 7장 검증 시나리오
        ├── config/nav2_params.yaml, gz_bridge.yaml, apriltag.yaml
        ├── maps/kickboard_world.{pgm,yaml} world 에서 생성한 Nav2 지도
        ├── scripts/generate_map.py         world → 지도 생성기
        ├── launch/bringup.launch.py        ★ 전체 실행 (아래 두 개를 포함)
        ├── launch/sim.launch.py            Gazebo/브리지/TF/(Nav2)/(RViz)
        ├── launch/system.launch.py         직접 구현 노드들
        └── rviz/kickboard.rviz
```

### 노드 연결 한눈에 보기

```
scooter_fleet_sim ──RequestCollection──▶ request_gateway ──EnqueueRequest──▶ queue_manager
       │ /scooter_states ───────────────────────────────────────────────────▶ (자동 취소)
       │ /sim/scooter_truth ─▶ sim_tag_detector ─/tag_detections─▶ collector, dock_controller
                                                                          ▲
mission_manager ──SelectNextTarget / UpdateRequestStatus──▶ queue_manager │
       ├──CollectScooter(action)──▶ collector ──GoToPose──▶ nav_bridge ──▶ Nav2 또는 /cmd_vel
       ├──DockToCharger(action)───▶ dock_controller ──GoToPose──┘
       └── /robot_status ◀── /battery_pct ── battery_manager ◀── /odom
```

---

## 2. 설치 & 빌드

이 VM에는 이미 ROS 2 Humble, Gazebo Harmonic, `~/ros_gz_ws`(ros_gz 소스 빌드)가 있고
`~/.bashrc` 가 둘 다 source 한다. 추가로 필요한 것은 없다 (기본 모드 기준).

> **이 문서의 모든 명령은 워크스페이스(clone 한 `ROS_TermProject` 폴더) 안에서 실행한다고 가정한다.**
> 폴더를 어디에 두었든 그 폴더로 이동한 뒤 명령을 실행하면 된다.

```bash
colcon build --symlink-install
```

```bash
source install/setup.bash
```

**코드를 고친 뒤에는 항상 다시 `colcon build`** 한다. 이 환경에서는 `--symlink-install` 이어도
Python 노드(`kickboard_robot`) 파일이 `install/` 로 복사되기 때문에, 재빌드하지 않으면 예전 코드가 실행된다.
(launch·YAML·SDF 는 링크되므로 바로 반영된다.)

**선택 설치** — 쓰고 싶은 모드에서만 필요:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

```bash
sudo apt install ros-humble-apriltag-ros
```

---

## 3. 실행

새 터미널마다 워크스페이스에서 `source install/setup.bash` 를 먼저 한다.

### 3-1. 가장 기본: Gazebo + 시나리오 1

```bash
ros2 launch kickboard_sim bringup.launch.py scenario:=s1_single
```

Gazebo 창이 뜨고 약 6초 뒤 노드들이 시작된다. 'ㄴ'자 모델이 킥보드, 파란 박스가 충전소,
바닥의 녹색 패널이 주차구역이다. 터미널 로그의 `[상태] A -> B` 줄로 상태 머신 진행을 볼 수 있다.

**킥보드 색 = 상태** (`scooter_fleet_sim` 이 상태가 바뀔 때마다 모델을 새 색으로 다시 만든다)

| 부분 | 색 | 뜻 |
|---|---|---|
| 발판 | 초록 | 정상 (주차구역 안, 서 있음, 배터리 정상) |
| 발판 | 빨강 | 넘어짐 (FALLEN) |
| 발판 | 주황 | 주차구역 밖 (OUT_OF_ZONE) |
| 발판 | 보라 | 배터리 부족 (LOW_BATTERY, 서 있고 구역 안) |
| 기둥·핸들 | 보라 | 배터리 < 20% (사유와 상관없이 → 충전소로 갈 킥보드) |
| 기둥·핸들 | 회색 | 배터리 정상 |

킥보드 모델 이름은 `scooter_<id>` (링크 `scooter_<id>_body`), 로봇팔이 들고 있거나 적재함에 실린 것은 `cargo_<id>` 다.
Gazebo 오른쪽 Entity Tree 에서 이름으로 찾거나 클릭해서 확인한다.

**킥보드 상태 패널** — Gazebo 오른쪽 아래 `Kickboard status (press Echo)` 패널에서 **Echo** 를 누르면
`status_panel` 노드가 보내는 상태 목록이 1초마다 갱신된다 (토픽 칸은 기본값 `/echo` 그대로).
```
data: "robot DROPPING | bay 3/4 | battery 87%"
data: "#1 FALLEN | 80% | COLLECTING -> PARK"      바닥: 상태 | 배터리 | 대기열 -> 하역 예정지
data: "#2 IN BAY | 70% -> PARK"                   적재함에 실림
data: "#3 AT CHARGER | 10%"                       충전소에 맡김
data: "#4 OK | 75% (parked)"                      주차구역에 세워 둠
```
패널은 `config/gz_gui.config` (gz-sim8 기본 GUI + TopicEcho) 로 붙는다. 터미널에서는 `gz topic -e -t /echo`.

**로봇팔 적재 (LOADING 단계, 약 25초)** — 로봇이 킥보드 중심에서 1.35 m 떨어진 빈 자리(벽·장애물과 1.1 m 이상)에서 킥보드를 바라보면 캡 위의 노란 팔이
① 적재함 쪽에 접어 둔 자세에서 앞으로 돌아 발판 위로 뻗고 ② 내려가 빨간 집게로 발판을 잡고
③ 들어 올리며 넘어진 킥보드를 똑바로 세우고 ④ 180° 돌아 ⑤ 적재함 빈 칸(뒤 칸부터)에 세워 꽂은 뒤
⑥ 다시 적재함 쪽으로 높게 접힌다 (집게 높이 1.4 m — 실린 킥보드 핸들 1.26 m 위). 실린 킥보드는 로봇을 따라 움직인다.

**하역 장소는 킥보드마다 다르다** (실을 때 `/scooter_states` 의 배터리로 결정, `mission_manager`)

| 실은 킥보드 | 하역 |
|---|---|
| 배터리 < 20% (사유가 넘어짐·구역 밖이어도) | **충전소** — 도킹 후 로봇팔이 꺼내 충전소 양옆 **충전 자리**(4곳)에 세운다 (충전하러 맡김). 자리가 다 차면 가장 오래 맡긴 킥보드를 '충전 끝'으로 치우고 그 자리를 쓴다 |
| 그 외 (넘어짐·구역 밖, 배터리 정상) | **주차구역** — `DROPPING` 상태: 가장 가까운 주차구역 변 앞에 서서 로봇팔이 앞 칸부터 꺼내 구역 안에 나란히 세운다. 세운 킥보드는 다시 정상 킥보드가 된다 (넘어지면 다시 요청) |

만재이거나 대기열이 비면 주차구역행이 있을 때 먼저 `DROPPING`, 그다음 충전소행이 있거나 할 일이 없으면 충전소로 간다.
**근처 주차구역 (2026-11-17, D 요청)**: 배터리가 정상이고 주차구역행을 싣고 있을 때, 가장 가까운 하역 자리가
`opportunistic_drop_dist`(기본 2.0 m) 안이면 다음 대상을 고르기 전에 들러서 내린다. 판단은 `SELECTING` 에서만 하므로
이동 중에 경로를 바꾸지는 않는다(비선점). `ros2 param set /mission_manager opportunistic_drop_dist 0.0` 이면 예전 동작.
로봇 배터리가 부족하면 주차구역행을 실은 채 충전부터 하고, 충전 후 내려놓는다. 주차구역에 빈 자리가 없으면 충전소에서 전부 내린다.
하역 자리는 `kickboard_robot/drop_zones.py` 가 `map_config.yaml` 의 주차구역과 지도에서 계산한다 (구역 A·B 에서 로봇이 설 공간이 있는 변 각 1개, 변마다 4자리, 총 8자리). 팔 동작을 천천히 보려면 `ros2 param set /arm_loader time_scale 2.0`.

로봇 치수 (`models/kickboard_robot/model.sdf`, 원점 = 구동축 중심 바닥)

| 부분 | 범위 |
|---|---|
| 차체 | x −0.85 ~ +0.75, 폭 0.74 (바퀴 포함 0.89), 바퀴 반지름 0.10, 간격 0.82 |
| 캡 | x +0.35 ~ +0.75, 높이 0.80. 앞면에 카메라(0.76, 0, 0.45), 범퍼 위 라이다(0.80, 0, 0.22) |
| 로봇팔 | 회전축 (0.50, 0) 캡 위, 어깨 높이 0.95, 윗팔 0.75 + 아랫팔 0.75, 손목 회전 + 2지 집게 |
| 적재함 | x −0.85 ~ +0.33, 바닥 높이 0.31, 칸 x = −0.58 / −0.36 / −0.14 / +0.08. 이웃 칸은 킥보드 방향을 반대로 놓아 핸들이 겹치지 않음 |

### 3-2. 다른 시나리오 / 옵션

```bash
ros2 launch kickboard_sim bringup.launch.py scenario:=s3_full_load rviz:=true
```

```bash
ros2 launch kickboard_sim bringup.launch.py sim:=kinematic scenario:=s4_low_battery rviz:=true
```

| 인자 | 값 | 설명 |
|---|---|---|
| `scenario` | `s1_single` … `s9_drop_sites` 또는 YAML 절대경로 | 아래 시나리오 표 참고 |
| `sim` | **`gz`** / `kinematic` | `kinematic`: Gazebo 없이 `/cmd_vel` 적분. 로직만 빠르게 검증할 때 (가장 가벼움) |
| `nav` | **`simple`** / `nav2` | `simple`: 직접 구현한 코스트맵 + A* 주행 (벽·킥보드 회피, `planner.py`). `nav2`: Nav2 설치 필요 |
| `perception` | **`sim`** / `apriltag` | `sim`: 실제 위치 + 카메라 FOV(62°, 3 m)로 검출 흉내 (카메라 렌더링 생략). `apriltag`: 실제 카메라 영상 + apriltag_ros |
| `gui` | **`true`** / `false` | Gazebo GUI 표시 여부 (`false` 면 서버만) |
| `sensors` | **`true`** / `false` | `false`: 라이다·카메라 렌더링 끔. 디스플레이 없는 SSH 세션용 |
| `rviz` | `true` / **`false`** | RViz (대기열 마커, TF, 스캔, 오도메트리) |
| `start_delay` | **`6.0`** | Gazebo 기동 후 노드 시작까지 대기(초). 느린 VM이면 늘린다 |
| `arm` | **`true`** / `false` | `sim:=gz` 에서 로봇팔 적재 애니메이션. `false` 면 예전처럼 3초 정지로 적재 (kinematic 은 항상 정지) |

**VM 이 느리거나 뜨거울 때** (가벼운 순):
1. `sim:=kinematic rviz:=true` — Gazebo 없이 로직만. 가장 가볍다.
2. `gui:=false rviz:=true` — Gazebo 는 화면 없이 서버로만, RViz 로 본다. Gazebo GUI 가 가장 무겁다.
3. 시뮬레이션은 한 번에 하나만. 다른 앱(브라우저, VS Code)은 닫는다.

> 성능 메모 (s1, Gazebo 서버 모드, 6코어 VM): 노드 전체 CPU 370% → 142%.
> ① 물리 스텝 4 → 10 ms (`/clock` 270 → 100 Hz), ② `perception:=sim` 일 때 카메라 렌더링 생략,
> ③ rclpy `MultiThreadedExecutor` 를 쓰지 않음 — 메시지마다 오버헤드가 커서 노드당 20~60% 를 썼다.
> 모든 노드는 단일 스레드 executor 로 돌고, 오래 걸리는 Action 작업만 `common.run_blocking()` 으로 작업 스레드에서 실행한다.
> 새 노드를 만들 때도 이 방식을 따른다.

> SSH 등 디스플레이가 없는 터미널에서 Gazebo를 띄우면 ogre 가 `Couldn't open X display` 로 죽는다.
> 데스크톱 터미널에서 실행하거나, `gui:=false sensors:=false` 를 쓰거나, `sim:=kinematic` 을 쓴다.

### 3-3. 시나리오 (PRD 7장)

| 파일 | 내용 | 통과 기준 |
|---|---|---|
| `s1_single` | 1대, 10초에 넘어져 같은 ID로 **병합** | 접근 → 확인 → 적재 → 주차구역에 세움 → 충전소 대기 |
| `s2_new_request` | 2건 처리 중 20초에 1건 추가 | 현재 대상 완료 후 재평가, 3건 모두 수거 |
| `s3_full_load` | 용량 4, 요청 5건 | 4대 후 하역·충전, 남은 1건 처리 |
| `s4_low_battery` | 초기 배터리 25% | 20%에서 대상 반납·복귀, 80% 충전 후 재개 |
| `s5_aging` | 먼 곳 1건 + 가까운 요청 25초마다 | 먼 1번보다 t_max 이상 늦게 들어온 요청이 1번을 앞지르지 않음 (아래 참고) |
| `s6_verify_fail` | 1번은 실물 없음 | 회전 탐색 후 실패, 다음 대상으로 |
| `s7_obstacle` | 경로에 장애물 투입 | 충돌 없이 도착 — **`nav:=nav2` 필요** |
| `s8_auto_cancel` | 대기 중인 킥보드가 다시 세워짐 (P1) | 대기열에서 자동 제거 |
| `s9_drop_sites` | 넘어짐·구역 밖(배터리 정상) 2대 + 배터리 부족 2대 | 주차구역에 2대 세움(재요청 없음) → 충전소 하역 2대 |

시나리오 YAML 형식 (새 시나리오는 파일만 추가하면 된다):

```yaml
name: "이름"
robot: {initial_battery_pct: 100, capacity: 4}
scooters:            # id = AprilTag id, model: false 면 요청만 오고 실물 없음, spawn_at: 등장 시각(s)
  - {id: 1, x: 1.5, y: -1.0, yaw_deg: 30, tilt_deg: 0, battery_pct: 80}
events:              # type: fall | stand | battery(pct) | move(x,y) | obstacle(name,x,y,size)
  - {t: 10, id: 1, type: fall}
```

요청 조건(펌웨어 판정, 2026-11-15 확정): 기울기 > 45° → `FALLEN`, 주차구역 밖 → `OUT_OF_ZONE`, 배터리 < 20% → `LOW_BATTERY`.
정확히 45°·20% 는 정상으로 본다. 자동 취소(F13)는 반대 조건(≤ 45°, 구역 안, ≥ 20%)을 모두 만족할 때 (s8 로 확인).

---

## 4. 실행 중 확인·조작

```bash
ros2 topic echo /robot_status
```

```bash
ros2 topic echo /queue_status
```

```bash
ros2 topic echo /mission_events
```

```bash
ros2 action send_goal /dock_to_charger kickboard_interfaces/action/DockToCharger "{station_id: station_1}" --feedback
```

P1 서비스:

```bash
ros2 service call /get_queue kickboard_interfaces/srv/GetQueue
```

```bash
ros2 service call /cancel_request kickboard_interfaces/srv/CancelRequest "{scooter_id: '2'}"
```

```bash
ros2 service call /set_config kickboard_interfaces/srv/SetConfig "{return_battery_pct: 30.0, capacity: 2, w_r: -1.0, w_t: 2.0, w_d: -1.0}"
```

(음수 = 변경 안 함.) 수동으로 수거 요청 넣기 — 위경도는 `map_config.yaml` 의 기준점 근처:

```bash
ros2 service call /request_collection kickboard_interfaces/srv/RequestCollection "{scooter_id: '9', lat: 37.56649, lon: 126.97802, reason: 1}"
```

(실물이 없는 ID라 확인 실패 경로를 탄다. 실물이 있는 킥보드는 시나리오 YAML 로 추가한다.)

---

## 5. 지표 측정 (PRD 7장)

```bash
ros2 bag record -o bags/s3 /mission_events /robot_status /queue_status
```

시나리오가 끝나면 Ctrl+C 후:

```bash
ros2 run kickboard_robot analyze bags/s3
```

출력: 수거 성공률, 최대/평균 대기 시간, 도킹 위치·방향 오차, GPS 오차 vs 카메라 보정 후 오차.
`queue_manager` 시작 로그의 `t_max` 가 점수식의 기아 방지 상한이다 (기본 가중치 10/1/5, d_max 12.8 m → 84 s). 경북대 맵은 100/0.5/1, d_max 1200 m → 3000 s (10장).

단위 테스트:

```bash
cd src/kickboard_robot && python3 -m pytest -q test/
```

---

## 6. Nav2 모드

1. `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
2. 지도는 world 에서 바로 생성해 두었다 (`maps/kickboard_world.*`). world 를 고치면 `scripts/generate_map.py` 의 `OBSTACLES` 를 맞추고 다시 생성한다.
3. 실행:

```bash
ros2 launch kickboard_sim bringup.launch.py nav:=nav2 scenario:=s7_obstacle rviz:=true start_delay:=15
```

AMCL 초기 위치는 `nav2_params.yaml` 의 `initial_pose` (= `map_config.yaml` 의 `robot_spawn`) 로 자동 설정된다.
**Nav2 모드는 이 VM에 Nav2가 없어 아직 실행해 보지 못했다** — 파라미터는 튜닝 전 초안이다.

## 7. 실제 카메라 + AprilTag 모드 (향후 확장)

> **팀 결정 (2026-11-12)**: 발표 범위의 인식은 `perception:=sim`(시뮬레이션 검출기)로 한다.
> 태그 검출 결과는 정답 위치로 만들고, 그 결과로 확인·위치 보정·도킹 로직을 검증한다.
> 지표 수치는 "시뮬레이션 검출 기준"으로 표기한다. 아래 모드는 지우지 않고 향후 확장용으로 남긴다.

현재 킥보드·충전소 모델의 태그 자리는 흰 패널이다. `perception:=apriltag` 를 쓰려면:
1. tag36h11 이미지(킥보드 1~99, 충전소 100)를 텍스처로 만들어 `gz_utils.scooter_sdf()` 와 `charging_station/model.sdf` 의 `tag` visual 에 입힌다.
2. `apriltag_adapter.py` 의 `normal_axis_sign` 을 실제 태그 프레임 방향에 맞춘다.
3. `ros2 launch kickboard_sim bringup.launch.py perception:=apriltag`

---

## 8. 맵 바꾸기

맵에 종속된 값은 `config/map_config.yaml` 한 곳에 있다: GPS 기준점, 로봇 스폰(=도킹) pose, 충전소 태그 pose,
주차구역 폴리곤. 새 world 를 쓰면 이 파일과 `scripts/generate_map.py`, 시나리오의 킥보드 좌표만 고친다.

## 9. 역할 분담과 다음 할 일 (2026-11-12 조정)

PRD 8장의 역할을 다음처럼 조정했다.
- 미션(D)에서 주행을 떼어 C로 만들었다. 도킹(`dock_controller`)도 정밀 주행이라 C가 맡는다.
- 인식은 실제 카메라 대신 시뮬레이션 검출기를 유지하므로 (7장), `sim_tag_detector` 는 다른 시뮬 센서처럼 A가 맡는다. 수거 절차(`collector`)는 미션 흐름의 일부라 D가 맡는다.
- D와 E는 "만드는 사람"과 "확인하는 사람"으로 나눈다. D는 판단 로직과 수거 절차를 만든다. E는 인터페이스를 관리하고, 전체가 맞물려 도는지 확인하며, 숫자로 증명한다.
- E는 팀장(기술 리드)을 겸한다. 전체를 보는 자리라 일정·범위 결정·역할 사이 조율을 함께 맡고, 매주 회귀 결과를 그대로 진행 점검 자료로 쓴다.

| 역할 | 맡는 파일 | 다음 할 일 |
|---|---|---|
| A. 시뮬 환경·시나리오·데모 영상 | `kickboard_sim/` (world·모델·launch·`gz_bridge.yaml`·`map_config.yaml`·시나리오), `scooter_fleet_sim.py`, `arm_loader.py`, `status_panel.py`, `sim_tag_detector.py`, `fake_base.py`, `apriltag_adapter.py`(보류, 7장) | 수업 맵 적용, 로봇팔 동작 다듬기, `sim_tag_detector` 현실화(벽·킥보드 가림, 넘어진 킥보드의 태그 방향), s7 장애물 정보를 C에 제공하는 부분 확인, 발표 데모 영상 촬영·편집(시나리오는 E가 선정) |
| B. 요청·대기열 | `request_gateway.py`, `geo.py`, `queue_logic.py`, `queue_manager.py`, README·git (PRD는 E) | 요청 임계값(45°, 15%) 확정, `w_r·w_t·w_d` 실험(s5), RViz 마커 다듬기, 문서·git·빌드 관리 |
| C. 주행·도킹 | `nav_bridge.py`, `planner.py`, `motion.py`, `sim_lidar.py`, `sim_localizer.py`, `dock_controller.py`, `config/nav2_params.yaml` | s7 장애물이 `sim_lidar` 에 잡히게 해서 회피 검증, Nav2를 쓰기로 정하면 설치·튜닝, 좁은 곳 회전 시 모서리 접촉 줄이기, 도킹 오차를 Gazebo 실제 위치로 측정 |
| D. 미션 | `mission_manager.py`, `collector.py`, `battery_manager.py`, `drop_zones.py` | 실패 경로(확인 실패, 하역 실패, 도킹 3회 실패, 주차 자리 부족) 검증, `/battery_low`·`/admin_alerts` 처리 결정, `capacity` 기본값(코드 3, 설정 4) 정리, F16·F17 |
| E. 기술 리드 (인터페이스·통합 검증·일정) | `kickboard_interfaces/`, `analyze.py`, `kickboard_robot/test/`, PRD | 인터페이스 리뷰 후 11주차 동결(이후 변경 창구), 매주 s1~s9 회귀 실행과 담당자 통보, `analyze` 로 지표 측정해 10장 검증표 갱신(s3·s9 재측정), 발표 데모 시나리오 선정·rosbag, 추가 개발 기술 조사(실제 카메라 인식, Nav2 등)와 채택·보류 결정, 주간 회의·발표(12월 초) 역산 일정 관리, PRD를 실제 구현에 맞게 유지 |

공통 규칙
- msg·srv·action 변경은 E를 거치고, 영향받는 담당과 같은 날 반영한다. `mission_manager` 가 쓰는 것을 바꿀 때는 D도 확인한다. E는 변경 요청에 하루 안에 답한다.
- `mission_manager.py` 는 D만 고친다. 다른 사람은 D에게 요청한다.
- `/cmd_vel` 은 nav_bridge·dock_controller(C)와 collector(D)가 함께 쓴다. 두 노드가 동시에 보내지 않는다.
- E는 회귀 테스트에서 찾은 문제를 담당자에게 알리고, 남의 코드를 직접 고치지 않는다.
- 두 역할에 걸친 일은 한 명이 맡고 다른 한 명이 확인한다 (예: s7 은 C가 작업, A가 확인). 누가 맡을지 애매하면 E가 정한다.
- 범위를 넓히는 일(Nav2, 실제 카메라, 새 기능)은 E의 조사 결과를 보고 주간 회의에서 채택·보류를 정한다.
- 코드를 공유하기 전에 각자 s1 을 돌려 통과를 확인한다.

## 10. 검증 결과 (2026-11-10, 초안 기준)

kinematic 모드에서 시나리오별로 돌리고 `analyze` 로 계산한 값이다. s1·s3 은 Gazebo(Harmonic)에서도 돌렸다.

| 시나리오 | 결과 | 비고 |
|---|---|---|
| s1 단일 수거 | ✅ | 병합(OUT_OF_ZONE → FALLEN) 확인. Gazebo 에서도 통과, 도킹 1.0 cm / 0.7° |
| s2 이동 중 새 요청 | ✅ 3/3 | 처리 중 추가된 요청은 현재 대상 완료 후 재평가 |
| s3 만재 | ✅ 5/5 | 3대 → 하역·충전 → 2대. Gazebo 에서 벽까지 최소 거리 0.74 m (충돌 없음) |
| s4 배터리 부족 | ✅ 2/2 | 주행 중 20% → 대상 반납·복귀, 80% 충전 후 재개 |
| s5 aging | ✅ 7/7 | 먼 1번(LOW_BATTERY)이 대기 23~56 s 에 선택 (상한 t_max 84 s) |
| s6 확인 실패 | ✅ | 1번 회전 탐색 10 s 후 실패·관리자 알림, 2번 정상 수거 |
| s7 장애물 | ⏸ | Nav2 미설치로 미검증 |
| s8 자동 취소 | ✅ | 2번이 다시 세워지자 대기열에서 제거 |

- 도킹 오차: 최대 2.4 cm / 3.9° (목표 ≤ 5 cm, ≤ 5°)
- 위치 보정: GPS 평균 1.1~1.9 m → 카메라 보정 후 평균 0.04~0.09 m, 최대 0.12 m (목표 ≤ 0.2 m)
- 주의: VM 에 부하가 크면 rosbag 기록이 늦게 시작돼 앞부분 이벤트가 빠질 수 있다. 측정할 때는 시뮬레이션을 하나만 돌린다.

### 대기열 가중치 실험 (2026-11-15, B)

결론: **기본값 `w_r` 10 / `w_t` 1 / `w_d` 5 를 유지한다.** 세 조합 중 평균·최대 대기가 가장 짧았다.

t_max 가 보장하는 것은 "1번보다 t_max 이상 늦게 들어온 요청은 1번을 앞지르지 못한다"이다.
1번보다 먼저, 또는 t_max 안에 들어온 요청에는 밀릴 수 있고, 수거 도중에는 대상을 바꾸지 않는다(비선점).
그래서 "1번이 t_max 안에 선택"은 보장이 아니다. 기본 가중치에서도 1번은 116 s 만에 선택됐다 (t_max 84 s).

| w_r / w_t / w_d | t_max | 1번 선택까지 | 1번을 앞지른 요청 (접수 시각) | 보장 | 평균 대기 | 최대 대기 | 수거 |
|---|---|---|---|---|---|---|---|
| **10 / 1 / 5** | 84.0 s | 116 s | 2·3 (0 s), 5 (20 s), 4 (10 s), 6 (30 s) | O | **82.7 s** | **175 s** | 6/7 |
| 10 / 1 / 2 | 45.6 s | 124 s | 2·3 (0 s), 4 (10 s), 5 (20 s) | O | 123.2 s | 202 s | 6/7 |
| 10 / 2 / 5 | 42.0 s | 100 s | 2·3 (0 s), 4 (10 s), 5 (20 s) | O | 108.0 s | 179 s | 6/7 |

- 조건: `sim:=kinematic`, `nav:=simple`, 조합마다 1회. 가중치는 시작 직후 `/set_config` 로 바꿨다.
- 평균·최대 대기 = 접수부터 적재 완료까지 (`queue_manager` 로그의 `N: DONE (대기 Ns)`).
- 시나리오: s5 를 바꾼 실험용 배치. 1번 = (2.5, −3.0) 구역 B 안 배터리 8% (LOW_BATTERY), 넘어진 킥보드 2~7번 = s5 와 같은 자리에 0·0·10·20·30·40 s 에 등장.
  원래 s5 는 kinematic 에서 2번 수거가 16 s 에 끝나 대기열에 1번만 남고(가중치와 무관하게 바로 선택), 1번 (3.5, −3.0) 으로 가는 경로가 없어(`no path`) 실험에 쓸 수 없었다.
- 세 번 모두 4번 (−3.0, −2.6) 은 경로가 없어 `NEEDS_HUMAN` → 수거 6/7. 주행(C) 확인 필요.
- `w_d` 를 낮추면 먼 곳을 오가는 시간이 늘어 모든 지표가 나빠졌다. `w_t`=2 는 1번을 16 s 빨리 뽑지만 평균 대기가 25 s 늘었다.

### 경북대 맵 대기열 가중치 (2026-11-23, B)

결론: **경북대 맵(`knu_campus`, `knu_campus_flat`, `knu_k00`~`knu_k32`)은 `w_r` 100 / `w_t` 0.5 / `w_d` 1, `d_max` 1200 m 를 쓴다.**
교실 맵(s1~s9)은 기본값 10 / 1 / 5 그대로다. 맵 설정의 `params.queue_manager` 가 덮어쓴다.

- 기준: 거리와 넘어짐을 중점으로 본다. `w_d` = 1 이라 점수 1점 = 1 m.
- `w_r` 100: 사유 가중치 1 차이 = 100 m. 넘어짐(3.0)은 배터리 부족(1.0)보다 200 m, 불법 주차(2.0)보다 100 m 멀어도 먼저 뽑힌다.
  예전 10 / 1 / 1 에서는 20 m 만 멀어도 밀려서 사유가 거의 반영되지 않았다. 사유 순서(통행 방해 4.0 > 넘어짐 3.0 > 고장 2.5 ...)는 그대로다.
- `w_t` 0.5: 대기 1분 = 30 m (예전 60 m). 대기 시간보다 거리·사유를 먼저 본다.
- 기아 방지 상한 t_max = (100 × 3 + 1 × 1200) / 0.5 = 3000 s (예전 1230 s). 거리·사유 비중을 키운 대가다.
- 시나리오 거리 (도크 → 수거 대상, 직선): 중앙값 337 m, 최대 772 m. 한 시나리오 안 대상끼리 거리 차이 중앙값 76 m.

오프라인 비교 (직선 이동 1.5 m/s, 대당 30 s, 4대마다 도크 복귀로 단순화한 계산. 실제 주행 결과가 아니다):

| 시나리오 | 가중치 | 총 이동 | 평균 대기 | 최대 대기 | 넘어짐 평균 대기 |
|---|---|---|---|---|---|
| k16 (25대) | 10 / 1 / 1 | 7907 m | 38.3 분 | 90.2 분 | 40.6 분 |
| k16 | **100 / 0.5 / 1** | 7927 m | **36.6 분** | **88.7 분** | **27.1 분** |
| k16 | 100 / 1 / 1 | 9009 m | 41.5 분 | 100.7 분 | 32.7 분 |
| k00 (14대) | 10 / 1 / 1 | 4097 m | 20.4 분 | 47.9 분 | 39.8 분 |
| k00 | **100 / 0.5 / 1** | 4577 m | 24.8 분 | 51.4 분 | **7.6 분** |

- k32·k14 는 세 조합의 결과가 같았다.
- k00 은 넘어진 2대를 먼저 가느라 총 이동이 12 % 늘고 평균 대기가 4.4 분 늘었다.
- 시뮬레이션(`run_all_knu.sh`)으로 다시 확인할 것.

같은 킥보드 요청 병합 규칙 (`queue_logic.merge_reason`):
- 킥보드 판정끼리는 최신 상태를 따른다 (구역 밖 → 넘어짐, 다시 세워지면 배터리 부족으로 내려감). 예전과 같다.
- 사용자 신고(`OBSTRUCTION`, `REPORTED`)가 끼면 가중치가 높은 사유를 남긴다. 예전에는 넘어진 킥보드에 신고(2.0)가 들어오면 넘어짐(3.0)이 신고로 덮여 순위가 내려갔다.
- 신고 위치는 GPS 오차가 커서, 이미 들어온 요청의 위치를 덮지 않는다.

## 11. 알려진 한계 (초안)

- `nav:=simple` 주행 (`nav_bridge` + `planner.py`): 0.1 m 격자 코스트맵 = 정적 지도(벽·선반·기둥·충전소)
  + 라이다로 본 장애물(킥보드 등; 빔이 지나간 칸은 비우고 맞은 칸은 60초 기억). 장애물에서 로봇 반폭 0.47 m 안은
  통과 불가, 1.1 m(로봇을 덮는 원 0.98 m + 격자 오차)까지는 비용을 높여 되도록 피한다. A* 경로를 1초마다(새 장애물이 경로를 막으면 즉시)
  다시 계획한다. 접근·정렬 목표는 목표 둘레에서 장애물과 1.1 m 이상 떨어진 빈 자리로 고른다 (거기서 제자리 회전해도 안 닿게).
  라이다: 이 VM(ogre1)에서 Gazebo gpu_lidar 는 모든 빔에 최소값 0.12 m 만 내서 쓸 수 없다 (예전부터 그랬음 — 그래서
  옛 '전방 정지' 안전장치도 실제로는 동작하지 않았다). `sim_lidar` 가 정적 지도 + 킥보드 정답 위치·기울기로 360 빔을
  쏴서 /scan 을 만든다 (sim_tag_detector 가 카메라를 흉내 내는 것과 같은 방식). x86 + ogre2 PC 에서는
  gz_bridge.yaml 에 /scan 을 다시 넣고 sim_lidar 를 빼면 실제 센서로 돌아간다.
  전방 0.15 m 안에 장애물이 있으면 안전장치로 멈춘다.
- 물리 스텝은 10 ms 다 (`/clock` 100 Hz). 더 정밀한 접촉이 필요하면 world 의 `max_step_size` 를 줄이되 CPU 가 늘어난다.
- `nav:=simple` + Gazebo 에서 위치 추정은 `sim_localizer` 가 Gazebo 정답 위치로 map→odom 을 계속 보정한다 (완벽한 AMCL 가정).
  1.7 m 로봇은 부딪히거나 미끄러지면 오도메트리가 수 m 어긋났기 때문 (실측 2.7 m). kinematic 모드는 정적 TF.
- `sim_tag_detector` 는 가림(occlusion)을 고려하지 않는다. 넘어진 킥보드도 서 있을 때와 같은 면 배치로 본다.
- 카메라 보정 위치는 태그 면에서 고정 거리(0.2 m)만큼 안쪽을 중심으로 근사한다.
- 로봇팔 적재는 **시각화용 운동학 애니메이션**이다. 팔 관절은 Gazebo 관절 제어기로 실제로 움직이지만,
  킥보드는 집는 순간 충돌체 없는 복사본(`cargo_<id>`)으로 바뀌고 `arm_loader` 가 매 주기(25 Hz) 집게·적재함 위치로 옮긴다.
  집기 성공 여부나 무게는 시뮬레이션하지 않는다. 넘어진 킥보드는 들어 올리는 동안 회전을 보간해 세운다.
- 로봇이 1.7 x 0.9 m 라 제자리 회전 반경이 ~1 m 다. 경로는 장애물과 1.1 m 를 되도록 지키지만, 좁은 곳에서는
  0.47 m 까지 붙을 수 있어 회전 중 모서리가 닿을 수 있다. s5 의 킥보드 위치는 도킹 자리에서 회전할 수 있도록 옮겼다.


## 12. 통합 현황과 협업 규칙 (2026-11-17 · 11-23 갱신, 병합 관리)

12-0 은 11-23 병합, 12-1 ~ 12-4 는 11-17 병합 당시 기록이다 (12-3 의 빠진 파일은 대부분 11-23 병합으로 들어옴).

### 12-0. 2026-11-23 병합 (role_A · B · C · D 전부)

| 순서 | 브랜치 | 내용 | 병합 시 처리 |
|---|---|---|---|
| 1 | role_B (role_A 포함) | A: 경북대 캠퍼스 맵·시나리오 `scenarios_knu/` k00~k32, 도로 맵 s1~s19, 확장 프로필, `archive/`. B: 킥보드 배터리 기준 15→20%, 경북대 맵 가중치 100/0.5/1, 요청 병합 규칙 | fast-forward (충돌 없음) |
| 2 | role_C | 접근 지점 없음도 재시도, 실패 사유 구분(pose/scan unavailable, no safe approach point), 경로 없음 진단, sim_lidar 잡음 범위 제한 | `nav_bridge.py` 충돌: A 의 '가장 가까운 빈 자리' 대체 목표 + C 의 (목표, 사유) 반환 형식을 **둘 다 유지**. 경로 없음 로그는 A 의 상세판 |
| 3 | role_D | 가공부 분리(`*_logic.py`), 에너지 기반 배터리 판단, 하역 위치 막힘 처리, 근처 하역 D 구조 이식, 정렬 다른 각도 재시도, CI·PR 템플릿·CONTRIBUTING | `collector.py`: D 버전(A 패치 이미 포함). `motion.py`: D 의 히스테리시스(A 와 같은 진동 문제 수정이라 하나로). `mission_manager.py`: D 구조 + 배터리 20% |
| 4 | (공통) | 줄바꿈 renormalize (확장 프로필 CRLF 26개 → LF). `mission_logic` 기본값 20%, `lidar_logic.add_noise` 에 C 의 range_max 제한 반영 | 병합 커밋에 포함 |

검증 (ROS 없는 환경): 전체 `.py` 문법·pyflakes 통과, ROS 무관 pytest **54/56 통과**
(실패 2 = `test_sim_localizer_map_odom` rclpy 없음, `test_legacy_manifest_hashes` 확장 프로필 해시 — 둘 다 병합 전부터 실패. 해시는 아래에서 갱신),
`build_env.py` 재생성 결과가 저장소 파일과 동일. **Ubuntu 에서 `colcon build` · kinematic/Gazebo 회귀는 아직 안 함.**

추가 병합·수정 (11-23 저녁, role_E 에 반영):
- role_D `43cb2b1`(정렬 실패 시 기다렸다 같은 자리로) · `role_D_align_wait_knu`(같은 코드의 KNU 적용판) 병합
- **D 의 `MissionManager` 구조 변경에 상속 클래스 3개를 맞춤** — 맞추지 않으면 경북대 시나리오가 시작하자마자 죽었음
  - `mission_manager_campus.py`(경북대) · `mission_manager_station.py`(충전소 별도 도로맵): 상태 이름(`IDLE` 등)을
    `mission_logic` 에서 가져오도록, 적재함을 `Cargo` 객체 방식(`cargo.load`, `cargo.add`, `cargo.items`, `cargo.for_dest`)으로
  - station: 충전 자리를 `ChargerSlots` 로 (예전 `charger_spots/occ/fifo` 는 base 에서 없어짐)
  - `extended_runtime.py` 와 그 테스트도 같은 방식으로
  - 가짜 ROS 환경에서 경북대 하역 한 바퀴(일반·고장·충전 칸 3대), 충전소 하역·도크 비우기 동작 확인
- CI: `package.xml` 에 `python3-scipy` 추가, `test_navigation` 의 가짜 `free_spot` 을 A 의 인자에 맞춤
- 확장 프로필 해시 목록: 기본 파일을 일부러 고쳤으면 `python3 tools/update_legacy_hashes.py` 로 갱신하고 같이 커밋

확인 요청:
- **A**: `mission_manager_campus.py` 를 D 구조에 맞춘 것 (동작은 그대로, 적재함 다루는 방식만 바뀜). KNU 시나리오 회귀 한 번
- **C**: `nav_bridge.py` 병합 결과 (A 의 nearest_free 대체 목표가 C 의 재시도 흐름 안에서 도는지)
- **D · A**: `collector.py` 를 D 버전으로 정리한 것, `motion.py` 의 A 쪽 `_at` 방식을 D 히스테리시스로 대체한 것
- **D**: 새 `nav_logic.py` · `lidar_logic.py` 등 제안 파일은 아직 노드가 쓰지 않음. `nav_logic.choose_goal` 에는 A·C 의 최신 목표 선택(가장 가까운 빈 자리, 실패 사유)이 없음
- **전체**: D 의 CI(`.github/workflows/ci.yml`)는 모든 push 에서 돈다. scipy·해시 문제를 고쳐 통과함. 확장 프로필 유지·삭제는 아직 결정 필요

### 12-1. 이번에 main 에 합친 것

| 순서 | 브랜치 | 내용 | 병합 시 처리 |
|---|---|---|---|
| 0 | (공통) | `.gitattributes` 추가: 줄바꿈을 LF 로 고정 | Windows 에서 파일 전체가 CRLF 로 바뀌는 문제 방지 |
| 1 | role_B | 요청 임계값 45°/15% 확정, 가중치 10/1/5 유지 확정(실험 결과 10장), t_max 보장 범위 정정, RViz 대기열 마커 2줄(순위·대기 시간) | 충돌 없음 |
| 2 | role_C | `nav_bridge.py` 주행 안전 처리(동시 goal 거절, 센서 타임아웃, 미측정 빔 처리, 직선 offset 대체 제거, 정지 보장), `planner.py`(floor 좌표, 대각선 모서리 통과 금지, 미측정 빔이 장애물을 지우지 않음), `test_navigation.py` 17개, 타임아웃 파라미터 3개 | role_C 의 CRLF 변환(`0105a1e`)을 LF 로 정규화해서 합침. 실제 변경 5개 파일만 반영 |
| 2-1 | role_C | `motion.py` 의 테스트용 `"test2"` 줄 제거 (`8c746cd`, `358ea2a`) | 별도 커밋 |
| 3 | role_A | `scripts/build_env.py`(도로 맵 생성기), `scripts/check_env.py`(사전 점검), `GAZEBO_COMMANDS.md`, `docs/SIM_GUIDE.html` | 새 파일만 추가. 충돌 없음 |

### 12-2. 병합본 검증 결과

ROS 가 없는 환경에서 확인할 수 있는 것만 돌렸다. **Gazebo/ROS 실행 확인은 각 담당자가 Ubuntu 에서 해야 한다.**

| 항목 | 결과 |
|---|---|
| 전체 `.py` 문법 검사 (`py_compile`) | 통과 |
| `test/test_logic.py` | 8/9 통과. 실패 1개(`test_sim_localizer_map_odom`)는 `rclpy` 가 없는 환경 문제 |
| `test/test_navigation.py` 중 planner 테스트 4개 | 4/4 통과. 나머지 13개는 ROS 필요 (미실행) |
| `robot_params.yaml` 파싱 | 통과 (B 의 가중치 주석 + C 의 타임아웃 파라미터 모두 반영) |
| `build_env.py street` | 통과. world · 지도 2장 · 맵 설정 · 배치도 생성 |
| `check_env.py` | **실패**. C 담당 패치(`Costmap.set_physical`, `free_spot(keep_clearance=)`)가 아직 없음 → 12-3 참고 |

### 12-3. 아직 브랜치에 없는 파일 (담당자별 요청 목록)

`GAZEBO_COMMANDS.md`, `docs/SIM_GUIDE.html` 에 적혀 있지만 어떤 브랜치에도 없는 것들이다.
**이것들이 들어와야 `GAZEBO_COMMANDS.md` 의 명령(s10~s19 등)이 그대로 동작한다.**

#### role_A (haein0248)

- [ ] 시나리오 `s10_road_collect` ~ `s19_peak_stream` (10개)
- [ ] 시나리오 `s1` ~ `s9` 도로판 (좌표·배터리 초기값·s5 구성 변경. 지금 main 에는 실내판만 있음)
- [ ] `run_gazebo.sh`
- [ ] launch 수정, `map_utils.py`
- [ ] `scooter_fleet_sim.py`, `gz_utils.py`, `fake_base.py` 수정 (새 토픽 `/sim/obstacles` 포함)
- [ ] 도로 맵 생성물: `worlds/street.sdf`, `maps/street*.pgm|yaml`, `maps/street_layout.*`, `config/maps/street.yaml` (`build_env.py` 로 생성 가능하지만 launch 가 참조하므로 커밋 필요)
- [ ] `src/kickboard_sim/archive/` (실내 시나리오 17개, 캠퍼스 시나리오 6개, 캠퍼스 맵)
- [ ] 다른 담당자 파일 패치 4개 (`my_ws/patches_for_team/`) → 아래 각 담당자에게 전달하거나 해당 브랜치로 PR

#### role_C (monn098) — A 가 만든 패치 반영

- [ ] `C_nav_lidar.patch`: 인도 진입 금지(`set_physical`, `keep_clearance`), 도킹 방향 맞춤, 스캔 시각 보정, 가까운 빈 자리, 장애물 스캔 반영
  - 이미 main 에 들어간 C 의 `nav_bridge.py` · `planner.py` 변경과 **같은 파일**이므로, 최신 main 위에서 합쳐야 한다
  - 이것이 들어와야 `check_env.py` 가 동작한다
- [ ] `motion.py`, `sim_lidar.py`, `common.py` 수정 (A 문서 기준)

#### role_D — A 가 만든 패치 반영

- [ ] `D_collector_mission.patch`: `collector.py` · `mission_manager.py` (팔 도달 거리 확인, 상태 되돌아감 수정)

#### role_B (Lamelo0721)

- [ ] 가중치 실험에 쓴 **s5 변형 시나리오 파일** (10장 "s5 를 바꾼 실험용 배치". 재현용)
- [ ] A 가 만든 `B_queue_manager.patch`(보고 끊긴 대기 항목 자동 취소) 반영 여부 결정
  - `queue_manager.py` 는 이번에 B 의 마커 변경이 들어갔으므로 최신 main 위에서 합쳐야 한다
- [ ] `B_README_additions.patch` 는 A 문서상 "옛 내용"이라 반영 불필요

#### 출처 확인 필요 (팀 전체)

- [ ] 11/16 22:31 A 의 워크스페이스에 추가된 **"확장 프로필"**: `extended.launch.py`, `extended_*.py` 노드 8개, `config/scenarios_extended/`(별도 s10~s19), `street_extended` · `campus_extended` 맵, 테스트 `test_legacy_manifest_hashes`
  - 누가 만들었는지, A 의 s10~s19 와 어느 쪽을 기준으로 할지 정해야 한다 (이름이 겹침)

### 12-4. 결정이 필요한 것

1. **C: 대기 시간 기준.** `nav_bridge.py` 가 `sleep_sim`(시뮬 시간) 대신 `time.sleep`(실제 시간)을 쓴다. 시뮬을 느리게/일시정지하면 대기·타임아웃이 시뮬 시간과 어긋난다. 센서 타임아웃은 의도(파라미터 주석)로 보이지만 주행 루프 주기까지 실제 시간인 것이 맞는지 확인.
2. **B·C: s5 의 `no path`.** B 의 가중치 실험에서 4번 (−3.0, −2.6), 원래 s5 1번 (3.5, −3.0) 까지 경로가 없었다. C 의 대각선 통과 금지가 들어간 지금 main 에서 다시 확인.
3. **A · 확장 프로필 중 기준 결정** (12-3 마지막 항목).

### 12-5. 앞으로의 작업 규칙

1. **작업 시작 전에 최신 main 을 받는다.** `git pull origin main` (자기 role 브랜치에서)
2. **하루 이상 PC 에만 두지 않는다.** 작업 단위가 끝나면 커밋하고 push.
3. **main 에는 PR 로만 합친다.** 자기 role 브랜치 → main 으로 PR, 병합 관리자가 확인 후 병합. 주 1~2회 이상.
4. **다른 담당자 파일은 직접 고치지 않는다.** 필요하면 담당자에게 요청하거나, 그 담당자 브랜치로 PR.
5. **커밋 전에 `git status`, `git diff` 로 확인한다.** 줄바꿈 변경, 테스트용 코드가 섞이지 않게.
6. Windows 사용자는 한 번 설정: `git config --global core.autocrlf input`
