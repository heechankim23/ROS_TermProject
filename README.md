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
| 기둥·핸들 | 보라 | 배터리 < 15% (사유와 상관없이 → 충전소로 갈 킥보드) |
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
| 배터리 < 15% (사유가 넘어짐·구역 밖이어도) | **충전소** — 도킹 후 로봇팔이 꺼내 충전소 양옆 **충전 자리**(4곳)에 세운다 (충전하러 맡김). 자리가 다 차면 가장 오래 맡긴 킥보드를 '충전 끝'으로 치우고 그 자리를 쓴다 |
| 그 외 (넘어짐·구역 밖, 배터리 정상) | **주차구역** — `DROPPING` 상태: 가장 가까운 주차구역 변 앞에 서서 로봇팔이 앞 칸부터 꺼내 구역 안에 나란히 세운다. 세운 킥보드는 다시 정상 킥보드가 된다 (넘어지면 다시 요청) |

만재이거나 대기열이 비면 주차구역행이 있을 때 먼저 `DROPPING`, 그다음 충전소행이 있거나 할 일이 없으면 충전소로 간다.
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
| `s5_aging` | 먼 곳 1건 + 가까운 요청 25초마다 | 먼 항목이 t_max 안에 선택 |
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

요청 조건(펌웨어 판정): 기울기 > 45° → `FALLEN`, 주차구역 밖 → `OUT_OF_ZONE`, 배터리 < 15% → `LOW_BATTERY`.

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
`queue_manager` 시작 로그의 `t_max` 가 점수식의 기아 방지 상한이다 (기본 가중치 10/1/5, d_max 12.8 m → 84 s).

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
