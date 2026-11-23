# Gazebo 실행 명령 (Ubuntu 22.04 · ROS 2 Humble · Gazebo Harmonic)

도로 맵(`street`) 시나리오 19개 기준. 워크스페이스: `~/lecture_2026_2/Ros_TermProject/my_ws`

## 1. 처음 한 번 (또는 코드를 고친 뒤)

```bash
cd ~/lecture_2026_2/Ros_TermProject/my_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

`colcon build` 를 하지 않으면 고친 코드·시나리오가 반영되지 않는다.

## 2. 시나리오 실행

새 터미널을 열 때마다 아래 세 줄을 먼저 한다.

```bash
cd ~/lecture_2026_2/Ros_TermProject/my_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

그다음 시나리오 하나를 골라 실행한다 (한 번에 하나만. 끝내려면 Ctrl+C).

```bash
ros2 launch kickboard_sim bringup.launch.py scenario:=s1_single
```

위 네 줄을 한 번에 해 주는 스크립트 (빌드가 필요하면 빌드까지 한다):

```bash
cd ~/lecture_2026_2/Ros_TermProject/my_ws
./run_gazebo.sh                    # 시나리오 목록
./run_gazebo.sh s10_road_collect   # 실행
```

### 기존 9개 (도로 맵으로 옮김)

```bash
ros2 launch kickboard_sim bringup.launch.py scenario:=s1_single
ros2 launch kickboard_sim bringup.launch.py scenario:=s2_new_request
ros2 launch kickboard_sim bringup.launch.py scenario:=s3_full_load
ros2 launch kickboard_sim bringup.launch.py scenario:=s4_low_battery
ros2 launch kickboard_sim bringup.launch.py scenario:=s5_aging
ros2 launch kickboard_sim bringup.launch.py scenario:=s6_verify_fail
ros2 launch kickboard_sim bringup.launch.py scenario:=s7_obstacle
ros2 launch kickboard_sim bringup.launch.py scenario:=s8_auto_cancel
ros2 launch kickboard_sim bringup.launch.py scenario:=s9_drop_sites
```

### 현실 상황 10개

```bash
ros2 launch kickboard_sim bringup.launch.py scenario:=s10_road_collect
ros2 launch kickboard_sim bringup.launch.py scenario:=s11_side_street_blocked
ros2 launch kickboard_sim bringup.launch.py scenario:=s12_bus_stop_cluster
ros2 launch kickboard_sim bringup.launch.py scenario:=s13_rent_waiting
ros2 launch kickboard_sim bringup.launch.py scenario:=s14_rent_in_progress
ros2 launch kickboard_sim bringup.launch.py scenario:=s15_moved_waiting
ros2 launch kickboard_sim bringup.launch.py scenario:=s16_refall_after_drop
ros2 launch kickboard_sim bringup.launch.py scenario:=s17_night_battery
ros2 launch kickboard_sim bringup.launch.py scenario:=s18_double_parked
ros2 launch kickboard_sim bringup.launch.py scenario:=s19_peak_stream
```

## 3. 옵션 (명령 뒤에 붙인다)

| 붙이는 것 | 뜻 |
|---|---|
| `rviz:=true` | RViz 도 함께 띄움 (대기열 마커, 스캔) |
| `gui:=false rviz:=true` | Gazebo 화면 없이 RViz 로만 봄 (PC 가 느릴 때) |
| `gui:=false sensors:=false` | 화면 없이 로그만 (SSH) |
| `sim:=kinematic rviz:=true` | Gazebo 없이 로직만 빠르게 |
| `arm:=false` | 로봇팔 동작 대신 3초 정지로 적재 (빨리 보고 싶을 때) |
| `start_delay:=15` | Gazebo 가 늦게 뜨는 PC 에서 노드 시작을 늦춤 |

예:

```bash
ros2 launch kickboard_sim bringup.launch.py scenario:=s10_road_collect gui:=false rviz:=true
```

## 4. 실행 중 확인 (다른 터미널, 2장의 세 줄 먼저)

```bash
ros2 topic echo /robot_status        # 로봇 상태·배터리·적재 수
ros2 topic echo /queue_status        # 대기열과 점수
ros2 topic echo /mission_events      # 이벤트 로그 (수거, 하역, 실패)
ros2 topic echo /admin_alerts        # 관리자 알림 (사람이 처리할 건)
ros2 param set /arm_loader time_scale 2.0   # 로봇팔 동작을 2배 느리게
```

정상 진행 순서: 킥보드 배치 → 약 10초 뒤 `[상태] SELECTING -> NAVIGATING` → `VERIFYING` → `LOADING` → (반복)
→ `DROPPING`(주차구역) → `RETURNING` → `DOCKING` → `CHARGING` → `IDLE`. 마지막 `IDLE` 이 나오면 끝.

## 5. 지표 기록

```bash
ros2 bag record -o bags/s10 /mission_events /robot_status /queue_status
```

시나리오가 끝나면 Ctrl+C 후:

```bash
ros2 run kickboard_robot analyze bags/s10
```

## 6. 자주 나는 문제

| 증상 | 조치 |
|---|---|
| `Package 'kickboard_sim' not found` | `source install/setup.bash` 를 안 했다 (2장의 세 줄) |
| 고친 내용이 반영 안 됨 | `colcon build` 후 `source install/setup.bash` |
| Gazebo 창은 떴는데 로봇이 안 움직임 | 킥보드 배치 후 10초 대기가 정상. 30초 넘게 그대로면 `start_delay:=15` |
| 이전 Gazebo 가 남아 로봇이 두 대 보임 | `pkill -f "gz sim"` 후 다시 실행 |
| 시나리오 좌표를 바꿨는데 로봇팔이 안 닿음 | `python3 src/kickboard_sim/scripts/check_env.py --all` 로 사전 점검 |

## 7. 맵·시나리오를 고칠 때

```bash
# 시나리오: src/kickboard_sim/config/scenarios/*.yaml 을 고치거나 새로 추가
python3 src/kickboard_sim/scripts/check_env.py --all     # 팔이 닿는지, 길이 있는지 점검

# 도로 배치: src/kickboard_sim/scripts/build_env.py 의 street() 를 고친 뒤
python3 src/kickboard_sim/scripts/build_env.py           # world·지도·설정·배치도 다시 생성
colcon build && source install/setup.bash
```

실내·캠퍼스 시나리오와 캠퍼스 맵은 `src/kickboard_sim/archive/` 에 보관되어 있다 (빌드·실행 대상에서 빠짐).

## 8. 시나리오 전체를 한 번에

```bash
./run_all_gazebo.sh                        # 19개를 하나씩 차례로, 화면 없이 (2시간 안팎)
GUI=1 ./run_all_gazebo.sh                  # Gazebo 화면을 띄워서
./run_all_gazebo.sh s1_single s10_road_collect   # 고른 것만
```

결과 요약은 `results/<날짜_시각>/summary.txt`, 시나리오별 로그는 같은 폴더에 남는다. Ctrl+C 로 중단할 수 있다.

## 9. 킥보드 충전소가 따로 있는 버전 (`street_cs`)

기본 맵(`street`)에서는 배터리 부족 킥보드를 로봇 충전 도크 양옆에 내려놓는다.
`street_cs` 는 같은 도로에 **킥보드 충전소(CS)** 를 뒷길 쪽에 따로 둔 버전이다:
수거 → (배터리 정상) 주차구역 하역 → (배터리 부족) 킥보드 충전소로 이동·하역 → 로봇 도크 복귀 → 로봇 충전.

어떤 시나리오든 뒤에 `map:=street_cs` 를 붙이면 이 버전으로 돈다 (기존 실행은 그대로).

```bash
ros2 launch kickboard_sim bringup.launch.py scenario:=s10_road_collect map:=street_cs
ros2 launch kickboard_sim bringup.launch.py scenario:=s9_drop_sites map:=street_cs
ros2 launch kickboard_sim bringup.launch.py scenario:=s17_night_battery map:=street_cs
./run_gazebo.sh s10_road_collect map:=street_cs
```

- 로그에 `[상태] SELECTING -> DROPPING ... -> 킥보드 충전소 CS (N대)` 와 `킥보드 충전소 CS 하역 N대` 가 나오면 이 버전이 동작한 것이다.
- 충전소 위치·크기: `scripts/build_env.py` 의 `street_cs()` (`e.scooter_station(...)`). 고친 뒤 `build_env.py` → `colcon build`.
- 미션 노드: `kickboard_robot/mission_manager_station.py` (기존 `mission_manager.py` 의 서브클래스, 기존 파일은 고치지 않음).
  맵 설정에 `mission_variant: station` 이 있으면 launch 가 이 노드를 `mission_manager` 이름으로 띄운다.
- 충전 자리는 4곳. 다 차면 가장 오래 맡긴 킥보드를 '충전 끝'으로 치우고 그 자리를 쓴다.

## 10. 경북대 캠퍼스 맵 (`knu_campus`) — 역할 ① 맵 제작

경북대 대구캠퍼스 전체(약 1.2 × 1.2 km)를 실제 크기로 만든 맵이다. 건물 94동(공식 안내판 번호·이름, 5개 구역 색), 도로·보행로·계단,
일청담·운동장·테니스장, SRTM 고도로 만든 언덕·경사, 일반 주차구역 32곳과 충전 주차구역 8곳(거치대 슬롯 포함)이 들어 있다.

### Gazebo 로 보기 (노드 없이 맵만)

```bash
cd ~/lecture_2026_2/Ros_TermProject/my_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select kickboard_sim
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix kickboard_sim)/share/kickboard_sim/models
gz sim $(ros2 pkg prefix kickboard_sim)/share/kickboard_sim/worlds/knu_campus.sdf        # 지형 포함
gz sim $(ros2 pkg prefix kickboard_sim)/share/kickboard_sim/worlds/knu_campus_flat.sdf   # 같은 배치, 평지
```

- 캠퍼스가 커서 처음 화면은 멀리 보인다. 오른쪽 Entity Tree 에서 `kickboard_racks`·`charging_station` 을 골라 **Move to** 하면 가까이 간다.
- 웹 2D/3D 지도: `src/kickboard_sim/maps/knu_campus_viewer.html` 을 브라우저로 연다 (인터넷 필요: three.js CDN).
- 문서용 2D 지도: `src/kickboard_sim/maps/knu_campus_2d.png`

### 맵을 고칠 때

```bash
cd ~/lecture_2026_2/Ros_TermProject/my_ws/src/kickboard_sim
python3 scripts/build_knu_campus.py      # 약 40초. world·메시·지도·맵 설정·2D 지도·웹 지도를 모두 다시 만든다
cd ~/lecture_2026_2/Ros_TermProject/my_ws && colcon build --packages-select kickboard_sim
```

- 충전구역 후보 8곳: `build_knu_campus.py` 의 `CHARGING_SITES` (기준 건물·장소). 후보에서 45 m 안에 앱 주차 가능 구역이 있으면 그 구역을, 없으면 새 구역을 만든다.
- 거치대 칸 수: `PARK_SLOTS`(정상 8) · `BROKEN_SLOTS`(고장 분류 2, 반대 방향) · `CHARGE_SLOTS`(충전 8)
- 로봇 도크 위치: `ROBOT_DOCK_ANCHOR` (지금은 본관 100 옆)
- 한글이 깨진 2D 지도가 나오면: `sudo apt install fonts-noto-cjk`

### 만든 파일 (`src/kickboard_sim/`)

| 파일 | 내용 |
|---|---|
| `worlds/knu_campus.sdf`, `knu_campus_flat.sdf` | Gazebo world (지형 / 평지). world 프레임 = map 프레임, 지형 z 는 로봇 도크 = 0 |
| `models/knu_campus/meshes/` | 지형(텍스처: 도로·보행로·잔디·등고선·주차 칸), 건물(구역 색), 거치대·충전 스탠드·출입문 메시 |
| `config/maps/knu_campus.yaml` | GPS 기준점, 로봇 도크, `parking_zones`(일반) · `charging_zones`(충전): 폴리곤 · 거치대 · 로봇 접근 자리 · 슬롯(x, y, z, yaw, role) |
| `maps/knu_campus.png/yaml` | 주행 지도 0.25 m (건물·물·캠퍼스 밖 + 운동장·계단 진입 금지) |
| `maps/knu_campus_lidar.png/yaml` | 물리 지도 (건물·물·캠퍼스 밖만) |
| `maps/knu_campus.json` | 모든 레이어 + 고도 격자(16 m) + 등고선 + 구역·슬롯. 시각화(④)·시나리오(③)용 |
| `maps/knu_campus_src/` | 원본: OSM 캠퍼스 데이터, 앱 주차 가능 구역 이미지와 정합 결과, SRTM 고도 |

### 구역 규칙

- **일반 주차구역 32곳** = 킥보드 앱의 주차 가능 구역(사용자 제공 이미지의 흰색·하늘색 부분)을 OSM 경계에 맞춰 그대로 옮긴 것. 구역마다 길가에 거치대 1개(정상 8칸 + 고장 분류 2칸).
- **충전 주차구역 8곳**: 중앙도서관 앞, 편의점 인근(복지관), 일청담–사범대 사이, 글로벌플라자 광장, 인문대 쪽, 공식당 쪽(공대9호관), 테니스장 쪽, 대운동장 쪽. 충전 스탠드 8칸.
- 슬롯 `yaw` 는 킥보드 앞(핸들)이 향하는 방향. 정상·충전 칸은 거치대 안쪽(길 반대)을, **고장 분류 칸은 반대 방향**을 향한다.
- `robot_access` = 로봇이 길 위에 서서 로봇팔로 슬롯에 닿는 자리 (슬롯 중심까지 약 1.75 m).

### 아직 안 된 것 (역할 ②·③에서 이어서)

- 기존 노드(`nav_bridge` A*, `mission_manager`, `scooter_fleet_sim`)는 아직 이 맵에 맞추지 않았다. 지도가 5000 × 5160 칸이라 기존 A* 그대로는 느리고,
  지형 높이(z)를 쓰는 노드가 없다. 노드 검증은 `knu_campus_flat` 부터 하는 것을 권장.
- 이 PC(WSL)에서는 Gazebo 카메라 렌더링이 안 돼서 Gazebo 화면 캡처로는 확인하지 못했다. world 로드와 물리 300 스텝은 오류 없이 돈다.

## 11. 경북대 캠퍼스 시나리오 (`config/scenarios_knu/`) — 역할 ③ 시나리오 설계

시나리오 32개(k01~k32): A 상태별 기본 8 · B 복합 판단 4 · C 수용량·적재량 7 · D 운영 중 돌발상황 13.
상황·임무·킥보드별 예상 처리·통과 기준은 `src/kickboard_sim/config/scenarios_knu/README.md`, 지도로 보려면 같은 폴더 `catalog.html`.

```bash
cd ~/lecture_2026_2/Ros_TermProject/my_ws/src/kickboard_sim
python3 scripts/build_knu_scenarios.py     # YAML 32개 + README + catalog 다시 생성, 좌표·로봇 접근·조건 검사
```

- 시나리오는 장소(건물 출입구, 보행로, 잔디, 슬롯)로 정의하고 스크립트가 캠퍼스 맵에서 좌표를 고른다. 좌표를 직접 고치지 말고 `build_all()` 을 고친다.
- 기존 형식(scooters·events·robot)을 그대로 쓰고 필드를 더했다: `fault`, `slot`, `charging`, `silent`, `false_report`, `overrides.charging_capacity`,
  이벤트 `report`·`fault`·`park`·`stand_fault`·`zone_closed`·`gps_drift`·`robot_fault`.
- **아직 바로 실행되지는 않는다.** 새 필드·이벤트는 ② 시뮬레이션 환경에서 `scooter_fleet_sim`·`mission_manager` 에 구현해야 한다.
  지금 `scooter_fleet_sim` 은 `id` 없는 이벤트(`stand_fault`, `zone_closed`, `gps_drift`, `robot_fault`)를 만나면 KeyError 로 멈춘다.

## 12. 캠퍼스 시뮬레이션 실행 (역할 ②) · 운영 현황 시각화 (역할 ④)

### 시나리오 실행

```bash
cd ~/lecture_2026_2/Ros_TermProject/my_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select kickboard_interfaces kickboard_robot kickboard_sim
source install/setup.bash
ros2 launch kickboard_sim bringup.launch.py scenario:=k00 start_delay:=15          # Gazebo (평지판 캠퍼스)
ros2 launch kickboard_sim bringup.launch.py scenario:=k01 sim:=kinematic            # Gazebo 없이 로직만
ros2 launch kickboard_sim bringup.launch.py scenario:=k01 map:=knu_campus           # 언덕 지형 world
```

`scenario:=k00` ~ `k32` (config/scenarios_knu). 맵은 시나리오의 `map` 키(기본 `knu_campus_flat`).

### 운영 현황 보기 (실행 중, 다른 터미널)

| 무엇을 | 명령 |
|---|---|
| 지금 하는 일·어디로·왜·목적지·정렬 결과 (JSON) | `ros2 topic echo /ops_status` |
| RViz 지도 위 표시 (킥보드 상태 색, 로봇 경로, 목적지 화살표, 구역, 정렬 완료 칸) | `ros2 launch kickboard_sim bringup.launch.py scenario:=k08 rviz:=true` |
| Gazebo 패널 (Kickboard status → Echo) | 맨 위에 `[지금]`, `[이유]`, `[적재]`, `[결과]` 줄 |
| 거치대 슬롯 점유 | `ros2 topic echo /zone_occupancy` |
| 관리자 알림 | `ros2 topic echo /admin_alerts` |

### 리플레이 (실행 뒤)

`ops_monitor` 가 실행마다 `~/.ros/kickboard_ops/<시나리오>_<시각>.jsonl` 을 남긴다. 시나리오가 끝나면 Ctrl+C 로 끄고:

```bash
cd ~/lecture_2026_2/Ros_TermProject/my_ws/src/kickboard_sim
python3 scripts/build_ops_replay.py                      # 시나리오별 최신 기록 -> my_ws/results/ops_replay.html
xdg-open ~/lecture_2026_2/Ros_TermProject/my_ws/results/ops_replay.html
```

리플레이: 캠퍼스 지도 위 로봇 이동·킥보드 상태 변화 재생(×1~×60), '지금 로봇은' 카드(어디로·왜·대상·일반/충전 목적지·적재함),
정리 결과(정렬 완료 수·진행률), 이벤트 흐름, 킥보드별 결과 표(처음 상태 → 사유 → 보낸 곳 → 정렬 칸 → 완료 시각).

## 13. 캠퍼스 시나리오 전체 확인 + 전체 리플레이 (`run_all_knu.sh`)

시나리오별 명령 전체 목록: [KNU_COMMANDS.md](KNU_COMMANDS.md) 2번

```bash
cd ~/lecture_2026_2/Ros_TermProject/my_ws
./run_all_knu.sh --gui --rtf 4            # k00~k32 전부 Gazebo 화면 + 4배속, 하나씩 (권장)
./run_all_knu.sh --gui --rtf 4 k32 k08 k13 # 고른 것만 (발표 데모)
./run_all_knu.sh                          # 화면 없이 (Gazebo 서버만)
./run_all_knu.sh --speed 2.5              # 로봇 속도를 올려 빨리
./run_all_knu.sh --kinematic -j 4         # Gazebo 없이 로직만, 4개 동시에
```

- 로봇이 대기(IDLE)로 돌아온 뒤 `--grace`(기본 200초) 동안 새 일이 없으면 그 시나리오를 끝내고 다음으로 넘어간다. 최대 `--limit`(기본 3600초).
- 끝나면 `results/knu_<날짜_시각>/summary.txt`(시나리오별 통과/부분, 어긋난 킥보드) 와 `results/ops_replay.html`(전체 리플레이)을 만든다.
- 요약·리플레이만 다시: `python3 src/kickboard_sim/scripts/check_knu_runs.py` → `python3 src/kickboard_sim/scripts/build_ops_replay.py`

## 14. 빨리 보기: 시나리오 전용 작은 world + 배속

캠퍼스 전체 world 는 무겁고 이동 거리가 길다. 시나리오마다 그 주변만 잘라 만든 작은 world(`knu_k00`~`knu_k32`)와 배속을 쓴다.

```bash
ros2 launch kickboard_sim bringup.launch.py scenario:=k08 map:=local rtf:=4 start_delay:=15
```

- `map:=local`: 시나리오 전용 작은 world (킥보드·관련 구역·근처로 옮긴 로봇 도크만, 바닥 충돌은 평면)
- `rtf:=4`: Gazebo 4배속 (모든 노드가 시뮬레이션 시간을 따름). PC 가 따라가지 못하면 2~3 으로 낮춘다
- 작은 world 를 다시 만들기: `python3 src/kickboard_sim/scripts/build_scenario_worlds.py` (시나리오를 고친 뒤)
- 전체 자동 실행도 작은 world 가 기본: `./run_all_knu.sh --gui --rtf 4`
