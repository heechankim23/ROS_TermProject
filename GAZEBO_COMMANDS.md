# Gazebo 실행 명령 (Ubuntu 22.04 · ROS 2 Humble · Gazebo Harmonic)

도로 맵(`street`) 시나리오 19개 기준. 모든 명령은 워크스페이스(clone 한 `ROS_TermProject` 폴더) 안에서 실행한다.

> **이 브랜치에 아직 없는 것 (role_A 후속 커밋·다른 역할 패치 필요)**
> 이 문서는 도로 맵 작업 전체를 기준으로 쓰였다. 지금 커밋에는 `scripts/build_env.py`, `scripts/check_env.py`, 이 문서, `docs/SIM_GUIDE.html` 만 들어 있다.
> 아래가 함께 들어와야 문서의 명령이 그대로 동작한다.
> - A 담당: 도로 맵 생성물(`worlds/street.sdf`, `maps/street*`, `config/maps/street.yaml` — `build_env.py` 를 실행하면 만들어짐),
>   시나리오 `s1`~`s19` 도로판, launch 수정(`map_utils.py` 포함), `scooter_fleet_sim.py`·`gz_utils.py`·`fake_base.py` 수정, `run_gazebo.sh`
> - C 담당: `nav_bridge.py`·`planner.py`·`motion.py`·`sim_lidar.py`·`common.py` 수정 (인도 진입 금지, 장애물 스캔 반영 등). `check_env.py` 도 이 수정에 의존한다
> - D 담당: `collector.py`·`mission_manager.py` 수정 / B 담당: `queue_manager.py` 수정

## 1. 처음 한 번 (또는 코드를 고친 뒤)

```bash
cd <ROS_TermProject 폴더>
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

`colcon build` 를 하지 않으면 고친 코드·시나리오가 반영되지 않는다.

## 2. 시나리오 실행

새 터미널을 열 때마다 아래 세 줄을 먼저 한다.

```bash
cd <ROS_TermProject 폴더>
source /opt/ros/humble/setup.bash
source install/setup.bash
```

그다음 시나리오 하나를 골라 실행한다 (한 번에 하나만. 끝내려면 Ctrl+C).

```bash
ros2 launch kickboard_sim bringup.launch.py scenario:=s1_single
```

위 네 줄을 한 번에 해 주는 스크립트 (빌드가 필요하면 빌드까지 한다):

```bash
cd <ROS_TermProject 폴더>
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
