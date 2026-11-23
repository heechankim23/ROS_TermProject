# 확장 시나리오 프로필 — 기존 9개 + 신규 10개

원본 `config/scenarios/`의 s1~s19와 기존 제어 노드는 변경하지 않았다. 새 프로필은 원본 s1~s9와 이 폴더의 새 s10~s19를 묶은 **19개 실행 목록**이다. 두 프로필을 합친 전체 저장소에는 29개의 서로 다른 시나리오 YAML이 있다. 기존 실행은 `bringup.launch.py`, 새 실행은 `extended.launch.py`를 사용한다. 기존 s10~s19도 이전 명령으로 실행할 수 있다.

## 빌드 및 실행

Ubuntu 22.04 / ROS 2 Humble / Gazebo Harmonic / ros_gz 환경에서 실행한다. 기존 ros_gz overlay가 있다면 그 환경도 먼저 source한다.

```bash
cd /home/haein/lecture_2026_2/Ros_TermProject/my_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kickboard_interfaces kickboard_robot kickboard_sim
source install/setup.bash
ros2 launch kickboard_sim extended.launch.py scenario:=s10_damaged_kickboard sim:=gazebo nav:=nav2
```

Nav2가 없다면 우분투에 `ros-humble-navigation2`, `ros-humble-nav2-bringup`이 필요하다. 기본 `nav:=nav2`는 AMCL을 사용한다. `nav:=simple`은 기존 A* 제어와 Gazebo 정답 위치 보정으로 동작하므로 위치 추정 성능 검증에 사용하지 않는다.

한 번에 시뮬레이션 하나만 실행한다. 각 시험 종료는 Ctrl+C 후 노드와 Gazebo 프로세스 종료를 확인한다. `sim:=gazebo` 별칭은 **새 extended.launch.py에서 지원**한다. 기존 launch의 `sim:=gz`는 그대로 유지된다.

```bash
# 40×30 m 캠퍼스 차도 맵: 건물 용도를 학생회관/연구동으로 바꾼 단순화 배치
ros2 launch kickboard_sim extended.launch.py scenario:=s12_clustered_blockage sim:=gazebo nav:=nav2 map:=campus_extended
# 물리 없이 빠른 상태 머신 확인
ros2 launch kickboard_sim extended.launch.py scenario:=s15_overload_weight sim:=kinematic nav:=simple arm:=false
# 기존 s1은 원본 launch로 위임: 기존 동작·설정 유지
ros2 launch kickboard_sim extended.launch.py scenario:=s1_single sim:=gazebo nav:=simple
```

## 신규 10개 개별 실행

```bash
ros2 launch kickboard_sim extended.launch.py sim:=gazebo scenario:=s10_damaged_kickboard
ros2 launch kickboard_sim extended.launch.py sim:=gazebo scenario:=s11_nighttime_collection
ros2 launch kickboard_sim extended.launch.py sim:=gazebo scenario:=s12_clustered_blockage
ros2 launch kickboard_sim extended.launch.py sim:=gazebo scenario:=s13_pedestrian_avoidance
ros2 launch kickboard_sim extended.launch.py sim:=gazebo scenario:=s14_gps_denied_zone
ros2 launch kickboard_sim extended.launch.py sim:=gazebo scenario:=s15_overload_weight
ros2 launch kickboard_sim extended.launch.py sim:=gazebo scenario:=s16_docking_failure
ros2 launch kickboard_sim extended.launch.py sim:=gazebo scenario:=s17_dynamic_car_parking
ros2 launch kickboard_sim extended.launch.py sim:=gazebo scenario:=s18_weather_slippery
ros2 launch kickboard_sim extended.launch.py sim:=gazebo scenario:=s19_cancel_during_transit
```

## 상태 전이와 판정

공통: SELECTING → NAVIGATING → VERIFYING → ALIGNING(상태 토픽에는 LOADING) → LOADING → SELECTING. 정상 배터리는 P N1/E1/S1 중 가능한 하역점, 방전 킥보드는 충전소로 운반한다. 신규 정책은 기존 노드의 서브클래스 또는 별도 복사본에서만 적용된다.

| 시나리오 | 자극과 예외 처리 | 통과 기준 |
|---|---|---|
| s10 파손 | 검정 발판·빨강 기둥 및 damaged 플래그. 선택 단계에서 NEEDS_HUMAN, 다음 대상으로 | 1번 적재 0회, 관리자 알림, 2번 수거 |
| s11 야간 | 어두운 월드·로봇 전조등, 모의 검출 거리 2.2 m·위치 잡음 5 cm, 탐색 시간 25초 | 조명 확인, 두 대 수거 또는 제한 시간 내 명시적 실패. 미검출을 성공 처리하지 않음 |
| s12 군집 | 세 대 실물은 처음부터 스폰. 1→2→3 순서로 요청 허용, 앞 대상 적재 확인 후 다음 요청 | 순서대로 3대 수거, 부딪힘·인도 진입 없음 |
| s13 보행자 | 이동 시작 시 횡단 보행자 왕복 투입, 스캔 기반 정지 후 재개·Nav2 재계획 | 위험 접근에서 guard obstacle_stop, 안전 거리 확보 후 재개, 충돌 없음 |
| s14 GPS 음영 | 건물 사이 **킥보드 보고 GPS** σ=0.8 m, 40초 후 0.15 m 복원; 태그로 수거 좌표 보정 | request_sent 좌표와 collected.corrected 좌표 비교. 실제 대상 적재 또는 탐색 실패 처리 |
| s15 중량 | 32 kg씩 세 대, 한도 50 kg. 추가 적재 전 예상 합산 중량 검사, REQUEUE → DROPPING/RETURNING | 동시에 2대 적재 금지, 하역 후 남은 요청 처리. 개별 한도 초과 대상은 NEEDS_HUMAN |
| s16 도킹 실패 | 실제 정렬 수행 후 확인 결과를 처음 2회 실패로 주입. 재접근·정렬 | dock_retry 2회 후 docked. 실패 횟수를 3 이상으로 바꾼 음성 시험에서는 NEEDS_HUMAN, 충전·하역 금지 |
| s17 불법 주차 | NAVIGATING 시 샛길에 차폭 4.8 m 차량 투입, 동쪽 연결로 대체 경로 | 차량 투입 후 우회 또는 유한 횟수 재시도 후 명시적 실패; 인도 횡단 금지 |
| s18 저마찰 | SDF 접촉 마찰 μ/μ2=0.15, 최종 속도 0.2 m/s, 정상 가감속 0.15 m/s² | 속도 상한, 물리 미끄러짐·정지 거리 측정. 긴급 정지는 램프를 우회하므로 정상 감속과 별도 판정 |
| s19 이동 중 취소 | 1번 NAVIGATING에서 CancelRequest 호출. 중첩 Nav2·수거 액션 종료 확인 후 CANCELLED → SELECTING | 1번 미적재, transit_cancelled 이벤트 후 2번 처리. 정렬·적재 중 취소는 거부 |

주행 실패를 성공으로 판정하지 않는다. 도로 폭·노이즈·동적 장애물 때문에 실제 주행 튜닝이 필요할 수 있다. 군집 첫 대상 수거가 실패하면 뒤 대상 요청은 안전하게 보류되므로 운영자 개입 대상이다.

## 맵·스폰·제어 구조

- `worlds/street_extended.sdf`, `campus_extended.sdf`: 40×30 m, 동일 도로망. 남서쪽 충전 위치와 N1/E1/S1 유지. 캠퍼스는 실제 KNU 90×90 m 모델을 축소한 것이 아니라 요청 치수에 맞춘 별도 단순화 맵이다.
- `maps/*_extended.yaml`: 도로·주차장 밖 점유 처리. `*_extended_lidar.yaml`: 물리 장애물 지도. 밝은 인도는 6 cm 높이 충돌체도 추가한다.
- AMCL은 물리 지도, 전역·지역 costmap은 `/policy_map`을 사용한다. 초기 위치는 선택한 맵의 충전 위치로 설정한다.
- `extended_lidar.py`: Gazebo의 실제 모델 pose에서 레이캐스트하여 AMCL 추정 위치를 다시 센서 정답으로 쓰는 순환을 피한다. kinematic에서는 스폰 위치와 odom을 합성한다. 실제 Gazebo GPU 라이다 대신 기존 모의 라이다를 확장한 것이다.
- `extended_guard.py`: 모든 최종 속도 명령을 한 곳에서 출력. TF·스캔·명령이 오래되면 정지, 회전한 1.7×0.9 m 전체 footprint와 미래 이동 구간을 점유 지도에서 검사한다. 동적 스캔과 속도 제한을 적용한다. 취소 응답 timeout은 정지 상태를 유지하며 재시작 전 자동 해제하지 않는다.
- 초기 배치는 직립·90도 전도·0% 방전(보라색 기둥)·3대 군집·연석 걸침을 포함한다. 대상 중심이 인도에 있어도 로봇은 차도에서 팔로 접근한다. 각 대상에 대해 1.5 m 반경 내 정적 지도상 접근점 존재를 검사했다. 동적 장애물과 팔의 전체 충돌 검증은 별도다.

## 로그와 검증 절차

다른 터미널에서 같은 install 환경을 source한 뒤:

```bash
ros2 topic echo /mission_events
ros2 topic echo /robot_status
ros2 topic echo /admin_alerts
ros2 topic info /cmd_vel --verbose
ros2 bag record -o extended_run /mission_events /robot_status /admin_alerts /queue_status /cmd_vel /scan /tf /tf_static /sim/robot_pose /sim/scooter_truth
```

`/cmd_vel` 발행자는 새 프로필에서 extended_guard 하나여야 한다. 다른 발행자가 있으면 중복 실행을 종료하고 토픽 경로를 확인한다. RViz에서 전역·지역 costmap 모두 인도가 점유되는지 확인한다. Gazebo pose 기준 모든 주행 구간에서 footprint가 인도·외곽에 들어가지 않는지, 동적 모델과 접촉하지 않는지 확인한다. 제한 시간(권장 시뮬레이션 시간 900초) 초과는 미통과로 기록한다. 성공·실패·미실행을 구분하고 rosbag을 보관한다.

```bash
python3 -m pytest src/kickboard_robot/test -q
python3 src/kickboard_sim/scripts/build_extended_env.py
```

## 현재 검증 범위와 한계

Windows에서 Python 3.10 문법 호환 검사, 맵·SDF XML 파싱, 10개 스폰 접근점, 원본 SHA-256 보존, 정책·상태 전이 및 ROS 비의존 기존 테스트를 실행했다. **Gazebo/ROS 프로세스 실행은 WSL E_ACCESSDENIED로 수행하지 못했다. colcon 빌드, Nav2 lifecycle, 물리 주행 성공은 미검증이다.** ROS 모듈이 필요한 기존 위치 추정 테스트 1개는 로컬 실행에서 제외했다.

파손은 선언된 결함 플래그와 색상 표현이며 실제 파단 물리 모델이 아니다. 중량은 시나리오의 선언 질량으로 사전 입장 제어하며 실제 로드셀·적재 동역학이 아니다. 야간 탐색은 모의 태그 가시성 제한이며 렌더링 영상 인식 시험이 아니다. s14는 로봇 GPS/EKF 장애 주입이 아니라 원본 시스템에 존재하는 킥보드 GPS→태그 좌표 보정 시험이다. s16은 정렬 확인 결과 실패 주입이며 도킹 접점 센서 모델은 없다. 마찰 파라미터가 설치된 Harmonic 물리 엔진에서 의도대로 작동하는지는 주행·정지 거리로 확인해야 한다. 이 구분을 유지하여 모의 성공을 실제 센서·안전 성능으로 보고하지 않는다.

참고한 공식 자료: [Humble Nav2 속도 연결](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py), [Humble Velocity Smoother](https://api.nav2.org/nav2-humble/html/md_nav2_velocity_smoother_README.html), [Gazebo Harmonic SDF](https://gazebosim.org/docs/harmonic/sdf_worlds/).
