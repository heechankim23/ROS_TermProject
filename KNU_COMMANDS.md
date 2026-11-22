# 경북대 캠퍼스 킥보드 수거 로봇 — 실행 명령 정리

모든 명령은 워크스페이스 `~/lecture_2026_2/Ros_TermProject/my_ws` 에서 실행한다.

## 1. 준비 (터미널을 열 때마다)

```bash
cd ~/lecture_2026_2/Ros_TermProject/my_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

코드를 받거나 고쳤을 때만 빌드한다 (빌드 뒤 `source install/setup.bash` 다시):

```bash
colcon build --packages-select kickboard_interfaces kickboard_robot kickboard_sim
```

## 2. 시나리오 하나 Gazebo 로 보기

`k08` 자리에 `k00` ~ `k32` 중 하나.

```bash
ros2 launch kickboard_sim bringup.launch.py scenario:=k08 map:=local rtf:=4 start_delay:=15
```

| 옵션 | 뜻 |
|---|---|
| `map:=local` | 그 시나리오 주변만 잘라 만든 작은 world (가볍고 이동 거리가 짧음). 빼면 캠퍼스 전체 world (무거움) |
| `rtf:=4` | 4배속. 로봇이 버벅이거나 경로를 벗어나면 2~3 으로 낮춘다 |
| `rviz:=true` | RViz 도 함께 (킥보드 상태 색, 로봇 경로, 목적지 화살표, 구역, 정렬 완료 칸) |
| `gui:=false` | Gazebo 창 없이 서버만 |
| `sim:=kinematic` | Gazebo 없이 로직만 (배속 대신 로봇 속도로 조절) |
| `map:=knu_campus` | 언덕 지형이 있는 캠퍼스 전체 world |

로봇이 도크로 돌아와 대기(IDLE)하면 끝난 것이다. `Ctrl+C` 로 끄고 다음 시나리오를 실행한다.

## 3. 여러 시나리오 자동으로 돌리기

하나가 끝나면 알아서 다음으로 넘어가고, 전부 끝나면 통과 여부 요약과 리플레이를 만든다. 중간에 `Ctrl+C` 로 멈춰도 그때까지 결과로 만든다.

```bash
./run_all_knu.sh --gui --rtf 4                     # 전부, Gazebo 화면 + 4배속
./run_all_knu.sh --gui --rtf 4 k01 k08 k13 k29     # 고른 것만
./run_all_knu.sh --rtf 4                           # 화면 없이 (더 가벼움)
./run_all_knu.sh --kinematic -j 4 --speed 3.0      # Gazebo 없이 로직만, 4개 동시 (가장 빠름)
```

| 옵션 | 뜻 |
|---|---|
| `--gui` | Gazebo 화면 띄우기 (한 번에 하나씩) |
| `--rtf N` | Gazebo 배속 |
| `--speed V` | 로봇 최고 속도 m/s (기본 1.5) |
| `--full` | 작은 world 대신 캠퍼스 전체 world |
| `--kinematic -j N` | Gazebo 없이 N 개 동시에 |
| `--grace S` | 로봇이 대기로 돌아온 뒤 S 초 동안 새 일이 없으면 끝 (기본 200) |
| `--limit S` | 시나리오당 최대 S 초 (기본 3600) |

결과: `results/knu_<날짜_시각>/summary.txt` (시나리오별 통과/부분), `results/ops_replay.html` (리플레이)

## 4. 실행 중 상황 보기 (새 터미널, 1번 먼저)

```bash
ros2 topic echo /ops_status        # 지금 하는 일 · 어디로 · 왜 · 목적지 · 정렬 결과
ros2 topic echo /zone_occupancy    # 거치대 슬롯에 서 있는 킥보드
ros2 topic echo /admin_alerts      # 관리자 알림 (충전 대기, 스탠드 고장, 수거 불가 …)
ros2 topic echo /robot_status      # 로봇 상태 · 배터리 · 적재 수
```

Gazebo 오른쪽 아래 "Kickboard status" 패널에서 Echo 를 누르면 `[지금]` `[이유]` `[적재]` `[결과]` 줄이 나온다.

## 5. 결과 확인과 리플레이

`run_all_knu.sh` 를 쓰면 자동. 2번처럼 하나씩 돌렸을 때:

```bash
python3 src/kickboard_sim/scripts/check_knu_runs.py      # 시나리오별 통과/부분 (예상 결과와 비교)
python3 src/kickboard_sim/scripts/build_ops_replay.py    # 리플레이 만들기 -> results/ops_replay.html
xdg-open results/ops_replay.html                         # 브라우저로 열기 (재생 속도 ×1~×60)
```

실행 기록은 `~/.ros/kickboard_ops/<시나리오>_<시각>.jsonl` 에 쌓이고, 리플레이에는 시나리오별 최신 기록이 들어간다.

## 6. 시나리오·맵을 고쳤을 때

```bash
python3 src/kickboard_sim/scripts/build_knu_scenarios.py     # 시나리오 다시 만들기 (좌표·로봇 접근·조건 검사)
python3 src/kickboard_sim/scripts/build_scenario_worlds.py   # 시나리오 전용 작은 world 다시 만들기
python3 src/kickboard_sim/scripts/build_knu_campus.py        # 캠퍼스 맵 전체 다시 만들기 (약 40초)
```

그다음 1번의 `colcon build` 와 `source install/setup.bash`.

## 7. 시나리오 목록

| 이름 | 내용 | 이름 | 내용 |
|---|---|---|---|
| k00 | 캠퍼스 운영 전경 (킥보드 56대) | k17 | 적재함 안 목적지가 다른 4대 |
| k01 | 도서관 앞 무질서 방치 | k18 | 가까운 일반 구역 만석 |
| k02 | 건물 출입구 앞 불법 주차 | k19 | 고장 분류 칸 부족 (고장 5대) |
| k03 | 통행 방해 신고 접수 | k20 | 가는 도중 누가 빌려 감 |
| k04 | 넘어진 킥보드 | k21 | 대기 중 사용자가 직접 정렬 |
| k05 | 정렬된 킥보드는 그대로 | k22 | 충전 스탠드 고장 |
| k06 | 배터리 부족 → 충전구역 | k23 | 충전구역 일시 폐쇄 (광장 행사) |
| k07 | 고장 → 고장 분류 칸 | k24 | 짐 실은 채 로봇 배터리 부족 |
| k08 | 한 장소에 상태 다른 7대 | k25 | 언덕길 왕복 (지형 맵) |
| k09 | 넘어짐 + 배터리 부족 | k26 | 계단 위 킥보드 (접근 불가) |
| k10 | 고장 + 배터리 부족 | k27 | GPS 오차 큰 곳의 신고 |
| k11 | 방치 신고 → 실제 고장 | k28 | 보행자 혼잡 |
| k12 | 오신고 | k29 | 수거 중 로봇팔 고장 |
| k13 | 충전구역 만석 (최대 5대) | k30 | 세워 둔 킥보드가 다시 넘어짐 |
| k14 | 캠퍼스 전체 충전구역 포화 | k31 | 야간 일괄 순회 |
| k15 | 적재량 초과 (한 곳에 7대) | k32 | 하루 운영 종합 (발표 데모용) |
| k16 | 요청 폭주 25건 | | |

시나리오별 상황·예상 결과·통과 기준: `src/kickboard_sim/config/scenarios_knu/README.md`

## 8. 문제가 생기면

- Gazebo 창이 안 뜨거나 바로 꺼짐: `gui:=false` 로 서버만 돌리거나 `sim:=kinematic` 으로 로직만 확인
- 로봇이 안 생김: `start_delay:=20` 처럼 늘린다 (world 를 불러오는 데 시간이 걸림)
- 로봇이 경로를 벗어나거나 멈칫거림: `rtf` 를 낮춘다
