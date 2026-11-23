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

## 2. 시나리오 시뮬레이션 (Gazebo 화면 + 4배속, 권장)

시나리오마다 그 주변만 잘라 만든 작은 world 에서 4배속으로 돌린다. 하나가 끝나면(로봇이 도크로 돌아와 대기) 알아서 다음으로 넘어가고,
다 끝나면 통과 여부 요약(`results/knu_<날짜_시각>/summary.txt`)과 리플레이(`results/ops_replay.html`)를 만든다. 중간에 `Ctrl+C` 로 멈춰도 그때까지 결과로 만든다.

### 발표 데모 (추천 3개)

```bash
./run_all_knu.sh --gui --rtf 4 k32 k08 k13
```

- k32 하루 운영 종합: 우선순위 · 목적지 분류 · 적재량 · 자동 취소 · 충전 스탠드 고장 대응을 한 번에
- k08 한 장소에 상태가 다른 7대: 정렬된 2대는 두고 5대를 일반 / 충전 / 고장 칸으로 나눠 보냄
- k13 충전 구역 만석: C1 에 1대, 나머지는 다음으로 가까운 빈 충전 구역으로

### 분류별로 한 번에

```bash
./run_all_knu.sh --gui --rtf 4 k01 k02 k03 k04 k05 k06 k07 k08        # A 상태별 기본
./run_all_knu.sh --gui --rtf 4 k09 k10 k11 k12                        # B 복합 판단
./run_all_knu.sh --gui --rtf 4 k13 k14 k15 k16 k17 k18 k19            # C 수용량·적재량
./run_all_knu.sh --gui --rtf 4 k20 k21 k22 k23 k24 k25 k26 k27 k28 k29 k30 k31 k32  # D 돌발상황
./run_all_knu.sh --gui --rtf 4 k00                                    # E 운영 환경
./run_all_knu.sh --gui --rtf 4                                        # 33개 전부 (k00~k32)
```

### 시나리오 하나씩

| 시나리오 | 내용 | 명령 |
|---|---|---|
| k00 | 캠퍼스 운영 전경 (킥보드 56대) | `./run_all_knu.sh --gui --rtf 4 k00` |
| k01 | 도서관 앞에 무질서하게 놓인 킥보드 | `./run_all_knu.sh --gui --rtf 4 k01` |
| k02 | 건물 출입구 앞 불법 주차 | `./run_all_knu.sh --gui --rtf 4 k02` |
| k03 | 통행 방해 신고 접수 | `./run_all_knu.sh --gui --rtf 4 k03` |
| k04 | 넘어진 킥보드 | `./run_all_knu.sh --gui --rtf 4 k04` |
| k05 | 이미 바르게 정렬된 킥보드는 건드리지 않음 | `./run_all_knu.sh --gui --rtf 4 k05` |
| k06 | 배터리 부족 킥보드를 충전 주차구역으로 | `./run_all_knu.sh --gui --rtf 4 k06` |
| k07 | 고장 킥보드 분류 | `./run_all_knu.sh --gui --rtf 4 k07` |
| k08 | 한 장소에 상태가 다른 킥보드 7대 | `./run_all_knu.sh --gui --rtf 4 k08` |
| k09 | 넘어졌고 배터리도 부족 | `./run_all_knu.sh --gui --rtf 4 k09` |
| k10 | 고장 났고 배터리도 부족 | `./run_all_knu.sh --gui --rtf 4 k10` |
| k11 | 방치 신고 → 확인해 보니 고장 | `./run_all_knu.sh --gui --rtf 4 k11` |
| k12 | 오신고 (이미 바르게 주차됨) | `./run_all_knu.sh --gui --rtf 4 k12` |
| k13 | 충전 주차구역 수용량 초과 (최대 5대) | `./run_all_knu.sh --gui --rtf 4 k13` |
| k14 | 캠퍼스 전체 충전 주차구역 포화 | `./run_all_knu.sh --gui --rtf 4 k14` |
| k15 | 로봇 적재량 초과 (한 곳에 7대) | `./run_all_knu.sh --gui --rtf 4 k15` |
| k16 | 요청 폭주 (시험 끝난 날 25건) | `./run_all_knu.sh --gui --rtf 4 --limit 5400 k16` |
| k17 | 적재함 안에 목적지가 다른 4대 | `./run_all_knu.sh --gui --rtf 4 k17` |
| k18 | 가장 가까운 일반 주차구역이 만석 | `./run_all_knu.sh --gui --rtf 4 k18` |
| k19 | 고장 분류 칸 부족 (고장 5대) | `./run_all_knu.sh --gui --rtf 4 k19` |
| k20 | 가는 도중 누가 빌려 감 | `./run_all_knu.sh --gui --rtf 4 k20` |
| k21 | 대기 중 사용자가 직접 정렬 | `./run_all_knu.sh --gui --rtf 4 k21` |
| k22 | 꽂으려던 충전 스탠드 고장 | `./run_all_knu.sh --gui --rtf 4 k22` |
| k23 | 충전구역 일시 폐쇄 (광장 행사) | `./run_all_knu.sh --gui --rtf 4 k23` |
| k24 | 짐을 실은 채 로봇 배터리 부족 | `./run_all_knu.sh --gui --rtf 4 k24` |
| k25 | 언덕길 왕복 (경사 구간) | `./run_all_knu.sh --gui --rtf 4 k25` |
| k26 | 계단 위 킥보드 (로봇 접근 불가) | `./run_all_knu.sh --gui --rtf 4 k26` |
| k27 | GPS 오차가 큰 곳의 신고 | `./run_all_knu.sh --gui --rtf 4 k27` |
| k28 | 수업 교대 시간 보행자 혼잡 | `./run_all_knu.sh --gui --rtf 4 k28` |
| k29 | 수거 중 로봇팔 고장 | `./run_all_knu.sh --gui --rtf 4 k29` |
| k30 | 세워 둔 킥보드가 다시 넘어짐 | `./run_all_knu.sh --gui --rtf 4 k30` |
| k31 | 야간 일괄 순회 (배터리 부족 8 + 고장 2) | `./run_all_knu.sh --gui --rtf 4 k31` |
| k32 | 하루 운영 종합 (발표 데모용) | `./run_all_knu.sh --gui --rtf 4 k32` |

| 옵션 | 뜻 |
|---|---|
| `--gui` | Gazebo 화면을 띄움 (한 번에 하나씩) |
| `--rtf 4` | 4배속. 로봇이 버벅이거나 경로를 벗어나면 `--rtf 3` 으로 낮춘다 |
| `--full` | 작은 world 대신 캠퍼스 전체 world (무거움) |
| `--grace S` | 로봇이 대기로 돌아온 뒤 S 초 동안 새 일이 없으면 다음으로 (기본 200). 빨리 넘기려면 `--grace 60` |
| `--limit S` | 시나리오당 최대 S 초 (기본 3600). k16 처럼 큰 시나리오는 `--limit 5400` |

- Gazebo 카메라는 로봇을 따라간다. 고정 화면으로 보려면 실행 중 새 터미널에서
  `gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: ""'`
  (시나리오가 바뀌면 다시 따라가므로 그때마다 다시 실행)

## 3. 그 밖의 실행 방법

```bash
./run_all_knu.sh --kinematic -j 4 --speed 3.0          # Gazebo 없이 로직만, 4개 동시에 (가장 빠른 검증)
./run_all_knu.sh --rtf 4                               # Gazebo 화면 없이 서버만
```

`ros2 launch` 로 직접 하나만 띄우기 (끝나도 자동으로 꺼지지 않음, `Ctrl+C` 로 끔):

```bash
ros2 launch kickboard_sim bringup.launch.py scenario:=k08 map:=local rtf:=4 start_delay:=15
```

| 옵션 | 뜻 |
|---|---|
| `map:=local` | 그 시나리오 주변만 잘라 만든 작은 world. 빼면 캠퍼스 전체 world |
| `rtf:=4` | 4배속 |
| `follow:=false` | 카메라가 로봇을 따라가지 않음 |
| `rviz:=true` | RViz 도 함께 (킥보드 상태 색, 경로, 목적지 화살표, 구역) |
| `start_delay:=15` | world 를 불러온 뒤 시작까지 기다리는 초. 로봇이 안 생기면 20 |

이렇게 하나씩 돌렸을 때 결과·리플레이 만들기는 5번.

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
