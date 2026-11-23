# 노드 파이프라인: 통신부와 가공부 (제안)

> **우선순위: GitHub `main` 의 README 가 최우선이다.** 이 문서와 README 가 다르면 README 를 따른다.
> 이 문서는 팀 합의 전 제안이다. `*_logic.py`·`queue_book.py`·`profiling.py` 는 저장소에 추가만 했고 아직 어느 노드도
> 쓰지 않는다 (README 9장: 각 파일은 담당 역할만 고친다). 각 담당자가 동의하면 자기 노드를 이 구조로 바꾼다.

노드 연결(토픽·서비스·액션)은 단순하게 고정하고, **노드 안에서 받은 데이터를 가공하는 부분**을 최적화한다.
그래서 각 노드를 두 층으로 나눈다.

```
  구독 / 서비스 요청 / 액션 목표          (통신부: <노드>.py)
        │  ROS 메시지 -> 파이썬 값 (tuple, float, dataclass)
        ▼
  가공 함수·클래스                         (가공부: <영역>_logic.py, planner.py 등)
        │  ROS 를 모름. 입력·출력이 파이썬 값이라 단위 테스트·벤치마크가 바로 된다
        ▼
  파이썬 값 -> ROS 메시지
  발행 / 서비스 응답 / 액션 결과·피드백     (통신부)
```

**규칙**
- 가공부(`*_logic.py`, `queue_book.py`, `planner.py`, `motion.py`, `geo.py`, `occupancy.py`, `drop_zones.py`, `queue_logic.py`)는
  `rclpy`·메시지 타입을 import 하지 않는다. numpy·scipy·yaml 은 써도 된다.
- 통신부는 메시지 변환, 파라미터, 타이밍(대기·재시도 루프), 로그만 맡는다. 계산식이 보이면 가공부로 옮긴다.
- 토픽 이름·메시지·노드 이름을 바꾸는 건 "통신부 변경"이다 → 월요일 병합 전에 팀에 미리 알린다 (CONTRIBUTING.md).

## 노드별 입력 → 가공 → 출력

주기는 2026-11-12 Gazebo(헤드리스) 실행에서 잰 값.

| 노드 (통신부) | 입력 | 가공부 | 출력 | 측정 이름 |
|---|---|---|---|---|
| `request_gateway` | `/request_collection` 서비스 | `gateway_logic`: `check_request`, `to_map_checked` (+`geo`) | `/queue/enqueue` 호출 | `gateway.check` |
| `queue_manager` | `/queue/*` 서비스, `/scooter_states` (2 Hz) | `queue_book.QueueBook` (병합·선택·상태 변경·취소·자동 취소, `queue_logic` 의 `score` 사용) | `/queue_status`, `/queue_markers` (1 Hz) | `queue.select`, `queue.auto_cancel`, `queue.publish` |
| `mission_manager` | `/queue_status` (1 Hz), `/battery_pct` (2 Hz), `/scooter_states` | `mission_logic`: `decide_tick`, `decide_select`, `decide_queue_empty`, `classify_failure`, `Cargo`, `ChargerSlots` (+`drop_zones`) | 액션·서비스 호출, `/robot_status` (2 Hz), `/scooter_dropped`, `/admin_alerts` | `mission.tick` |
| `collector` | `/collect_scooter` 액션, `/tag_detections` (10 Hz) | `collector_logic`: `arrived_by_tag`, `center_from_tag`, `mean_center` | `/cmd_vel`, `/scooter_loaded`, GoToPose·LoadScooter 호출 | `collector.center` |
| `dock_controller` | `/dock_to_charger` 액션, `/tag_detections` | `dock_logic`: `charger_poses`, `DockGoalFilter`, `dock_errors`, `within_accept` (+`motion.GoToController`) | `/cmd_vel` | `dock.align_step` |
| `nav_bridge` | `/nav_bridge/go_to_pose` 액션, `/scan` (5 Hz) | `nav_logic`: `filter_scan`, `choose_goal`, `PathFollower` (+`planner.Costmap` A*, `motion`) | `/cmd_vel` (주행 중 20 Hz) | `nav.scan`, `nav.follow_step` |
| `battery_manager` | `/odom` (25 Hz), `/robot_status` | `battery_logic`: `BatteryModel` | `/battery_pct`, `/battery_low` (2 Hz) | `battery.update` |
| `sim_lidar` [시뮬] | `/sim/scooter_truth` (5 Hz), `/scooter_states`, TF | `lidar_logic`: `LidarSim` | `/scan` (5 Hz) | `lidar.scan` |
| `sim_tag_detector` [시뮬] | `/sim/scooter_truth`, TF | `tag_sim_logic`: `detect` | `/tag_detections` (10 Hz) | `tag_sim.detect` |
| `sim_localizer` [시뮬] | `/sim/robot_pose` (25 Hz), `/odom` (25 Hz) | `localizer_logic`: `MapOdomFilter` | TF map→odom (25 Hz) | `localizer.update` |

아직 나누지 않은 노드: `scooter_fleet_sim`, `arm_loader`, `status_panel`, `fake_base`, `apriltag_adapter`
(시뮬·표시용. `arm_loader` 의 역기구학 함수 `fk`·`ik` 는 이미 파일 위쪽에 순수 함수로 있다).

## 측정하는 법

| 무엇을 | 명령 | 언제 |
|---|---|---|
| 가공 함수 1회 비용 (ROS 없이) | `python3 tools/bench_logic.py [이름 일부]` | 가공부를 고친 전후. `--json` 으로 저장해 비교 |
| 노드 안에서의 실제 비용 | `KICKBOARD_PROFILE=1 ros2 launch ...` → 노드 로그의 `[가공 시간]` (노드가 이 구조로 바뀐 뒤부터) | 주기·호출 횟수까지 볼 때 |
| 동작이 그대로인지 | `python3 tools/run_scenarios.py [시나리오...] [--sim kinematic]` | PR 전. 결과는 `runs/<시각>/summary.md` |
| 노드별 CPU | 시뮬 실행 중 `top -c` (Python 노드는 `lib/kickboard_robot/<노드>`) | 고정비를 볼 때 |

`KICKBOARD_PROFILE_SEC=5` 로 보고 주기, `KICKBOARD_PROFILE_DIR=runs/prof` 로 노드별 누적 통계(JSON) 저장.

## 기준값 (2026-11-12, x86 노트북)

**가공 함수** (`tools/bench_logic.py`)

| 항목 | 1회 |
|---|---|
| `nav.plan` (A*, 방 대각선) | 6.0 ms |
| `lidar.scan` (킥보드 7대, 360빔) | 1.3 ms |
| `nav.costmap_update_scan` | 0.54 ms |
| `nav.follower_step` | 0.08 ms |
| `nav.filter_scan` | 27 µs |
| `tag_sim.detect` (7대) | 19 µs |
| `queue.select` / `queue.rows` (50건) | 17 µs / 19 µs |
| `mission`, `battery`, `dock`, `localizer` 판단 1회 | < 1 µs |

**노드별 CPU** (Gazebo 헤드리스, s4 실행 중): 노드마다 3~8% (`arm_loader` 7.9, `mission_manager` 5.9,
`sim_lidar`·`nav_bridge`·`dock_controller` 5.3, … `request_gateway` 2.9). 거의 일을 안 하는 노드도 3% 안팎이다.

**토픽 주기**: `/clock` 100 Hz, `/joint_states` 100 Hz, `/tf` 45 Hz, `/odom`·`/sim/robot_pose` 25 Hz,
`/tag_detections` 10 Hz, `/scan`·`/sim/scooter_truth` 5 Hz, `/robot_status`·`/battery_pct`·`/scooter_states` 2 Hz.

## 최적화 후보 (근거 있는 것부터)

1. **노드 공통 고정비**: Python 노드 15개가 모두 `/clock`(100 Hz)을 받고, 8개 노드가 각자 TF 리스너로 `/tf`(45 Hz)를
   받는다. 계산은 거의 없는데 노드당 3% 안팎을 쓰는 이유. → 위치가 필요한 노드만 TF 를 받게 하거나, TF 조회 주기를 낮추기.
2. **A* 재계획**: `PathFollower` 가 1초마다 6 ms 짜리 A* 를 다시 돈다. → 경로가 막혔거나 목표가 바뀔 때만 다시 계획.
3. **`sim_lidar`**: 매 주기 정적 격자를 복사하고 킥보드 상자를 다시 그린다 (1.3 ms × 5 Hz). → 킥보드가 안 움직였으면 재사용.
4. **`arm_loader`**: `/joint_states`(100 Hz)를 팔이 안 움직일 때도 처리한다. → 동작 중에만 처리.
5. **판단 품질** (2026-11-12 시나리오 결과):
   - s5: 구석(3.5, -3.0)의 킥보드에 접근할 빈 자리를 못 찾아 3회 실패 → NEEDS_HUMAN.
     태그만 보이면 되니 "카메라 거리(3 m) 안 아무 빈 자리"로 접근하는 방법이 후보.
   - s7: 시나리오로 넣은 장애물을 `sim_lidar` 가 모른다 (정적 지도 + 킥보드만 그림) → 회피를 검증할 수 없음.
   - s4: 적재 중에는 배터리 복귀를 미루므로 "이동 중 대상 반납" 경로가 실행되지 않을 수 있다.
