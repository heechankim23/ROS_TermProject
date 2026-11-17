# 통합 현황 (2026-11-17, 병합 관리)

main 에 role_A · role_B · role_C 를 합친 결과와, 아직 브랜치에 올라오지 않은 파일 목록이다.
role_D 는 main 과 같고 변경이 없었다.

## 1. 이번에 main 에 합친 것

| 순서 | 브랜치 | 내용 | 병합 시 처리 |
|---|---|---|---|
| 0 | (공통) | `.gitattributes` 추가: 줄바꿈을 LF 로 고정 | Windows 에서 파일 전체가 CRLF 로 바뀌는 문제 방지 |
| 1 | role_B | 요청 임계값 45°/15% 확정, 가중치 10/1/5 유지 확정(실험 결과 README 10장), t_max 보장 범위 정정, RViz 대기열 마커 2줄(순위·대기 시간) | 충돌 없음 |
| 2 | role_C | `nav_bridge.py` 주행 안전 처리(동시 goal 거절, 센서 타임아웃, 미측정 빔 처리, 직선 offset 대체 제거, 정지 보장), `planner.py`(floor 좌표, 대각선 모서리 통과 금지, 미측정 빔이 장애물을 지우지 않음), `test_navigation.py` 17개, 타임아웃 파라미터 3개 | role_C 의 CRLF 변환(`0105a1e`)을 LF 로 정규화해서 합침. 실제 변경 5개 파일만 반영 |
| 2-1 | role_C | `motion.py` 의 테스트용 `"test2"` 줄 제거 (`8c746cd`, `358ea2a`) | 별도 커밋 |
| 3 | role_A | `scripts/build_env.py`(도로 맵 생성기), `scripts/check_env.py`(사전 점검), `GAZEBO_COMMANDS.md`, `docs/SIM_GUIDE.html` | 새 파일만 추가. 충돌 없음 |

## 2. 병합본 검증 결과

ROS 가 없는 환경에서 확인할 수 있는 것만 돌렸다. **Gazebo/ROS 실행 확인은 각 담당자가 Ubuntu 에서 해야 한다.**

| 항목 | 결과 |
|---|---|
| 전체 `.py` 문법 검사 (`py_compile`) | 통과 |
| `test/test_logic.py` | 8/9 통과. 실패 1개(`test_sim_localizer_map_odom`)는 `rclpy` 가 없는 환경 문제 |
| `test/test_navigation.py` 중 planner 테스트 4개 | 4/4 통과. 나머지 13개는 ROS 필요 (미실행) |
| `robot_params.yaml` 파싱 | 통과 (B 의 가중치 주석 + C 의 타임아웃 파라미터 모두 반영) |
| `build_env.py street` | 통과. world · 지도 2장 · 맵 설정 · 배치도 생성 |
| `check_env.py` | **실패**. C 담당 패치(`Costmap.set_physical`, `free_spot(keep_clearance=)`)가 아직 없음 → 3장 참고 |

## 3. 아직 브랜치에 없는 파일 (담당자별 요청 목록)

`GAZEBO_COMMANDS.md`, `docs/SIM_GUIDE.html` 에 적혀 있지만 어떤 브랜치에도 없는 것들이다.
**이것들이 들어와야 `GAZEBO_COMMANDS.md` 의 명령(s10~s19 등)이 그대로 동작한다.**

### role_A (haein0248)

- [ ] 시나리오 `s10_road_collect` ~ `s19_peak_stream` (10개)
- [ ] 시나리오 `s1` ~ `s9` 도로판 (좌표·배터리 초기값·s5 구성 변경. 지금 main 에는 실내판만 있음)
- [ ] `run_gazebo.sh`
- [ ] launch 수정, `map_utils.py`
- [ ] `scooter_fleet_sim.py`, `gz_utils.py`, `fake_base.py` 수정 (새 토픽 `/sim/obstacles` 포함)
- [ ] 도로 맵 생성물: `worlds/street.sdf`, `maps/street*.pgm|yaml`, `maps/street_layout.*`, `config/maps/street.yaml` (`build_env.py` 로 생성 가능하지만 launch 가 참조하므로 커밋 필요)
- [ ] `src/kickboard_sim/archive/` (실내 시나리오 17개, 캠퍼스 시나리오 6개, 캠퍼스 맵)
- [ ] 다른 담당자 파일 패치 4개 (`my_ws/patches_for_team/`) → 아래 각 담당자에게 전달하거나 해당 브랜치로 PR

### role_C (monn098) — A 가 만든 패치 반영

- [ ] `C_nav_lidar.patch`: 인도 진입 금지(`set_physical`, `keep_clearance`), 도킹 방향 맞춤, 스캔 시각 보정, 가까운 빈 자리, 장애물 스캔 반영
  - 이미 main 에 들어간 C 의 `nav_bridge.py` · `planner.py` 변경과 **같은 파일**이므로, 최신 main 위에서 합쳐야 한다
  - 이것이 들어와야 `check_env.py` 가 동작한다
- [ ] `motion.py`, `sim_lidar.py`, `common.py` 수정 (A 문서 기준)

### role_D — A 가 만든 패치 반영

- [ ] `D_collector_mission.patch`: `collector.py` · `mission_manager.py` (팔 도달 거리 확인, 상태 되돌아감 수정)

### role_B (Lamelo0721)

- [ ] 가중치 실험에 쓴 **s5 변형 시나리오 파일** (README 10장 "s5 를 바꾼 실험용 배치". 재현용)
- [ ] A 가 만든 `B_queue_manager.patch`(보고 끊긴 대기 항목 자동 취소) 반영 여부 결정
  - `queue_manager.py` 는 이번에 B 의 마커 변경이 들어갔으므로 최신 main 위에서 합쳐야 한다
- [ ] `B_README_additions.patch` 는 A 문서상 "옛 내용"이라 반영 불필요

### 출처 확인 필요 (팀 전체)

- [ ] 11/16 22:31 A 의 워크스페이스에 추가된 **"확장 프로필"**: `extended.launch.py`, `extended_*.py` 노드 8개, `config/scenarios_extended/`(별도 s10~s19), `street_extended` · `campus_extended` 맵, 테스트 `test_legacy_manifest_hashes`
  - 누가 만들었는지, A 의 s10~s19 와 어느 쪽을 기준으로 할지 정해야 한다 (이름이 겹침)

## 4. 결정이 필요한 것

1. **C: 대기 시간 기준.** `nav_bridge.py` 가 `sleep_sim`(시뮬 시간) 대신 `time.sleep`(실제 시간)을 쓴다. 시뮬을 느리게/일시정지하면 대기·타임아웃이 시뮬 시간과 어긋난다. 센서 타임아웃은 의도(파라미터 주석)로 보이지만 주행 루프 주기까지 실제 시간인 것이 맞는지 확인.
2. **B·C: s5 의 `no path`.** B 의 가중치 실험에서 4번 (−3.0, −2.6), 원래 s5 1번 (3.5, −3.0) 까지 경로가 없었다. C 의 대각선 통과 금지가 들어간 지금 main 에서 다시 확인.
3. **A · 확장 프로필 중 기준 결정** (3장 마지막 항목).

## 5. 앞으로의 작업 규칙

1. **작업 시작 전에 최신 main 을 받는다.** `git pull origin main` (자기 role 브랜치에서)
2. **하루 이상 PC 에만 두지 않는다.** 작업 단위가 끝나면 커밋하고 push.
3. **main 에는 PR 로만 합친다.** 자기 role 브랜치 → main 으로 PR, 병합 관리자가 확인 후 병합. 주 1~2회 이상.
4. **다른 담당자 파일은 직접 고치지 않는다.** 필요하면 담당자에게 요청하거나, 그 담당자 브랜치로 PR.
5. **커밋 전에 `git status`, `git diff` 로 확인한다.** 줄바꿈 변경, 테스트용 코드가 섞이지 않게.
6. Windows 사용자는 한 번 설정: `git config --global core.autocrlf input`
