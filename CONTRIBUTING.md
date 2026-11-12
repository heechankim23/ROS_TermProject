# 협업 규칙 (보조 문서)

> **우선순위: GitHub `main` 의 [README.md](README.md) 가 최우선이다.** 이 문서는 README 를 보조하며, 서로 다르면 README 를 따른다.
> README 가 바뀌면 이 문서를 README 에 맞춰 고친다.

GitHub 로 코드를 주고받으며 각자 맡은 노드의 **가공부**를 개선한다. 노드 연결(토픽·서비스·액션)은 단순하게 유지한다.
구조 설명은 [docs/PIPELINE.md](docs/PIPELINE.md) (팀 합의 전 제안. `*_logic.py` 등은 추가만 했고 아직 노드가 쓰지 않는다).

## 1. 역할별 담당 파일

**역할 분담·다음 할 일·공통 규칙의 기준은 [README 9장](README.md)** 이다 (2026-11-12 조정). 아래 표는 그 분담에
이번에 생긴 가공부 파일(`*_logic.py`)을 붙인 것. `src/kickboard_robot/kickboard_robot/` 기준, 굵은 글씨가 주로 고칠 가공부.

| 역할 | 통신부 (되도록 그대로) | 가공부 (최적화 대상) | 그 밖 |
|---|---|---|---|
| A. 시뮬 환경·시나리오·데모 영상 | `scooter_fleet_sim.py`, `arm_loader.py`, `status_panel.py`, `sim_tag_detector.py`, `fake_base.py`, `apriltag_adapter.py` | **`tag_sim_logic.py`**, `gz_utils.py` | `src/kickboard_sim/` (world, 맵, launch, 시나리오) |
| B. 요청·대기열 | `request_gateway.py`, `queue_manager.py` | **`gateway_logic.py`, `queue_book.py`**, `queue_logic.py`, `geo.py` | README |
| C. 주행·도킹 | `nav_bridge.py`, `dock_controller.py`, `sim_lidar.py`, `sim_localizer.py` | **`nav_logic.py`, `planner.py`, `motion.py`, `dock_logic.py`, `lidar_logic.py`, `localizer_logic.py`** | `config/nav2_params.yaml` |
| D. 미션 | `mission_manager.py`, `collector.py`, `battery_manager.py` | **`mission_logic.py`, `collector_logic.py`, `battery_logic.py`**, `drop_zones.py` | |
| E. 기술 리드 (인터페이스·통합 검증·일정) | `src/kickboard_interfaces/` | `analyze.py` | `tools/`, `test/`, `.github/` (CI), PRD |

README 9장 공통 규칙 중 이 구조와 직접 관련된 것:
- `mission_manager.py`(+`mission_logic.py`)는 D만 고친다. 다른 사람은 D에게 요청한다.
- msg·srv·action 변경은 E를 거친다. E는 회귀 테스트에서 찾은 문제를 담당자에게 알리고, 남의 코드를 직접 고치지 않는다.
- `/cmd_vel` 은 nav_bridge·dock_controller(C)와 collector(D)가 함께 쓴다. 동시에 보내지 않는다.
- 코드를 공유(push)하기 전에 각자 s1 을 돌려 통과를 확인한다 (push 는 주 마무리·작업 완료 때): `python3 tools/run_scenarios.py s1_single`

## 2. 고칠 때 지킬 것

- **가공부**에는 `rclpy`·메시지 타입을 import 하지 않는다. 입력·출력은 파이썬 값(float, tuple, list, dataclass).
  그래야 ROS 없이 테스트(`pytest`)와 속도 측정(`tools/bench_logic.py`)이 된다.
- **통신부 변경** = 토픽·서비스·액션·노드 이름 추가/변경, `kickboard_interfaces` 메시지 수정, 발행 주기 변경.
  다른 역할의 노드가 깨질 수 있으니 PR 에 표시하고 **전원 리뷰** 후 병합한다.
- 가공부 함수의 입력·출력 형태를 바꾸면 그 함수를 부르는 통신부도 같은 PR 에서 고친다.
- 새 파라미터는 `src/kickboard_sim/config/robot_params.yaml` 에 기본값과 한 줄 설명을 같이 넣는다.

## 3. 브랜치와 병합 (팀 Git 사용 설명서 확정본 기준)

- 저장소: `https://github.com/heechankim23/ROS_TermProject.git`. 각자 자기 브랜치 `role_A` ~ `role_E` 에서만 작업한다.
- **commit 은 작업한 날마다** (`git add .` → `git commit -m "한 일"`): 내 컴퓨터에 그날 수정 기록을 저장.
- **push 는 한 주가 마무리되거나 특정 작업을 끝냈을 때** (`git push`): GitHub 내 브랜치에 올림. main 에는 영향 없음.
- `main` 은 관리자(role_E)만 합친다. **매주 월요일 밤 12시**까지 각자 push 하고 멈춤 → role_E 가 모든 브랜치를 main 에 합침
  (이 동안 커밋하지 않음) → 정리 완료 공지 후 각자 `git pull origin main` 으로 받아서 다음 주 작업 시작.
- 커밋 메시지에는 그날 무엇을 바꿨는지 적는다 (예: `D: mission_logic 재시도 제한 추가`).
  가공부 최적화는 메시지나 주간 정리 때 전후 수치(bench_logic, run_scenarios)를 같이 남긴다.
- `git add .` 은 폴더 전체를 담으므로, `build/`·`install/`·`log/`·`bags/`·`runs/` 는 `.gitignore` 로 빠지는지 커밋 전에 `git status` 로 확인한다.
- **통신부 변경**(토픽·메시지·노드 이름)은 월요일 병합 전에 팀에 미리 알린다 — 다른 역할 브랜치와 충돌하기 쉽다.
- PR 을 쓰게 되면 `.github/pull_request_template.md` 양식이 자동으로 붙는다 (선택).

## 4. push 전에 돌릴 것

```bash
colcon build --symlink-install && source install/setup.bash
cd src/kickboard_robot && python3 -m pytest -q test/ && cd -
python3 tools/bench_logic.py                      # 가공부를 고쳤으면 전후 비교
python3 tools/run_scenarios.py s1_single s6_verify_fail --sim kinematic   # 빠른 확인
python3 tools/run_scenarios.py                    # 전체 (Gazebo, 약 40분)
```

GitHub 에서는 어느 브랜치든 push 하면 빌드 → 단위 테스트 → 벤치마크 → kinematic 시나리오 3개(s1, s6, s8)가
자동으로 돈다 (`.github/workflows/ci.yml`, 저장소 Actions 탭에서 결과 확인). role_E 는 병합 전에 각 role 브랜치의
결과가 초록색인지 본다. Gazebo·Nav2 시나리오는 CI 에서 돌리지 않으니 각자 PC 에서 확인한다.

## 5. 환경 메모

- 새 터미널마다 ROS Humble 과 워크스페이스를 source 한다: `source /opt/ros/humble/setup.bash && source install/setup.bash`
- README 2장대로 **코드를 고친 뒤에는 항상 다시 `colcon build`** 한다.
- NVIDIA GPU 노트북(PRIME on-demand)에서 Gazebo 창을 GPU 로 그리려면, 같은 터미널에서 먼저:
  ```bash
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
  ```
  (설정하지 않으면 Mesa 소프트웨어 렌더러, 즉 CPU 로 그린다)
- 시뮬레이션은 한 PC 에서 하나만 돌린다. `tools/run_scenarios.py` 는 `ROS_DOMAIN_ID=42` 를 따로 써서 다른 시뮬과 섞이지 않는다.
