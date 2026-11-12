## 무엇을 바꿨나
<!-- 한두 줄. 어느 노드의 어떤 가공을 바꿨는지 -->

## 바꾼 범위
- [ ] 가공부만 (`*_logic.py`, `planner.py` 등 — 토픽·메시지·노드 이름 그대로)
- [ ] 통신부 (토픽·서비스·액션 추가/이름 변경, `kickboard_interfaces` 수정) → **전원 리뷰 필요**
- [ ] 시뮬 환경 (world, 맵, 시나리오, launch)

## 확인한 것
- [ ] `cd src/kickboard_robot && python3 -m pytest -q test/` 통과
- [ ] 가공부를 바꿨으면 `python3 tools/bench_logic.py` 전후 수치 (아래에 붙이기)
- [ ] 관련 시나리오 `python3 tools/run_scenarios.py <시나리오> [--sim kinematic]` 통과

## 전후 비교
<!-- bench_logic, run_scenarios summary.md, KICKBOARD_PROFILE 로그 중 해당하는 것 -->
