# 경북대 캠퍼스 킥보드 수거 로봇 — 결과물

아래 표의 링크를 누르면 브라우저에서 바로 열린다. 처음 열 때 "One more step" 화면이 나오면 **Open the page** 를 누른다.

| 결과물 (눌러서 열기) | 내용 |
|---|---|
| [캠퍼스 지도 — campus_map_viewer.html](https://raw.githack.com/heechankim23/ROS_TermProject/role_A/docs/knu_results/campus_map_viewer.html) | 2D · 3D, 일반 주차구역 · 충전 주차구역 C1~C8 |
| [시나리오 목록 — scenario_catalog.html](https://raw.githack.com/heechankim23/ROS_TermProject/role_A/docs/knu_results/scenario_catalog.html) | 33개 시나리오의 상황 · 임무 · 예상 결과 |
| [운영 리플레이 — ops_replay.html](https://raw.githack.com/heechankim23/ROS_TermProject/role_A/docs/knu_results/ops_replay.html) | 시나리오별 로봇 이동 · 이유 · 목적지 · 정렬 결과 (11 MB, 여는 데 5~10초) |
| [시나리오 검증 결과 — scenario_results.txt](scenario_results.txt) | 시나리오별 통과 / 부분 |

깃허브 파일 목록에서 html 파일을 직접 누르면 코드로만 보이고, `ops_replay.html` 은 크기가 커서
"can't show files that are this big" 로 나온다. **위 표의 링크로 열면 된다.**
내려받아 보려면: 위 링크로 연 페이지에서 Ctrl+S 로 저장한 뒤 더블클릭.

## 검증 결과 요약

- 33개 시나리오 중 **32개 통과** (예상 처리와 실제 처리 비교, `check_knu_runs.py`)
- k16 (요청 폭주 25건): 25건 중 23건 처리
  - #17: 킥보드를 싣고 충전 스탠드로 가던 중 실행 시간 제한(55분) 도달
  - #21: 킥보드 앞 공간이 좁아 정렬 위치에 서지 못함 → 정렬 시간 초과로 관리자 알림 후 다음 요청 처리
- 실행 방법: 저장소 맨 위 [KNU_COMMANDS.md](../../KNU_COMMANDS.md)
