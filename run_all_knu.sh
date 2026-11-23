#!/bin/bash
# 경북대 캠퍼스 시나리오(k00~k32)를 전부(또는 고른 것) 돌리고, 끝나면 통과 여부 요약 + 운영 리플레이를 만든다.
#
#   ./run_all_knu.sh --gui --rtf 4           전부, Gazebo 화면 + 4배속, 시나리오마다 그 주변만 만든 작은 world
#   ./run_all_knu.sh --gui --rtf 4 k08 k13   고른 것만
#   ./run_all_knu.sh --rtf 4                 화면 없이 (Gazebo 서버만)
#   ./run_all_knu.sh --gui --full            캠퍼스 전체 world 로 (느림)
#   ./run_all_knu.sh --speed 3.0             로봇 최고 속도도 올리기 (기본 1.5 m/s)
#   ./run_all_knu.sh --kinematic -j 4        Gazebo 없이 로직만, 4개 동시에 (가장 빠름)
#
# 결과: results/knu_<날짜_시각>/summary.txt (통과/부분), results/ops_replay.html (리플레이)
# Ctrl+C 로 중단하면 지금까지 결과로 요약·리플레이를 만든다.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/kickboard_sim/scripts/run_knu_scenarios.py "$@"
