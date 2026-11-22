#!/bin/bash
# Gazebo 로 시나리오 하나를 실행한다.
#   ./run_gazebo.sh                      시나리오 목록
#   ./run_gazebo.sh st1_road_collect     Gazebo 화면 + 시나리오
#   ./run_gazebo.sh s3_full_load rviz:=true          추가 인자는 그대로 launch 에 넘긴다
#   ./run_gazebo.sh s1_single gui:=false sensors:=false   화면 없이 (SSH 등)
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
if [ ! -f install/setup.bash ] || [ -n "$(find src -newer install/setup.bash -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.sdf' -o -name '*.pgm' \) -print -quit)" ]; then
  echo "[run_gazebo] 소스가 바뀌어 다시 빌드합니다"
  colcon build || exit 1
fi
source install/setup.bash
SC_DIR=src/kickboard_sim/config/scenarios
if [ -z "$1" ]; then
  echo "사용법: ./run_gazebo.sh <시나리오> [launch 인자...]"
  for f in $(ls $SC_DIR/*.yaml | sort -V); do
    printf "  %-26s %s\n" "$(basename $f .yaml)" "$(grep -m1 '^name:' $f | sed 's/name: *//; s/"//g')"
  done
  exit 0
fi
SC=$1; shift
[ -f "$SC_DIR/$SC.yaml" ] || { echo "시나리오 '$SC' 가 없습니다. 인자 없이 실행하면 목록이 나옵니다."; exit 1; }
pkill -f "gz sim" 2>/dev/null     # 이전 Gazebo 가 남아 있으면 정리
exec ros2 launch kickboard_sim bringup.launch.py scenario:=$SC "$@"
