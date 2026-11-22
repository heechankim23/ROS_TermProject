#!/bin/bash
# 시나리오 전체(또는 고른 것)를 Gazebo 로 하나씩 차례로 돌리고 결과를 표로 정리한다.
#
#   ./run_all_gazebo.sh                     전체 19개, 화면 없이 (가장 빠르고 안정적)
#   GUI=1 ./run_all_gazebo.sh               전체, Gazebo 화면을 띄워서
#   ./run_all_gazebo.sh s1_single s10_road_collect      고른 것만
#   LIMIT=1500 ./run_all_gazebo.sh          시나리오당 제한 시간(초) 변경 (기본 1200)
#
# 한 번에 하나만 돌린다 (동시에 여러 개 돌리면 CPU 가 포화되어 결과가 틀어진다).
# 로그: results/<날짜_시각>/<시나리오>.log, 요약: results/<날짜_시각>/summary.txt
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] || colcon build || exit 1
source install/setup.bash
SC_DIR=src/kickboard_sim/config/scenarios
LIMIT=${LIMIT:-1200}
OUT=results/$(date +%m%d_%H%M)
mkdir -p "$OUT"
if [ $# -gt 0 ]; then LIST="$*"; else LIST=$(ls $SC_DIR/*.yaml | sort -V | xargs -n1 basename | sed 's/\.yaml$//'); fi
if [ "${GUI:-0}" = "1" ]; then VIEW="gui:=true"; else VIEW="gui:=false sensors:=false"; fi

stop_sim() {            # 이번 실행이 띄운 프로세스 정리
  [ -n "$LP" ] && kill -INT -- -"$LP" 2>/dev/null
  sleep 5
  [ -n "$LP" ] && kill -KILL -- -"$LP" 2>/dev/null
  pkill -KILL -f "gz si[m]" 2>/dev/null
  LP=
}
trap 'stop_sim; echo; echo "중단됨. 지금까지 결과: $OUT/summary.txt"; exit 1' INT TERM

printf "%-26s %-8s %6s  %s\n" "시나리오" "결과" "시간" "수거/실패/취소/알림" | tee "$OUT/summary.txt"
for SC in $LIST; do
  F=$SC_DIR/$SC.yaml
  [ -f "$F" ] || { echo "$SC: 시나리오 파일 없음"; continue; }
  # 마지막 이벤트·등장 시각: 그 전에 로봇이 잠깐 쉬는 것(IDLE)을 '끝'으로 착각하지 않게 한다
  LAST=$(python3 -c "
import yaml,sys
d=yaml.safe_load(open('$F'))
t=[float(e.get('t',0)) for e in d.get('events') or []]+[float(s.get('spawn_at',0)) for s in d.get('scooters') or []]
print(int(max(t+[0])))")
  LOG="$OUT/$SC.log"
  setsid ros2 launch kickboard_sim bringup.launch.py scenario:=$SC $VIEW > "$LOG" 2>&1 &
  LP=$!
  T0=$(date +%s); RES="시간초과"
  while true; do
    sleep 5
    EL=$(( $(date +%s) - T0 ))
    CLEAN=$(sed 's/\x1b\[[0-9;]*m//g' "$LOG")
    # 끝 = 한 번이라도 출동한 뒤, 마지막 줄 상태가 IDLE 이고, 시나리오 이벤트가 모두 지난 뒤(여유 2배 + 90초)
    LASTSTATE=$(echo "$CLEAN" | grep "\[상태\]" | tail -1)
    if echo "$CLEAN" | grep -q "NAVIGATING" && echo "$LASTSTATE" | grep -q -- "-> IDLE" && [ $EL -ge $(( LAST * 2 + 90 )) ]; then
      sleep 8
      [ "$(sed 's/\x1b\[[0-9;]*m//g' "$LOG" | grep "\[상태\]" | tail -1)" = "$LASTSTATE" ] && { RES="완료"; break; }
    fi
    kill -0 $LP 2>/dev/null || { RES="중단됨"; break; }
    [ $EL -ge $LIMIT ] && break
  done
  stop_sim
  CLEAN=$(sed 's/\x1b\[[0-9;]*m//g' "$LOG")
  NC=$(echo "$CLEAN" | grep -c "\[collector\]: 킥보드 .* 적재 완료")
  NF=$(echo "$CLEAN" | grep -c "\[상태\] .* -> FAILED")
  NX=$(echo "$CLEAN" | grep -c "자동 취소")
  NA=$(echo "$CLEAN" | grep -c "\[mission_manager\]: \[관리자 알림\]")
  printf "%-26s %-8s %5ss  수거 %s / 실패 %s / 취소 %s / 알림 %s\n" "$SC" "$RES" "$EL" "$NC" "$NF" "$NX" "$NA" | tee -a "$OUT/summary.txt"
done
echo "요약: $WS/$OUT/summary.txt   (시나리오별 로그는 같은 폴더)"
