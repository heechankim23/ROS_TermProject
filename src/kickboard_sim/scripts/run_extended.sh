#!/usr/bin/env bash
set -euo pipefail
scenario="${1:-s10_damaged_kickboard}"
shift || true
exec ros2 launch kickboard_sim extended.launch.py "scenario:=$scenario" sim:=gazebo "$@"
