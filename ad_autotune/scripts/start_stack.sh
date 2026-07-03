#!/usr/bin/env bash
# ROS 스택 기동(roscore + rosbridge_websocket + primer). MORAI는 GUI로 연결.
# rosbridge가 morai_msgs 타입을 알도록 devel dist-packages를 PYTHONPATH에 넣음.
set -u
source /opt/ros/noetic/setup.bash
source "$HOME/heven_common_test_ws/devel/setup.bash"
HERE="$(cd "$(dirname "$0")" && pwd)"
DP="$HOME/heven_common_test_ws/devel/lib/python3/dist-packages"

if ! pgrep -x rosmaster >/dev/null; then
  echo "[stack] roscore 시작"; setsid roscore >/tmp/roscore.log 2>&1 < /dev/null &
  sleep 3
fi
if ! ss -tlnp 2>/dev/null | grep -q ':9090'; then
  echo "[stack] rosbridge_websocket 시작(:9090)"
  PYTHONPATH="$DP:${PYTHONPATH:-}" setsid roslaunch rosbridge_server rosbridge_websocket.launch >/tmp/rosbridge.log 2>&1 < /dev/null &
  sleep 4
fi
if ! pgrep -f "primer.py" >/dev/null; then
  echo "[stack] primer 시작"
  PYTHONNOUSERSITE=1 setsid python3 "$HERE/primer.py" >/tmp/primer.log 2>&1 < /dev/null &
  sleep 2
fi
echo "[stack] 토픽 상태:"
rostopic list 2>/dev/null | grep -E 'Ego_topic|ctrl_cmd|ego_setting|Competition' || echo "(토픽 아직 없음 — MORAI ROS Connect 대기)"
