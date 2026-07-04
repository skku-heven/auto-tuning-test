#!/usr/bin/env bash
# auto_tune_ab를 감싸 크래시해도 재시작. 파라미터/DB는 SQLite(또는 OPTUNA_STORAGE)로 resume.
#   run_ab_forever.sh <hours|forever> [smoke]
#   HOURS가 0/forever/-1 이면 멈출 때까지 무제한(tmux/kill로 정지).
# DB: OPTUNA_STORAGE 지정 시 그 URL 사용(예: postgresql://user:pw@host/db). 미지정 시 sqlite.
set -u
HOURS="${1:-forever}"; SMOKE="${2:-25}"
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/noetic/setup.bash
source "$HOME/auto_tuning_ws/devel/setup.bash"   # 독립 ws (C++ ad_tracker 빌드본 포함)
STOR_ARG=()
[ -n "${OPTUNA_STORAGE:-}" ] && STOR_ARG=(--storage "$OPTUNA_STORAGE")

FOREVER=0
case "$HOURS" in 0|forever|inf|-1) FOREVER=1;; esac

if [ "$FOREVER" = 1 ]; then
  echo "[forever] 무제한 모드 — 멈출 때까지. (tmux kill-session -t tuning 로 정지)"
  while true; do
    echo "[forever] (re)start $(date '+%H:%M:%S')"
    PYTHONNOUSERSITE=0 python3 "$HERE/auto_tune_ab.py" --forever --mode balanced \
      --runner tracker --seg 400 --timeout 120 --smoke "$SMOKE" "${STOR_ARG[@]}" || true
    sleep 10
  done
else
  END=$(( $(date +%s) + $(printf '%.0f' "$(echo "$HOURS*3600" | bc)") ))
  while [ "$(date +%s)" -lt "$END" ]; do
    LEFT=$(( END - $(date +%s) )); H=$(echo "scale=3; $LEFT/3600" | bc)
    echo "[forever] restart, ${H}h left"
    PYTHONNOUSERSITE=0 python3 "$HERE/auto_tune_ab.py" --hours "$H" --mode balanced \
      --runner tracker --seg 400 --timeout 120 --smoke "$SMOKE" "${STOR_ARG[@]}" || true
    sleep 10
  done
  echo "[forever] deadline reached"
fi
