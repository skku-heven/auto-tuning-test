#!/usr/bin/env bash
# Run TPE and GP until EACH study reaches TARGET total trials.
# Resumable: Optuna storage is load_if_exists=True, so rerunning continues.
#   run_ab_balanced.sh [target_trials] [hours|forever]
#   run_ab_balanced.sh 1000 forever
set -u

TARGET="${1:-1000}"
HOURS="${2:-forever}"
HERE="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/noetic/setup.bash
source "$HOME/heven_common_test_ws/devel/setup.bash"

STOR_ARG=()
[ -n "${OPTUNA_STORAGE:-}" ] && STOR_ARG=(--storage "$OPTUNA_STORAGE")

BUDGET_ARG=()
case "$HOURS" in
  0|forever|inf|-1) BUDGET_ARG=(--forever) ;;
  *) BUDGET_ARG=(--hours "$HOURS") ;;
esac

PYTHONNOUSERSITE=0 python3 "$HERE/auto_tune_ab.py" \
  --mode balanced \
  --target-trials "$TARGET" \
  --seg 400 \
  --timeout 120 \
  "${BUDGET_ARG[@]}" \
  "${STOR_ARG[@]}"
