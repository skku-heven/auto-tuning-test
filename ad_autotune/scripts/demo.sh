#!/usr/bin/env bash
# [transferable-to-heven_ad]
# One-command demo of the whole ad_autotune workflow — NO MORAI needed.
#   ./demo.sh
# 1) offline auto-tune on two tracks  2) make a bag from the tuned run
# 3) score the bag through the ROS metric node (full pipeline end-to-end)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

echo "############################################################"
echo "# 1) OFFLINE AUTO-TUNE  (bicycle plant, no sim)"
echo "############################################################"
for T in oval tight; do
  echo "----- track: $T -----"
  python3 autotune.py --track "$T" 2>&1 | grep -E 'BEST after|lookahead=|completed='
  echo
done

echo "############################################################"
echo "# 2+3) ROS METRIC PIPELINE  (sim run -> rosbag -> 대회 채점)"
echo "############################################################"
WS="$(cd "$HERE/../../../.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true
source "$WS/devel/setup.bash" >/dev/null 2>&1 || true
export PYTHONNOUSERSITE=1
if python3 -c 'import rosbag, morai_msgs' 2>/dev/null; then
  python3 make_test_bag.py --track oval --velocity 45 --out /tmp/ad_autotune_demo.bag
  python3 ../../ad_metric/scripts/competition_score.py \
      --bag /tmp/ad_autotune_demo.bag --csv ../paths/oval_track.csv
else
  echo "[skip] rosbag/morai_msgs 미가용 — 'catkin build && source devel/setup.bash' 후 재실행"
fi

echo
echo "DONE. results in ad_autotune/results/ . To go live: see README '## Going live'."
