# ad_autotune — build progress (loop-tracked)

Auto-tuning workflow for ad_tracker. Built while user away (2026-06-29, /loop).

## ▶ 복귀하면 여기부터 (START HERE)
워크플로우 완성됨. MORAI 없이 지금 바로 보려면:
```bash
cd ad_autotune/scripts && ./demo.sh      # 전체 파이프라인 한 방에 (sim 불필요)
```
**라이브로 가려면** (roscore+rosbridge는 내가 백그라운드로 띄워둠):
1. MORAI 창 → Ego Network → ROS → **Connect** (이것만 네가 눌러야 함)
2. `rostopic list`에 /Competition_topic 뜨는지 확인
3. 새 맵 경로 녹화:  수동주행 후 `rosrun ad_autotune record_path.py --out <csv>`
4. 튜닝:  `python3 autotune.py --csv <csv>`
5. 주행:  `roslaunch ad_tracker ad_tracker.launch <tuned args>`  →  bag 녹화
6. 채점:  `rosrun ad_metric competition_score.py --bag <bag> --csv <csv>`
- trial간 ego 리셋 자동화는 `run_live_tuning.py` (SimControl 죽어서 리셋부분만 미검증).
- A/B/C 컨트롤러 버그는 네가 넣기로 함 → README "Known blockers" 4번.

## DONE ✅
- ROS bridge up: roscore(:11311) + rosbridge_websocket(:9090) running in bg.
- `autotune_core.py` — synthetic oval track gen, kinematic bicycle plant,
  faithful Stanley mirror of gps_tracker.cpp, `score_run` (competition scoring).
- `autotune.py` — offline optimizer (grid + Hooke-Jeeves). TESTED: 262 evals,
  baseline 32.5s→tuned 15.3s @ score 100 (V converged to 45.9, just under 50).
- `run_live_tuning.py` — live/dry-run trial loop over ROS. TESTED in --dry-run.
- package.xml, CMakeLists.txt, README.md, oval_track.csv, results/.
- 2nd track (`tight`, sharp corners) + ASCII visualizer. Optimizer ADAPTS:
  oval→LA=10, tight→LA=4 (proves it's not just maxing speed).
- ANTI-GAMING: caught high-LA corner-cutting (6m off, still scored 100 because
  short runs miss the 3s-cumulative penalty). Fixed: `score_run` voids 완주 on
  코스이탈 (>2m for >0.7s). Tuned runs now hug path (mean_cte<0.7m).

## DONE (cycle 4) ✅
- catkin build PASSES (ad_autotune + ad_metric, 1.7s, in-tree). rosrun works.
- Fixed stale-devel-copy bug: scripts now import autotune_core from SOURCE via
  rospkg (works from source AND rosrun). Cleared stale pyc.
- `scripts/demo.sh` — ONE COMMAND runs the whole workflow (offline tune both
  tracks → bag → ROS 채점). VALIDATED end-to-end, no MORAI needed.

## DONE (cycle 5) ✅
- `scripts/record_path.py` — record global_path.csv by driving (live subscribe)
  OR extract from a bag. 2-col output (dodges bug A). Solves scenario-coords:
  drive new map R_KR_PR_2025 → fresh path. VALIDATED via bag (300 wpt, ~169m).
- `autotune.py --csv <path>` — tune on a recorded path (not just synthetic).
- record→tune CHAIN proven: bag → record_path → autotune --csv → tuned params.

## DONE (cycle 6) ✅
- `scripts/test_autotune.py` — 18 self-contained tests (no pytest). Covers tracks,
  clean-run=100, speeding penalty, corner-cut→off_road 실격, objective ordering
  (주행점수>시간), offline/live score_run parity, write/load round-trip. 18/18 pass.
- Test caught my own wrong assumption → confirmed objective correctly ranks a
  clean slow run ABOVE a penalised faster one (규정대로). Harness validated.

## BLOCKED (needs user) ⛔
- MORAI live: Ego Network "Connect" is a UI press → topics not flowing yet.
  Bridge is up & waiting; poll `rostopic list` for /Competition_topic.
- Ego reset between live trials: SimControl dead → /Service_MoraiEventCmd
  (best-effort) or --manual-reset. Unverified on this build.

## DONE (cycle 3) ✅
- `ad_metric/scripts/competition_score.py` — ROS/rosbag 대회 채점 노드 (수정 G),
  imports score_run (single source of truth) + 충돌 감점. ADDITIVE (calc_metric.py 안 건드림).
- `ad_autotune/scripts/make_test_bag.py` — 시뮬런→rosbag, MORAI 없이 메트릭 검증.
- VALIDATED end-to-end: 합성 bag→competition_score 점수 일치(완주100/17.25s),
  속도 60kph bag→ -30 감점 정상. CMakeLists 둘 다 install 등록.

## NEXT (loop can pick up) 🔜
- [ ] Bug A/B/C in ad_tracker/calc_metric — user said they'd apply themselves;
      do NOT touch gps_tracker.cpp/pose_parser.cpp. README documents them.
- [ ] When MORAI connects: run `run_live_tuning.py` live, confirm reset path.
- [ ] Optional: catkin build to confirm ad_autotune package compiles in-tree.
- [ ] Optional: widen optimizer (add pid_ki/kd dims) once live metric is real.
