# auto-tuning-test

tracking, 각종 미션의 auto tuning을 위한 테스트 레포.

heven-common-test에서 이관한 자율주행(`ad_*`) 패키지 + MORAI 라이브 Optuna 튜닝 시스템.

## 패키지
- **ad_autotune** — Optuna 기반 파라미터 자동튜닝(GP vs TPE A/B). MORAI 라이브 주행 → path-tracking 목적함수. 핵심 스크립트: `scripts/{auto_tune_ab, ab_core, live_runner, ad_control, autotune_core}.py`
- **ad_metric** — 주행 메트릭 / 대회 점수 계산(rosbag 기반)
- **ad_tracker** — 경로추종 컨트롤러(C++, 실차용)
- **ad_pose_parser / ad_tf2_broadcaster / ad_udp / ad_lidar_bringup** — 센서·pose·TF·브링업

## 튜닝 목적함수 (현재)
```
minimize  cost = time_s + 50·mean(cte²) + 5·overspeed_s
constraint(feasible)  = 완주 AND max_cte ≤ 1.0m
탐색공간  target_velocity 25~50 (50kph 상한)
```
(상수는 `ad_autotune/scripts/ab_core.py` 상단에서 조정)

## 튜닝 실행 (동방 heven-z790-ud)
catkin workspace의 `src/`에 clone → `catkin build` → MORAI 연결(ROS Connect + Play) 후:
```bash
# ROS 스택 기동(roscore + rosbridge + primer). rosbridge는 tmux 세션에 넣을 것.
bash ad_autotune/scripts/start_stack.sh
# 무제한 A/B 튜닝 (멈출 때까지). tmux 세션 권장.
bash ad_autotune/scripts/run_ab_forever.sh forever 25
```
- 실행은 `PYTHONNOUSERSITE=0`(GPSampler의 torch가 user-site에 있음).
- DB: 기본 SQLite `ad_autotune/results/live_tune/tune_ab.db`. PostgreSQL 이전 시 `OPTUNA_STORAGE=postgresql://user:pw@host/db` 로 실행(스키마 동일).

## 설계 문서
- `docs/2026-07-02-optuna-ab-design.md` — 목적함수·제약·GP/TPE A/B 설계
- `docs/2026-07-02-optuna-ab-gp-tpe.md` — 구현 플랜(6 task)
