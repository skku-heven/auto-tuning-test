# Optuna 튜닝 재설계 — 목적함수 재구성 + GP/TPE A/B (2026-07-02)

## 배경
MORAI 라이브에서 governed-Stanley 제어기 연속 파라미터 5개를 튜닝. 각 trial =
리셋 텔레포트(~8s) → 400m 주행(30~120s) → competition 점수. 3시간 예산 ≈ 60~90 trial.
실시간이라 노이즈 있음. 기존 목적함수는 완주여부에 **±1e6 절벽**이 있어 GP surrogate가
못 다룸. 사용자가 GPSampler를 쓰고 싶어함 → 눈감고 채택 말고 TPE와 A/B로 검증 후 채택.

## 결정 (사용자 승인 완료)
1. **목적함수 재구성**: 완주 절벽 삭제.
   - `direction=minimize`, 목적값 = 주행시간 `time_s`.
   - 미완주는 **연속 proxy** (항상 완주보다 나쁨, 진행할수록 감소):
     `proxy = timeout_s + (seg - progress_s) + max_cte*0.5`
   - **완주여부 = 제약조건**(`constraints_func`, 값 ≤0 = feasible):
     reset 실패/disconnect → `[1.0]`; 그 외 `[0.0 if completed else 1.0]`.
   - RESET/DISCONNECT는 물리정보 아님 → objective에 큰 상수 안 섞고 proxy+제약위반+user_attr 로깅.
     같은 파라미터 1회 재시도 후 실패 처리.
2. **A/B (둘 다 튜닝)**: 스터디 2개.
   - `kcity_seg400_tpe_v4` : `TPESampler(constraints_func, n_startup_trials=15, multivariate=True, group=True, seed)`
   - `kcity_seg400_gp_v4`  : `GPSampler(constraints_func, n_startup_trials=15, deterministic_objective=False, seed)`
   - 단일 시뮬(차 1대)이라 trial은 순차 → **interleave**(fairness): 한 라운드에 tpe 1 trial, gp 1 trial 교대.
   - **Smoke**: 각 25 trial(총 50). **비교 지표**: ① 완주율(feasible rate) ② feasible 중 median `time_s`.
     승자 = 완주율 우선, 동률이면 median time 낮은 쪽.
   - **Full**: 승자 스터디로 남은 시간까지.
   - **Final**: 승자 top 3~5 파라미터를 각 3회 **재측정** → 완주율·worst-case CTE·평균 time으로 최종 1개 선택
     (best 1 trial 그대로 채택 금지 — 노이즈).
3. **Pruner 없음**: objective 내부 명시적 abort(진행부족/CTE초과 시 유한 나쁜값)로 대체.
   Optuna pruner는 pruned trial에서 constraints_func 호출 안 되어 feasibility 학습 데이터 손실.
4. **Warm-start 6개**(`enqueue_trial`, 두 스터디 모두): 실제 완주기록 기반, 공격~안전 다양하게.

## 탐색공간 (5 파라미터, pid_kp=0.3 고정)
| param | range | scale |
|---|---|---|
| lookahead | 1.5 ~ 5.5 | linear |
| target_velocity_kph | 25 ~ 55 | linear (시간최소화라 저속은 어차피 안 뽑힘 → 하한 25로 낭비 trial 제거; 실제 상한은 곡률 프로파일이 결정) |
| gain_k | 0.4 ~ 3.0 | **log** |
| k_soft | 0.5 ~ 3.0 | linear |
| a_lat | 1.0 ~ 3.0 | linear |

## Warm-start 6점 (v3 seg400 완주기록에서)
| # | 성격 | lookahead | v_kph | gain_k | k_soft | a_lat | (v3 time/cte) |
|---|---|---|---|---|---|---|---|
| 1 | 빠름·저CTE | 4.26 | 49.8 | 2.77 | 1.72 | 2.84 | 48.8s / 0.57 |
| 2 | 깨끗·빠름 | 2.19 | 37.4 | 2.43 | 0.58 | 2.48 | 59.6s / 0.16 |
| 3 | 균형 | 4.19 | 41.9 | 2.03 | 1.83 | 2.54 | 54.8s / 0.66 |
| 4 | 빠름 | 5.15 | 42.6 | 2.70 | 0.99 | 2.28 | 53.4s / 0.78 |
| 5 | 저gain 영역 | 3.71 | 35.5 | 1.13 | 0.77 | 1.27 | 66.5s / 0.59 |
| 6 | 중속·깨끗·저a_lat | 2.74 | 30.0 | 1.38 | 2.53 | 1.13 | 73.1s / 0.27 |

## 구현 범위
- 신규 오케스트레이터 스크립트 1개(A/B interleave + smoke비교 + full + final 재측정). 기존
  `auto_tune_live.py`의 reset/drive/score 로직은 재사용(제어기 `ad_control` 그대로).
- 기존 v3 DB/스터디는 보존(비교/롤백용), v4는 새 목적(minimize·제약) → 새 스터디명.
- 제어기(`ad_control.py`)는 **변경 없음**.

## 비범위 (YAGNI)
- BoTorchSampler(의존성/운영 리스크). 배속/lockstep(빌드 미지원). 세그먼트 다변화.
- 제어기 구조 변경(현 governed-Stanley 유지).

## 리스크
- Optuna 버전별 `constraints_func`/GPSampler 동작 차이 → 구현 전 설치버전 문서 재확인.
- 단일 시뮬 순차라 A/B가 시간축 드리프트 영향 받음 → interleave로 완화.
- MORAI 재연결 필요(세션 teardown으로 브릿지 죽음) → 튜닝 시작 전 스택 복구 선행.
