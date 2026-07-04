# [transferable-to-heven_ad]
"""tracker_runner — C++ ad_tracker를 trial마다 relaunch해서 평가하는 러너.

LiveRunner(파이썬 제어+측정 겸업)와 달리 여기서는 제어를 C++ ad_tracker에 맡기고
순수 관찰자로 메트릭만 잰다. LiveRunner와 같은 인터페이스(reset_and_arm/drive/_fresh)라
auto_tune_ab의 objective를 그대로 재사용.

trial 시퀀스:
  reset_and_arm(): 이전 trial tracker 확실히 종료(좀비가드) → 텔레포트 → External 모드
  drive(p): roslaunch ad_tracker(p 주입) → readiness gate(/ctrl_cmd 수신 확인) →
            관찰 루프(메트릭) → tracker 종료 + 정지 브레이크

전제: ego_pose_bridge(/Ego_topic→/ad_pose_parser/pose)가 별도로 떠 있어야 함.
"""
import math, os, signal, subprocess, time


def _mods():
    try:
        import rospkg
        p = os.path.join(rospkg.RosPack().get_path("ad_autotune"), "scripts")
    except Exception:
        p = os.path.dirname(os.path.abspath(__file__))
    import sys
    sys.path.insert(0, p)
    import autotune_core
    return autotune_core


# pgrep/pkill 자기매칭 함정 회피용 브래킷 패턴 (이 문자열 자체는 안 매칭됨)
TRACKER_PGREP = "[a]d_tracker"


def build_launch_args(p, csv_path):
    """파라미터 dict → roslaunch 인자 리스트 (순수함수, 테스트 대상)."""
    args = ["roslaunch", "ad_tracker", "ad_tracker.launch",
            f"csv_path:={csv_path}",
            "status_topic:=/Ego_topic"]      # 튜닝 땐 ground-truth 속도(/Competition_topic은 UDP 경유)
    for k in ("lookahead", "target_velocity_kph", "gain_k", "k_soft", "a_lat",
              "pid_kp", "pid_ki", "pid_kd"):
        if k in p:
            args.append(f"{k}:={p[k]}")
    return args


class TrackerRunner:
    def __init__(self, track, s_arr, seg, timeout, start, csv_path):
        self.core = _mods()
        self.track = track; self.s_arr = s_arr
        self.seg = seg; self.timeout = timeout; self.start = start
        self.csv_path = csv_path
        self.proc = None
        import rospy
        from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, MultiEgoSetting
        self.rospy = rospy
        self._CtrlCmd = CtrlCmd; self._MultiEgoSetting = MultiEgoSetting
        self.stop_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        self.egopub = rospy.Publisher("/ego_setting", MultiEgoSetting, queue_size=1)
        self.st = {"x": None, "y": None, "v": 0.0, "t": 0.0}
        self.cmd = {"n": 0}

        def cb(m):
            self.st["x"] = m.position.x; self.st["y"] = m.position.y
            self.st["v"] = math.hypot(m.velocity.x, m.velocity.y) * 3.6
            self.st["t"] = rospy.get_time()
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, cb, queue_size=10)

        def ccb(_m):
            self.cmd["n"] += 1
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, ccb, queue_size=10)

        try:
            from morai_msgs.srv import MoraiEventCmdSrv
            rospy.wait_for_service("/Service_MoraiEventCmd", timeout=5)
            self.event_srv = rospy.ServiceProxy("/Service_MoraiEventCmd", MoraiEventCmdSrv)
        except Exception:
            self.event_srv = None

    def _fresh(self):
        return self.st["x"] is not None and (self.rospy.get_time() - self.st["t"]) < 0.5

    # ---------- tracker 프로세스 관리 ----------
    def _stray_tracker_pids(self):
        try:
            out = subprocess.run(["pgrep", "-f", TRACKER_PGREP],
                                 capture_output=True, text=True, timeout=5).stdout.split()
            return [int(x) for x in out]
        except Exception:
            return []

    def _kill_tracker(self):
        """자기 proc SIGINT(roslaunch 클린 셧다운) → 좀비 pkill → /ctrl_cmd 침묵 확인."""
        if self.proc is not None and self.proc.poll() is None:
            try:
                self.proc.send_signal(signal.SIGINT)
                self.proc.wait(timeout=10)
            except Exception:
                try:
                    self.proc.kill(); self.proc.wait(timeout=5)
                except Exception:
                    pass
        self.proc = None
        # 좀비가드: 우리가 안 띄운 ad_tracker 포함 전부 정리 (과거 이중발행 사고 재발 방지)
        for _ in range(3):
            pids = self._stray_tracker_pids()
            if not pids:
                break
            subprocess.run(["pkill", "-INT", "-f", TRACKER_PGREP], timeout=5)
            time.sleep(1.5)
        pids = self._stray_tracker_pids()
        if pids:
            subprocess.run(["pkill", "-KILL", "-f", TRACKER_PGREP], timeout=5)
            time.sleep(0.5)
        # /ctrl_cmd 침묵 확인(0.6s 동안 새 메시지 없어야)
        n0 = self.cmd["n"]; self.rospy.sleep(0.6)
        return self.cmd["n"] == n0

    def _launch_tracker(self, p, ready_timeout=20.0):
        """roslaunch 기동 + readiness gate: /ctrl_cmd가 실제로 나올 때까지 대기."""
        args = build_launch_args(p, self.csv_path)
        self.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL, start_new_session=True)
        n0 = self.cmd["n"]
        t0 = self.rospy.get_time()
        while self.rospy.get_time() - t0 < ready_timeout:
            if self.proc.poll() is not None:
                return False                       # 기동 실패(즉사)
            if self.cmd["n"] >= n0 + 1:            # 첫 명령 = pose 수신+제어 시작(관찰 공백 최소화)
                return True
            self.rospy.sleep(0.05)
        return False

    # ---------- LiveRunner 호환 인터페이스 ----------
    def reset_and_arm(self):
        rospy = self.rospy; S = self.start
        if not self._kill_tracker():
            rospy.logwarn("tracker_runner: /ctrl_cmd not silent after kill")
            return False
        m = self._MultiEgoSetting()
        m.number_of_ego_vehicle = 1; m.camera_index = 0; m.ego_index = [0]
        m.global_position_x = [S[0]]; m.global_position_y = [S[1]]; m.global_position_z = [S[2]]
        m.global_roll = [0.0]; m.global_pitch = [0.0]; m.global_yaw = [S[3]]
        m.velocity = [0.0]; m.gear = [4]; m.ctrl_mode = [16]
        m.steering_angle = [0.0]; m.vehicle_speed = [0.0]; m.turn_signal = [0]; m.brake_light = [False]
        t0 = rospy.get_time()
        while self.egopub.get_num_connections() == 0 and rospy.get_time() - t0 < 5:
            rospy.sleep(0.1)
        for _ in range(8):
            self.egopub.publish(m); rospy.sleep(0.1)
        t0 = rospy.get_time()
        while rospy.get_time() - t0 < 8:
            if self._fresh() and math.hypot(self.st["x"] - S[0], self.st["y"] - S[1]) < 3.0:
                break
            rospy.sleep(0.1)
        else:
            return False
        if self.event_srv:
            try:
                from morai_msgs.msg import EventInfo
                ev = EventInfo(); ev.option = 3; ev.ctrl_mode = 3; ev.gear = 4
                self.event_srv(ev)
            except Exception:
                pass
        rospy.sleep(0.5)
        # 텔레포트가 속도를 안 0으로 만들었을 수 있음 → 브레이크로 bleed(트래커 없을 때만 안전)
        t0 = rospy.get_time()
        while rospy.get_time() - t0 < 3.0 and self.st["v"] > 0.5:
            self._stop_car()
        return True

    def _stop_car(self):
        stop = self._CtrlCmd(); stop.longlCmdType = 1; stop.brake = 1.0
        for _ in range(6):
            self.stop_pub.publish(stop); self.rospy.sleep(0.05)

    def drive(self, p, save_path, seg=None, timeout=None):
        rospy = self.rospy; track = self.track; s_arr = self.s_arr
        seg = seg if seg is not None else self.seg
        timeout = timeout if timeout is not None else self.timeout
        if not self._launch_tracker(p):
            self._kill_tracker()
            rospy.logwarn("tracker_runner: launch/readiness failed")
            return None

        n = len(track)
        # 관찰용 monotonic nearest (시작점에서 global 시드)
        x0, y0 = self.st["x"], self.st["y"]
        cur = min(range(n), key=lambda i: (track[i][0] - x0) ** 2 + (track[i][1] - y0) ** 2)
        max_s = max_cte = sum_cte = sum_cte_sq = 0.0; cnt = 0
        offtrack = overspeed = 0.0; diverged = False; complete_t = None
        stall_t = None; stale_since = None; samples = []
        tracker_died = False
        rate = rospy.Rate(20); t0 = rospy.get_time(); prev_t = t0
        while not rospy.is_shutdown():
            now = rospy.get_time(); t = now - t0; dt = now - prev_t; prev_t = now
            if t > timeout:
                break
            if self.proc.poll() is not None:
                tracker_died = True; break
            if not self._fresh():
                stale_since = stale_since or now
                if now - stale_since > 3.0:
                    self._kill_tracker()
                    return None
                rospy.sleep(0.05); continue
            stale_since = None
            x, y, v = self.st["x"], self.st["y"], self.st["v"]
            best_i, best_d = cur, 1e18
            for i in range(cur, min(cur + 200, n)):
                d = (track[i][0] - x) ** 2 + (track[i][1] - y) ** 2
                if d < best_d:
                    best_d, best_i = d, i
            cur = near = best_i
            cte = self.core._path_lateral_error(track, x, y)
            max_s = max(max_s, s_arr[near]); max_cte = max(max_cte, cte)
            sum_cte += cte; sum_cte_sq += cte * cte; cnt += 1
            if cte > 1.5: offtrack += dt
            if v > 50: overspeed += dt
            samples.append((round(t, 2), round(x, 2), round(y, 2), round(v, 1), round(cte, 2), near))
            if v < 1.0 and t > 8:                  # C++ 기동 딜레이 감안해 5→8s
                stall_t = stall_t or now
                if now - stall_t > 4.0:
                    diverged = True; break
            else:
                stall_t = None
            if cte > 12: diverged = True; break
            if s_arr[near] >= seg: complete_t = t; break
            rate.sleep()
        self._kill_tracker()
        self._stop_car()
        if tracker_died:
            rospy.logwarn("tracker_runner: ad_tracker died mid-trial")
            return None
        completed = complete_t is not None
        penalty = 5.0 * int(offtrack // 3.0) + (10.0 if overspeed > 0 else 0) + 10.0 * int(overspeed // 3.0)
        m = dict(completed=completed, diverged=diverged, progress_s=round(max_s, 1),
                 time_s=round(complete_t, 2) if completed else timeout,
                 driving_score=max(0.0, 100.0 - penalty), max_cte=round(max_cte, 2),
                 mean_cte=round(sum_cte / max(cnt, 1), 3),
                 mean_cte_sq=round(sum_cte_sq / max(cnt, 1), 4), offtrack_s=round(offtrack, 1),
                 overspeed_s=round(overspeed, 1))
        if save_path:
            with open(save_path, "w") as f:
                f.write("t,x,y,v_kph,cte,near\n")
                for s_ in samples:
                    f.write(",".join(map(str, s_)) + "\n")
        return m
