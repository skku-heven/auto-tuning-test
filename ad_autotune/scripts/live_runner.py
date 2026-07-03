# [transferable-to-heven_ad]
"""live_runner — MORAI 리셋 텔레포트 + governed-Stanley 세그먼트 주행.
auto_tune_live(v3)의 reset_and_arm/drive를 클래스화(로직 동일). 호출자가 rospy.init_node 선행."""
import math, os, sys


def _mods():
    try:
        import rospkg
        p = os.path.join(rospkg.RosPack().get_path("ad_autotune"), "scripts")
    except Exception:
        p = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, p)
    import autotune_core, ad_control
    return autotune_core, ad_control


class LiveRunner:
    def __init__(self, track, s_arr, seg, timeout, start):
        self.core, self.ctrl = _mods()
        self.track = track; self.s_arr = s_arr
        self.seg = seg; self.timeout = timeout; self.start = start
        import rospy
        from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, MultiEgoSetting
        self.rospy = rospy
        self._CtrlCmd = CtrlCmd; self._MultiEgoSetting = MultiEgoSetting
        self.pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        self.egopub = rospy.Publisher("/ego_setting", MultiEgoSetting, queue_size=1)
        self.st = {"x": None, "y": None, "th": 0.0, "v": 0.0, "t": 0.0}

        def cb(m):
            self.st["x"] = m.position.x; self.st["y"] = m.position.y
            self.st["th"] = math.radians(m.heading)
            self.st["v"] = math.hypot(m.velocity.x, m.velocity.y) * 3.6
            self.st["t"] = rospy.get_time()
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, cb, queue_size=10)
        try:
            from morai_msgs.srv import MoraiEventCmdSrv
            rospy.wait_for_service("/Service_MoraiEventCmd", timeout=5)
            self.event_srv = rospy.ServiceProxy("/Service_MoraiEventCmd", MoraiEventCmdSrv)
        except Exception:
            self.event_srv = None

    def _fresh(self):
        return self.st["x"] is not None and (self.rospy.get_time() - self.st["t"]) < 0.5

    def reset_and_arm(self):
        rospy = self.rospy; S = self.start
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
        return True

    def drive(self, p, save_path):
        rospy = self.rospy; track = self.track; s_arr = self.s_arr
        s, prof = self.ctrl.build_profile(track, a_lat=p["a_lat"])
        gs = self.ctrl.GovernedStanley(track, s, prof, p["lookahead"], p["gain_k"], p["k_soft"], p["pid_kp"])
        gs.reset(self.st["x"], self.st["y"])
        max_s = max_cte = sum_cte = sum_cte_sq = 0.0; cnt = 0
        offtrack = overspeed = 0.0; diverged = False; complete_t = None
        stall_t = None; stale_since = None; samples = []
        rate = rospy.Rate(20); t0 = rospy.get_time(); prev_t = t0
        while not rospy.is_shutdown():
            now = rospy.get_time(); t = now - t0; dt = now - prev_t; prev_t = now
            if t > self.timeout:
                break
            if not self._fresh():
                stale_since = stale_since or now
                if now - stale_since > 3.0:
                    return None
                rospy.sleep(0.05); continue
            stale_since = None
            x, y, th, v = self.st["x"], self.st["y"], self.st["th"], self.st["v"]
            target = min(p["target_velocity_kph"],
                         5.0 + (p["target_velocity_kph"] - 5.0) * min(t / 4.0, 1.0))
            steer, thr, brk, info = gs.step(x, y, th, v, dt, target)
            cmd = self._CtrlCmd(); cmd.longlCmdType = 1
            cmd.accel = float(thr); cmd.brake = float(brk)
            cmd.steering = float(max(-1.0, min(1.0, steer / 0.698))); cmd.front_steer = float(steer)
            self.pub.publish(cmd)
            cte = self.core._path_lateral_error(track, x, y)
            near = info["near"]
            max_s = max(max_s, s_arr[near]); max_cte = max(max_cte, cte)
            sum_cte += cte; sum_cte_sq += cte * cte; cnt += 1
            if cte > 1.5: offtrack += dt
            if v > 50: overspeed += dt
            samples.append((round(t, 2), round(x, 2), round(y, 2), round(v, 1), round(cte, 2), near))
            if v < 1.0 and t > 5:
                stall_t = stall_t or now
                if now - stall_t > 4.0:
                    diverged = True; break
            else:
                stall_t = None
            if cte > 12: diverged = True; break
            if s_arr[near] >= self.seg: complete_t = t; break
            rate.sleep()
        stop = self._CtrlCmd(); stop.longlCmdType = 1; stop.brake = 1.0
        for _ in range(4):
            self.pub.publish(stop); rospy.sleep(0.02)
        completed = complete_t is not None
        penalty = 5.0 * int(offtrack // 3.0) + (10.0 if overspeed > 0 else 0) + 10.0 * int(overspeed // 3.0)
        m = dict(completed=completed, diverged=diverged, progress_s=round(max_s, 1),
                 time_s=round(complete_t, 2) if completed else self.timeout,
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
