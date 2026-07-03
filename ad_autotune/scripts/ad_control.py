# [transferable-to-heven_ad]
"""
ad_control.py — governed Stanley controller (shared by live_drive & auto_tune_live).

Corrected per review (2026-07-02):
  - arc-length lookahead (heading preview), NOT euclidean target
  - signed cross-track error at the NEAREST path point (clean Stanley)
  - Stanley denominator uses MEASURED speed + softening: atan2(k*cte, k_soft + v_mps)
  - smooth speed profile: fwd/back pass (decel/accel limits), NOT sliding-window min
  - output slew rate-limit on steering
  - windowed monotonic nearest index
Sign convention (verify live: car must converge to path):
  cte > 0 when ego is to the RIGHT of the path -> steer left (positive front_steer).
"""
import math

MAX_STEER = 40.0 * math.pi / 180.0        # rad
STEER_RATE = 120.0 * math.pi / 180.0      # rad/s slew limit on steering output


def _norm(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a


def build_profile(track, a_lat=1.5, a_decel=2.5, a_accel=2.0, v_min_kph=5.0,
                  v_max_kph=60.0, curv_smooth=5):
    """Precompute arc-length s and a smooth speed profile [kph] over the path.
    a_decel/a_accel are physical vehicle limits (m/s^2), fixed (not tuned)."""
    n = len(track)
    s = [0.0]
    for i in range(1, n):
        s.append(s[-1] + math.hypot(track[i][0]-track[i-1][0], track[i][1]-track[i-1][1]))
    kap = [0.0]*n
    for i in range(2, n-2):
        a, b, c = track[i-2], track[i], track[i+2]
        A = math.hypot(b[0]-a[0], b[1]-a[1]); B = math.hypot(c[0]-b[0], c[1]-b[1]); C = math.hypot(c[0]-a[0], c[1]-a[1])
        if A*B*C < 1e-9: continue
        area = abs((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1]))/2
        kap[i] = 4*area/(A*B*C)
    # conservative smoothed curvature (max over window)
    ks = [max(kap[max(0, i-curv_smooth):i+curv_smooth+1] or [0.0]) for i in range(n)]
    vmin = v_min_kph/3.6; vmax = v_max_kph/3.6
    vcap = [min(vmax, max(vmin, math.sqrt(a_lat/max(k, 1e-6)))) for k in ks]   # m/s
    v = vcap[:]
    for i in range(n-2, -1, -1):                    # backward: decel limit
        ds = s[i+1]-s[i]
        v[i] = min(v[i], math.sqrt(v[i+1]**2 + 2*a_decel*ds))
    for i in range(1, n):                           # forward: accel limit
        ds = s[i]-s[i-1]
        v[i] = min(v[i], math.sqrt(v[i-1]**2 + 2*a_accel*ds))
    return s, [x*3.6 for x in v]                    # kph


class GovernedStanley:
    def __init__(self, track, s, profile_kph, lookahead=3.0, gain_k=0.8,
                 k_soft=1.0, pid_kp=0.3, pid_kd=0.05):
        self.track = track; self.n = len(track)
        self.s = s; self.prof = profile_kph
        self.lookahead = lookahead; self.gain_k = gain_k; self.k_soft = k_soft
        self.pid_kp = pid_kp; self.pid_kd = pid_kd
        self.cur = 0; self.prev_err = 0.0; self.prev_steer = 0.0

    def reset(self, x, y):
        self.cur = min(range(self.n), key=lambda i: (self.track[i][0]-x)**2 + (self.track[i][1]-y)**2)
        self.prev_err = 0.0; self.prev_steer = 0.0

    def step(self, x, y, heading_rad, v_kph, dt, target_kph):
        tr, n = self.track, self.n
        # windowed monotonic nearest
        best_i, best_d = self.cur, 1e18
        for i in range(self.cur, min(self.cur+200, n)):
            d = (tr[i][0]-x)**2 + (tr[i][1]-y)**2
            if d < best_d: best_d, best_i = d, i
        self.cur = near = best_i
        # signed cross-track at nearest segment (positive = ego right of path).
        # guard path end: use previous segment so direction is never zero-length.
        if near+1 < n:
            ax, ay = tr[near]; bx, by = tr[near+1]
        else:
            ax, ay = tr[near-1]; bx, by = tr[near]
        dx, dy = bx-ax, by-ay; L = math.hypot(dx, dy) or 1e-6
        cte = (dy*(x-ax) - dx*(y-ay)) / L
        # heading preview: path tangent at arc-length lookahead ahead of near
        pv = near
        while pv+1 < n and self.s[pv]-self.s[near] < self.lookahead:
            pv += 1
        if pv+1 < n:
            px, py = tr[pv]; qx, qy = tr[pv+1]
        else:
            px, py = tr[pv-1]; qx, qy = tr[pv]
        path_th = math.atan2(qy-py, qx-px)
        head_err = _norm(path_th - heading_rad)
        v_mps = v_kph/3.6
        steer = head_err + math.atan2(self.gain_k * cte, self.k_soft + v_mps)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        # output slew limit
        ds_max = STEER_RATE * max(dt, 1e-3)
        steer = max(self.prev_steer-ds_max, min(self.prev_steer+ds_max, steer))
        self.prev_steer = steer
        # speed from precomputed profile
        v_ref = min(target_kph, self.prof[near])
        err = v_ref - v_kph
        accel = self.pid_kp*err + self.pid_kd*(err - self.prev_err)/max(dt, 1e-3)
        self.prev_err = err
        thr = max(0.0, min(1.0, accel))
        brk = max(0.0, min(1.0, -accel)) if err < -1.0 else 0.0
        return steer, thr, brk, dict(near=near, cte=cte, head_err=head_err,
                                     v_ref=v_ref, s=self.s[near])
