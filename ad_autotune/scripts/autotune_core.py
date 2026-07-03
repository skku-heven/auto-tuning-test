#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""
ad_autotune — offline auto-tuning harness for the ad_tracker Stanley controller.

WHY THIS EXISTS
---------------
The real tuning loop is:  set params -> drive in MORAI -> record -> score -> repeat.
MORAI live data needs a UI "Connect" press, so this module proves the *workflow*
without the simulator by swapping the "plant" for a kinematic bicycle model.

Everything here mirrors src/ad_tracker/src/gps_tracker.cpp so that tuned params
(lookahead / target_velocity_kph / gain_k / pid_*) drop straight into
ad_tracker.launch.  To go live, replace `BicyclePlant` with the ROS bridge
(see run_live_tuning.py) — the controller, scoring and optimizer are unchanged.

Pure stdlib (math only). No numpy/scipy/ros required for the offline run.
"""
from __future__ import annotations

import math
import os

# ----------------------------------------------------------------------------
# Vehicle / competition constants (ioniq5, from 2025 규정 + sensor json)
# ----------------------------------------------------------------------------
WHEELBASE_M = 3.0          # ioniq5 wheelbase
MAX_STEER_RAD = 40.0 * math.pi / 180.0   # 0.698 rad, max front wheel angle
SPEED_LIMIT_KPH = 50.0     # 규정: 50kph 초과시 감점
A_MAX = 3.0                # m/s^2 at accel=1.0  (approx powertrain)
B_MAX = 6.0                # m/s^2 at brake=1.0
DRAG = 0.2                 # m/s^2 passive decel
DT = 0.05                  # 20 Hz control loop (matches ad_tracker rate)
TIME_LIMIT_S = 180.0       # give up after this (규정 본선 15분이지만 작은 트랙)
OFFTRACK_M = 1.5           # lateral error treated as 경로이탈 (X is undisclosed; pick)
OFFTRACK_HARD_M = 2.0      # beyond this = left the lane / off-road (~lane half-width)
HARD_FAIL_S = 0.7          # sustained off-road time that voids 완주 (코스이탈 실격)


# ----------------------------------------------------------------------------
# Synthetic track — a closed oval (straights + semicircle ends).
# Tests both speed (straights) and steering (curves), so gain/lookahead matter.
# ----------------------------------------------------------------------------
def generate_oval_track(straight=40.0, radius=15.0, spacing=0.5):
    pts = []
    # bottom straight: x from -straight/2..+straight/2 at y=-radius
    n = int(straight / spacing)
    for i in range(n):
        pts.append((-straight / 2 + i * spacing, -radius))
    # right semicircle: center (straight/2, 0), angle -90..+90
    arc = math.pi * radius
    m = int(arc / spacing)
    for i in range(m):
        a = -math.pi / 2 + math.pi * i / m
        pts.append((straight / 2 + radius * math.cos(a), radius * math.sin(a)))
    # top straight: x from +straight/2..-straight/2 at y=+radius
    for i in range(n):
        pts.append((straight / 2 - i * spacing, radius))
    # left semicircle: center (-straight/2, 0), angle 90..270
    for i in range(m):
        a = math.pi / 2 + math.pi * i / m
        pts.append((-straight / 2 + radius * math.cos(a), radius * math.sin(a)))
    return pts


def generate_tight_track(straight=18.0, radius=6.0, spacing=0.4):
    """Tighter closed loop — small corner radius. At high speed the bicycle
    model can't turn sharp enough, so the tuner is FORCED to back off velocity
    (proves it adapts instead of just maxing speed)."""
    return generate_oval_track(straight=straight, radius=radius, spacing=spacing)


TRACKS = {
    "oval": generate_oval_track,        # easy: gentle curves
    "tight": generate_tight_track,      # hard: sharp corners
}


def ascii_plot(track, traj=None, w=64, h=22):
    """Tiny ASCII view: '.' track, '*' driven trajectory."""
    xs = [p[0] for p in track]
    ys = [p[1] for p in track]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    sx = (w - 1) / max(maxx - minx, 1e-6)
    sy = (h - 1) / max(maxy - miny, 1e-6)
    grid = [[" "] * w for _ in range(h)]
    for x, y in track:
        c = int((x - minx) * sx)
        r = h - 1 - int((y - miny) * sy)
        grid[r][c] = "."
    if traj:
        for s in traj:
            x, y = s[1], s[2]
            c = int((x - minx) * sx)
            r = h - 1 - int((y - miny) * sy)
            if 0 <= r < h and 0 <= c < w:
                grid[r][c] = "*"
    return "\n".join("".join(row) for row in grid)


def write_track_csv(pts, path):
    # 2 columns (x,y) ONLY — avoids ad_tracker bug A (col3 read as heading).
    with open(path, "w") as f:
        f.write("# x,y  synthetic oval track for ad_autotune\n")
        for x, y in pts:
            f.write(f"{x:.4f},{y:.4f}\n")


def load_track_csv(path):
    pts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p for p in line.split(",") if p != ""]
            if len(parts) >= 2:
                pts.append((float(parts[0]), float(parts[1])))
    return pts


# ----------------------------------------------------------------------------
# Stanley controller — faithful mirror of gps_tracker.cpp (bug-free path_theta).
# ----------------------------------------------------------------------------
def _normalize(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class StanleyController:
    def __init__(self, track, lookahead, target_velocity_kph, gain_k,
                 pid_kp, pid_ki, pid_kd):
        self.wp = track
        self.lookahead = lookahead
        self.target_v_kph = target_velocity_kph
        self.k = gain_k
        self.kp, self.ki, self.kd = pid_kp, pid_ki, pid_kd
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_t = None

    def find_target(self, x, y):
        n = len(self.wp)
        # nearest waypoint
        best_i, best_d = 0, float("inf")
        for i, (wx, wy) in enumerate(self.wp):
            d = math.hypot(wx - x, wy - y)
            if d < best_d:
                best_i, best_d = i, d
        # advance by lookahead
        ti, td = best_i, best_d
        for _ in range(n):
            if td >= self.lookahead:
                break
            ti = (ti + 1) % n
            wx, wy = self.wp[ti]
            td = math.hypot(wx - x, wy - y)
            if ti == best_i:
                break
        return ti, td, best_i

    def control(self, x, y, theta_rad, v_kph, t):
        ti, td, nearest = self.find_target(x, y)
        n = len(self.wp)
        cx, cy = self.wp[ti]
        nx, ny = self.wp[(ti + 1) % n]
        path_theta = math.atan2(ny - cy, nx - cx)          # bug-free heading
        heading_err = _normalize(path_theta - theta_rad)
        target_theta = math.atan2(cy - y, cx - x) - path_theta
        cte = td * math.sin(target_theta)
        v_mps = max(self.target_v_kph / 3.6, 0.1)
        steer = heading_err + math.atan2(self.k * cte, v_mps)
        steer = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, steer))
        # speed PID
        err = self.target_v_kph - v_kph
        accel = self.kp * err
        if self.prev_t is not None:
            dt = max(t - self.prev_t, 1e-3)
            self.integral = max(-100.0, min(100.0, self.integral + err * dt))
            accel += self.ki * self.integral + self.kd * (err - self.prev_err) / dt
        self.prev_err, self.prev_t = err, t
        throttle = max(0.0, min(1.0, accel))
        brake = max(0.0, min(1.0, -accel)) if err < -1.0 else 0.0
        return steer, throttle, brake, cte, nearest


# ----------------------------------------------------------------------------
# Plant — kinematic bicycle model (rear-axle reference).
# Swap THIS for the MORAI ROS bridge to go live; nothing else changes.
# ----------------------------------------------------------------------------
class BicyclePlant:
    def __init__(self, x, y, theta, v=0.0):
        self.x, self.y, self.theta, self.v = x, y, theta, v

    def step(self, steer, throttle, brake, dt):
        a = throttle * A_MAX - brake * B_MAX - DRAG
        self.v = max(0.0, self.v + a * dt)             # m/s, no reverse
        self.theta += self.v / WHEELBASE_M * math.tan(steer) * dt
        self.theta = _normalize(self.theta)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt


def _path_lateral_error(track, x, y):
    """min distance from (x,y) to the track polyline (segment-aware)."""
    best = float("inf")
    n = len(track)
    for i in range(n):
        ax, ay = track[i]
        bx, by = track[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-9:
            d = math.hypot(x - ax, y - ay)
        else:
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
            px, py = ax + t * dx, ay + t * dy
            d = math.hypot(x - px, y - py)
        if d < best:
            best = d
    return best


# ----------------------------------------------------------------------------
# Scoring — competition rules (계산 G).  Operates on a recorded run so that the
# OFFLINE plant and the LIVE MORAI run are scored by the exact same code.
#   samples: list of (t, x, y, v_kph)
# ----------------------------------------------------------------------------
def score_run(track, samples):
    n = len(track)
    laps_idx_seen = 0
    prev_nearest = None
    offtrack_time = speeding_time = hard_offtrack_time = 0.0
    ever_sped = False
    max_cte = sum_cte = 0.0
    diverged = off_road = False
    complete_t = None

    def nearest_index(x, y):
        bi, bd = 0, float("inf")
        for i, (wx, wy) in enumerate(track):
            d = (wx - x) ** 2 + (wy - y) ** 2
            if d < bd:
                bi, bd = i, d
        return bi

    for j in range(len(samples)):
        t, x, y, v_kph = samples[j]
        cte = _path_lateral_error(track, x, y)
        near = nearest_index(x, y)
        if prev_nearest is not None:
            adv = (near - prev_nearest) % n
            if adv < n // 2:
                laps_idx_seen += adv
        prev_nearest = near

        dt = (samples[j][0] - samples[j - 1][0]) if j > 0 else DT
        if cte > OFFTRACK_M:
            offtrack_time += dt
        if cte > OFFTRACK_HARD_M:
            hard_offtrack_time += dt
        if v_kph > SPEED_LIMIT_KPH:
            speeding_time += dt
            ever_sped = True
        max_cte = max(max_cte, cte)
        sum_cte += cte

        if cte > 12.0:
            diverged = True
            break
        if hard_offtrack_time > HARD_FAIL_S:    # 코스 이탈 = 완주 무효
            off_road = True
            break
        if laps_idx_seen >= n and complete_t is None:
            complete_t = t
            break

    completed = (complete_t is not None) and not off_road
    penalty = 5.0 * int(offtrack_time // 3.0)        # 경로이탈 3초누적 -5
    if ever_sped:
        penalty += 10.0                               # 속도초과 즉시 -10
    penalty += 10.0 * int(speeding_time // 3.0)       # +3초누적당 -10
    driving_score = max(0.0, 100.0 - penalty)
    end_t = complete_t if completed else (samples[-1][0] if samples else 0.0)

    return {
        "completed": completed,
        "off_road": off_road,
        "driving_score": driving_score,
        "time_s": round(end_t, 2) if completed else TIME_LIMIT_S,
        "progress": round(min(1.0, laps_idx_seen / n), 3),
        "penalty": penalty,
        "offtrack_time": round(offtrack_time, 2),
        "speeding_time": round(speeding_time, 2),
        "max_cte": round(max_cte, 2),
        "mean_cte": round(sum_cte / max(len(samples), 1), 3),
        "diverged": diverged,
    }


# ----------------------------------------------------------------------------
# Offline episode: drive the bicycle plant with the controller, then score_run.
# ----------------------------------------------------------------------------
def simulate(track, params, record=False):
    ctrl = StanleyController(track, **params)
    x0, y0 = track[0]
    x1, y1 = track[1]
    theta0 = math.atan2(y1 - y0, x1 - x0)
    plant = BicyclePlant(x0, y0, theta0)
    n = len(track)

    t = 0.0
    samples = []
    laps = 0
    prev_near = None
    while t < TIME_LIMIT_S:
        v_kph = plant.v * 3.6
        steer, thr, brk, _cte, near = ctrl.control(plant.x, plant.y, plant.theta, v_kph, t)
        samples.append((t, plant.x, plant.y, v_kph))
        if prev_near is not None:
            adv = (near - prev_near) % n
            if adv < n // 2:
                laps += adv
        prev_near = near
        if _path_lateral_error(track, plant.x, plant.y) > 12.0 or laps >= n:
            break
        plant.step(steer, thr, brk, DT)
        t += DT

    result = score_run(track, samples)
    if record:
        result["traj"] = [(round(s[0], 2), round(s[1], 2), round(s[2], 2),
                           round(s[3], 1)) for s in samples]
    return result


def objective(r):
    """Single scalar, lexicographic: 완주 > 주행점수 > 시간 (규정 순위 기준)."""
    if not r["completed"]:
        # not finished: reward progress so the optimizer first learns to complete
        return -1e6 + r["progress"] * 1e5 - r["mean_cte"] * 10
    return 1e6 + r["driving_score"] * 1000.0 - r["time_s"]


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    track_path = os.path.join(here, "..", "paths", "oval_track.csv")
    track = generate_oval_track()
    write_track_csv(track, track_path)
    print(f"track: {len(track)} waypoints -> {track_path}")
    # smoke test with default ad_tracker.launch params
    defaults = dict(lookahead=3.0, target_velocity_kph=20.0, gain_k=0.5,
                    pid_kp=0.3, pid_ki=0.0, pid_kd=0.01)
    r = simulate(track, defaults, record=True)
    print("default params result:", {k: v for k, v in r.items() if k != "traj"})
    print("objective:", round(objective(r), 1))
