#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""Calculate driving metrics from a rosbag and a global path CSV."""

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Iterable, List, Tuple

import rosbag


COMPETITION_TOPIC = "/Competition_topic"
COLLISION_TOPIC = "/CollisionData"
POSE_TOPIC = "/ad_pose_parser/pose"


Waypoint = Tuple[float, float, float]


def _heading_to_rad(value: float) -> float:
    if abs(value) > 2.0 * math.pi:
        return math.radians(value)
    return value


def load_csv(path: Path) -> List[Waypoint]:
    waypoints: List[Waypoint] = []
    with path.open() as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            try:
                x = float(row[0])
                y = float(row[1])
            except (ValueError, IndexError):
                continue
            heading = _heading_to_rad(float(row[2])) if len(row) >= 3 and row[2].strip() else 0.0
            waypoints.append((x, y, heading))

    if len(waypoints) < 2:
        raise ValueError(f"CSV needs at least two waypoints: {path}")
    return waypoints


def nearest_waypoint(x: float, y: float, waypoints: Iterable[Waypoint]) -> Tuple[int, float]:
    best_i = 0
    best_d = float("inf")
    for i, (wx, wy, _) in enumerate(waypoints):
        d = math.hypot(x - wx, y - wy)
        if d < best_d:
            best_i = i
            best_d = d
    return best_i, best_d


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def path_heading(index: int, waypoints: List[Waypoint]) -> float:
    x, y, heading = waypoints[index]
    if abs(heading) > 1e-9:
        return heading
    if index + 1 < len(waypoints):
        nx, ny, _ = waypoints[index + 1]
        return math.atan2(ny - y, nx - x)
    px, py, _ = waypoints[index - 1]
    return math.atan2(y - py, x - px)


def stats(values: List[float]) -> Tuple[float, float, float]:
    if not values:
        return 0.0, 0.0, 0.0
    mean = sum(values) / len(values)
    var = sum((value - mean) ** 2 for value in values) / len(values)
    return mean, math.sqrt(var), max(values)


def collision_event_count(msg) -> int:
    objects = []
    if hasattr(msg, "collision_object"):
        objects.extend(msg.collision_object)
    if hasattr(msg, "collision_objecta"):
        objects.extend(msg.collision_objecta)
    return 1 if objects else 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    waypoints = load_csv(args.csv)
    speeds: List[float] = []
    cross_errors: List[float] = []
    heading_errors: List[float] = []
    collision_events = 0
    collision_messages = 0
    max_wpt_idx = 0

    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, msg, _ in bag.read_messages(topics=[COMPETITION_TOPIC, COLLISION_TOPIC, POSE_TOPIC]):
            if topic == COMPETITION_TOPIC:
                speeds.append(float(msg.velocity.x))
            elif topic == COLLISION_TOPIC:
                collision_messages += 1
                collision_events += collision_event_count(msg)
            elif topic == POSE_TOPIC:
                pose_theta = _heading_to_rad(float(msg.theta))
                idx, dist = nearest_waypoint(float(msg.x), float(msg.y), waypoints)
                cross_errors.append(dist)
                heading_errors.append(abs(normalize_angle(pose_theta - path_heading(idx, waypoints))))
                max_wpt_idx = max(max_wpt_idx, idx)

    sp_mean, sp_std, sp_max = stats(speeds)
    _, _, cross_max = stats(cross_errors)
    heading_mean, _, heading_max = stats(heading_errors)
    cross_rmse = math.sqrt(sum(value * value for value in cross_errors) / len(cross_errors)) if cross_errors else 0.0
    completion_pct = (max_wpt_idx / max(len(waypoints) - 1, 1)) * 100.0

    out_lines = [
        "# Driving Metrics",
        "",
        f"- bag: `{args.bag}`",
        f"- csv: `{args.csv}` ({len(waypoints)} waypoints)",
        "",
        "## Summary",
        "",
        "| Metric | Value |",
        "|---|---|",
        f"| 1. 충돌 횟수 | **{collision_events}** events ({collision_messages} CollisionData messages) |",
        f"| 2. 평균 속도 (m/s) | **{sp_mean:.3f}** ± {sp_std:.3f} (max {sp_max:.3f}) |",
        f"| 3. Cross-track RMSE (m) | **{cross_rmse:.3f}** (max {cross_max:.3f}) |",
        f"| 4. Heading error mean (rad) | **{heading_mean:.4f}** (max {heading_max:.4f}) |",
        f"| 5. 완주율 (%) | **{completion_pct:.1f}** ({max_wpt_idx}/{len(waypoints) - 1}) |",
        "",
    ]

    output = "\n".join(out_lines)
    print(output)
    if args.out:
        args.out.write_text(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
