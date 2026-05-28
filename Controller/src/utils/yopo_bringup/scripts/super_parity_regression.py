#!/usr/bin/env python3
"""Run SUPER-compat regression and report parity metrics."""

from __future__ import annotations

import argparse
import ast
import json
import statistics
import subprocess
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class GoalRun:
    goal: Tuple[float, float, float]
    reached: bool
    time_to_goal: float
    emerg_delta: float
    min_clearance_proxy: float
    hover_stability: float


def run_cmd(command: str, timeout_sec: float = 8.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", "-lc", command],
        capture_output=True,
        text=True,
        timeout=timeout_sec,
    )


def read_diag_metrics() -> Optional[List[float]]:
    proc = run_cmd("timeout 3 ros2 topic echo /super_compat/diag/metrics --once", timeout_sec=5.0)
    if proc.returncode != 0:
        return None
    for line in proc.stdout.splitlines():
        line = line.strip()
        if line.startswith("data:"):
            try:
                parsed = ast.literal_eval(line.split("data:", 1)[1].strip())
            except Exception:
                return None
            if isinstance(parsed, list):
                return [float(x) for x in parsed]
    return None


def publish_goal(x: float, y: float, z: float) -> bool:
    cmd = (
        "timeout 5 ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped "
        f"\"{{header: {{frame_id: world}}, pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1.0}}}}}}\""
    )
    proc = run_cmd(cmd, timeout_sec=7.0)
    return proc.returncode == 0


def parse_goals(raw: str, height: float) -> List[Tuple[float, float, float]]:
    out: List[Tuple[float, float, float]] = []
    for token in raw.split(";"):
        token = token.strip()
        if not token:
            continue
        parts = [p.strip() for p in token.split(",")]
        if len(parts) < 2:
            continue
        x = float(parts[0])
        y = float(parts[1])
        z = float(parts[2]) if len(parts) >= 3 else height
        out.append((x, y, z))
    return out


def run_one_goal(goal: Tuple[float, float, float], timeout_sec: float, hold_frames: int) -> GoalRun:
    if not publish_goal(*goal):
        return GoalRun(goal=goal, reached=False, time_to_goal=timeout_sec, emerg_delta=0.0, min_clearance_proxy=0.0, hover_stability=0.0)

    start = time.time()
    deadline = start + timeout_sec
    hold = 0
    first_emer = None
    last_emer = None
    min_clear = 1e9
    stable_samples = 0
    total_samples = 0

    while time.time() < deadline:
        metrics = read_diag_metrics()
        if not metrics or len(metrics) < 10:
            continue
        d_goal, _, speed_xy = metrics[0], metrics[1], metrics[2]
        emer = metrics[4]
        latched = metrics[7] > 0.5
        clear_proxy = metrics[9]
        first_emer = emer if first_emer is None else first_emer
        last_emer = emer
        min_clear = min(min_clear, clear_proxy)
        total_samples += 1
        stable = d_goal < 0.35 and speed_xy < 0.35
        if stable:
            stable_samples += 1
        if latched or stable:
            hold += 1
        else:
            hold = 0
        if hold >= hold_frames:
            dt = time.time() - start
            emer_delta = (last_emer - first_emer) if first_emer is not None and last_emer is not None else 0.0
            return GoalRun(
                goal=goal,
                reached=True,
                time_to_goal=dt,
                emerg_delta=emer_delta,
                min_clearance_proxy=min_clear if min_clear < 1e8 else 0.0,
                hover_stability=(stable_samples / max(total_samples, 1)),
            )

    emer_delta = (last_emer - first_emer) if first_emer is not None and last_emer is not None else 0.0
    return GoalRun(
        goal=goal,
        reached=False,
        time_to_goal=timeout_sec,
        emerg_delta=emer_delta,
        min_clearance_proxy=min_clear if min_clear < 1e8 else 0.0,
        hover_stability=(stable_samples / max(total_samples, 1)),
    )


def summarize(runs: List[GoalRun], wall_time_sec: float) -> dict:
    reached = [r for r in runs if r.reached]
    t_all = [r.time_to_goal for r in runs]
    emers = [r.emerg_delta for r in runs]
    clear = [r.min_clearance_proxy for r in runs]
    hover = [r.hover_stability for r in runs]
    return {
        "goal_reach_rate": len(reached) / max(len(runs), 1),
        "time_to_goal_mean": statistics.mean(t_all) if t_all else 0.0,
        "time_to_goal_median": statistics.median(t_all) if t_all else 0.0,
        "EMER_count_per_min": (sum(emers) * 60.0 / max(wall_time_sec, 1e-6)),
        "min_clearance_proxy": min(clear) if clear else 0.0,
        "hover_stability": statistics.mean(hover) if hover else 0.0,
        "runs": [
            {
                "goal": [r.goal[0], r.goal[1], r.goal[2]],
                "reached": r.reached,
                "time_to_goal": r.time_to_goal,
                "EMER_delta": r.emerg_delta,
                "min_clearance_proxy": r.min_clearance_proxy,
                "hover_stability": r.hover_stability,
            }
            for r in runs
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="SUPER parity regression metrics.")
    parser.add_argument(
        "--goals",
        default="8,2,2;6,-3,2;10,0,2",
        help="Semicolon-separated goals: x,y[,z];x,y[,z]",
    )
    parser.add_argument("--goal-timeout-sec", type=float, default=80.0)
    parser.add_argument("--hold-frames", type=int, default=5)
    parser.add_argument("--default-height", type=float, default=2.0)
    parser.add_argument("--output", type=str, default="")
    args = parser.parse_args()

    goals = parse_goals(args.goals, args.default_height)
    if not goals:
        raise SystemExit("No valid goals parsed.")

    print(f"[regression] running {len(goals)} goals...")
    runs: List[GoalRun] = []
    t0 = time.time()
    for goal in goals:
        print(f"[regression] goal={goal}")
        runs.append(run_one_goal(goal, timeout_sec=args.goal_timeout_sec, hold_frames=args.hold_frames))
    report = summarize(runs, wall_time_sec=time.time() - t0)
    payload = json.dumps(report, ensure_ascii=True, indent=2)
    print(payload)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(payload + "\n")
        print(f"[regression] report written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
