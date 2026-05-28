"""Backup trajectory optimizer (safe stop adaptation)."""

from __future__ import annotations

from typing import Optional

from committed_trajectory import CommittedTrajectory, TrajectoryState


class BackupTrajectoryOptimizer:
    def __init__(self, stop_horizon_sec: float = 1.2) -> None:
        self.stop_horizon_sec = max(stop_horizon_sec, 0.2)

    def generate(self, ref: Optional[CommittedTrajectory], now_state: TrajectoryState) -> CommittedTrajectory:
        if ref is not None and not ref.empty():
            return ref
        x0, y0 = now_state.x, now_state.y
        x1 = x0 + now_state.vx * 0.2
        y1 = y0 + now_state.vy * 0.2
        return CommittedTrajectory([(x0, y0), (x1, y1), (x1, y1)], [0.0, self.stop_horizon_sec * 0.5, self.stop_horizon_sec])
