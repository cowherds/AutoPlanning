"""Expected trajectory optimizer (Python MINCO-lite adaptation)."""

from __future__ import annotations

import math
from typing import List, Tuple

from committed_trajectory import CommittedTrajectory
from corridor_builder import Corridor

PathXY = List[Tuple[float, float]]


class ExpTrajectoryOptimizer:
    def __init__(self, max_speed: float = 5.0, min_seg_dt: float = 0.05) -> None:
        self.max_speed = max(max_speed, 0.5)
        self.min_seg_dt = max(min_seg_dt, 0.01)

    def optimize(self, guide_path: PathXY, corridor: Corridor) -> CommittedTrajectory:
        _ = corridor
        if len(guide_path) <= 1:
            return CommittedTrajectory(guide_path or [(0.0, 0.0)], [0.0])
        times = [0.0]
        t = 0.0
        for i in range(len(guide_path) - 1):
            x0, y0 = guide_path[i]
            x1, y1 = guide_path[i + 1]
            d = math.hypot(x1 - x0, y1 - y0)
            dt = max(self.min_seg_dt, d / self.max_speed)
            t += dt
            times.append(t)
        return CommittedTrajectory(list(guide_path), times)
