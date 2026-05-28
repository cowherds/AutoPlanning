"""Yaw trajectory optimizer synchronized with committed position trajectory."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from committed_trajectory import CommittedTrajectory


@dataclass
class YawPoint:
    t: float
    yaw: float


class YawTrajectoryOptimizer:
    def optimize(self, traj: CommittedTrajectory) -> List[YawPoint]:
        if traj.empty():
            return [YawPoint(t=0.0, yaw=0.0)]
        out: List[YawPoint] = []
        for i, t in enumerate(traj.times):
            if i + 1 < len(traj.points):
                x0, y0 = traj.points[i]
                x1, y1 = traj.points[i + 1]
                yaw = math.atan2(y1 - y0, x1 - x0)
            elif out:
                yaw = out[-1].yaw
            else:
                yaw = 0.0
            out.append(YawPoint(t=t, yaw=yaw))
        return out
