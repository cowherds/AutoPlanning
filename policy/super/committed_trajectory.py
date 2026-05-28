"""Time-parameterized committed trajectory for command sampling."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class TrajectoryState:
    x: float
    y: float
    vx: float
    vy: float
    ax: float
    ay: float


class CommittedTrajectory:
    def __init__(self, points: List[Tuple[float, float]], times: List[float]) -> None:
        self.points = points
        self.times = times
        self.total_duration = times[-1] if times else 0.0

    def empty(self) -> bool:
        return len(self.points) <= 1 or len(self.times) <= 1

    def sample(self, t: float) -> TrajectoryState:
        if self.empty():
            x, y = (self.points[0] if self.points else (0.0, 0.0))
            return TrajectoryState(x=x, y=y, vx=0.0, vy=0.0, ax=0.0, ay=0.0)
        if t <= 0.0:
            x0, y0 = self.points[0]
            x1, y1 = self.points[1]
            dt = max(self.times[1] - self.times[0], 1e-6)
            return TrajectoryState(x=x0, y=y0, vx=(x1 - x0) / dt, vy=(y1 - y0) / dt, ax=0.0, ay=0.0)
        if t >= self.total_duration:
            x, y = self.points[-1]
            return TrajectoryState(x=x, y=y, vx=0.0, vy=0.0, ax=0.0, ay=0.0)

        i = 0
        while i + 1 < len(self.times) and self.times[i + 1] < t:
            i += 1
        t0, t1 = self.times[i], self.times[i + 1]
        x0, y0 = self.points[i]
        x1, y1 = self.points[i + 1]
        dt = max(t1 - t0, 1e-6)
        r = (t - t0) / dt
        x = x0 + r * (x1 - x0)
        y = y0 + r * (y1 - y0)
        vx = (x1 - x0) / dt
        vy = (y1 - y0) / dt
        return TrajectoryState(x=x, y=y, vx=vx, vy=vy, ax=0.0, ay=0.0)
