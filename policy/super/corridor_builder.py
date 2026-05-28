"""Safe corridor builder (Python adaptation of SUPER corridor stage)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

PathXY = List[Tuple[float, float]]


@dataclass
class CorridorSegment:
    center: Tuple[float, float]
    radius: float
    length: float


@dataclass
class Corridor:
    segments: List[CorridorSegment]


class CorridorBuilder:
    """Build 2D circular corridor segments along guide path."""

    def __init__(self, robot_radius: float, margin: float = 0.4) -> None:
        self.robot_radius = max(robot_radius, 0.05)
        self.margin = max(margin, 0.05)

    def build(self, path: PathXY) -> Corridor:
        if len(path) <= 1:
            p = path[0] if path else (0.0, 0.0)
            return Corridor([CorridorSegment(center=p, radius=self.robot_radius + self.margin, length=0.0)])
        segs: List[CorridorSegment] = []
        rad = self.robot_radius + self.margin
        for i in range(len(path) - 1):
            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            cx = 0.5 * (x0 + x1)
            cy = 0.5 * (y0 + y1)
            length = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            segs.append(CorridorSegment(center=(cx, cy), radius=rad, length=length))
        return Corridor(segs)
