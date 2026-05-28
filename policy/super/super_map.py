"""Rolling 2D occupancy map (SUPER ROG-map lite for fixed-altitude flight)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Set, Tuple

GridCell = Tuple[int, int]


@dataclass
class MapConfig:
    resolution: float = 0.35
    planning_horizon: float = 16.0
    obstacle_inflation: float = 0.7
    map_decay_sec: float = 2.0
    start_clearance: float = 1.0
    flight_height: float = 3.0
    z_band: float = 1.5
    goal_search_radius: float = 3.0


class RollingOccupancyMap:
    """Time-decayed occupancy grid updated from LiDAR hits at cruise height."""

    def __init__(self, cfg: MapConfig) -> None:
        self.cfg = cfg
        self.occupancy_ts: Dict[GridCell, float] = {}

    def world_to_grid(self, x: float, y: float) -> GridCell:
        r = self.cfg.resolution
        return int(round(x / r)), int(round(y / r))

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        r = self.cfg.resolution
        return gx * r, gy * r

    def ingest_world_points(self, points: Iterable[Tuple[float, float, float]], now_t: float) -> None:
        z_center = self.cfg.flight_height
        z_band = self.cfg.z_band
        for wx, wy, wz in points:
            if abs(wz - z_center) > z_band:
                continue
            self.occupancy_ts[self.world_to_grid(wx, wy)] = now_t
        self.purge_stale(now_t)

    def purge_stale(self, now_t: float) -> None:
        stale = [
            k for k, t in self.occupancy_ts.items() if now_t - t > self.cfg.map_decay_sec
        ]
        for k in stale:
            del self.occupancy_ts[k]

    def cells_in_horizon(self, center_xy: Tuple[float, float]) -> Set[GridCell]:
        cx, cy = self.world_to_grid(center_xy[0], center_xy[1])
        max_cell = int(self.cfg.planning_horizon / self.cfg.resolution)
        out: Set[GridCell] = set()
        for cell in self.occupancy_ts:
            if abs(cell[0] - cx) <= max_cell and abs(cell[1] - cy) <= max_cell:
                out.add(cell)
        return out

    def inflate(self, obstacles: Set[GridCell]) -> Set[GridCell]:
        inf_cells = int(math.ceil(self.cfg.obstacle_inflation / self.cfg.resolution))
        out: Set[GridCell] = set()
        for ox, oy in obstacles:
            for dx in range(-inf_cells, inf_cells + 1):
                for dy in range(-inf_cells, inf_cells + 1):
                    if dx * dx + dy * dy <= inf_cells * inf_cells:
                        out.add((ox + dx, oy + dy))
        return out

    def clear_around(self, blocked: Set[GridCell], center: GridCell) -> Set[GridCell]:
        clear_cells = int(math.ceil(self.cfg.start_clearance / self.cfg.resolution))
        out = set(blocked)
        for dx in range(-clear_cells, clear_cells + 1):
            for dy in range(-clear_cells, clear_cells + 1):
                if dx * dx + dy * dy <= clear_cells * clear_cells:
                    out.discard((center[0] + dx, center[1] + dy))
        return out

    def is_blocked_xy(self, x: float, y: float, blocked: Set[GridCell]) -> bool:
        return self.world_to_grid(x, y) in blocked

    def line_cells(self, a: GridCell, b: GridCell) -> List[GridCell]:
        """Integer grid traversal between two cells."""
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        out: List[GridCell] = []
        while True:
            out.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return out

    def segment_is_free(
        self,
        p0_xy: Tuple[float, float],
        p1_xy: Tuple[float, float],
        blocked: Set[GridCell],
    ) -> bool:
        c0 = self.world_to_grid(p0_xy[0], p0_xy[1])
        c1 = self.world_to_grid(p1_xy[0], p1_xy[1])
        for c in self.line_cells(c0, c1):
            if c in blocked:
                return False
        return True

    def nearest_free_xy(
        self,
        x: float,
        y: float,
        blocked: Set[GridCell],
        search_radius_m: Optional[float] = None,
    ) -> Optional[Tuple[float, float]]:
        """SUPER-style: snap goal/start to nearest non-occupied cell."""
        radius = search_radius_m if search_radius_m is not None else self.cfg.goal_search_radius
        gx, gy = self.world_to_grid(x, y)
        if (gx, gy) not in blocked:
            return x, y
        max_ring = max(1, int(math.ceil(radius / self.cfg.resolution)))
        best: Optional[Tuple[float, float]] = None
        best_d = 1e18
        for ring in range(1, max_ring + 1):
            for dx in range(-ring, ring + 1):
                for dy in range(-ring, ring + 1):
                    if max(abs(dx), abs(dy)) != ring:
                        continue
                    cell = (gx + dx, gy + dy)
                    if cell in blocked:
                        continue
                    wx, wy = self.grid_to_world(cell[0], cell[1])
                    d = (wx - x) ** 2 + (wy - y) ** 2
                    if d < best_d:
                        best_d = d
                        best = (wx, wy)
            if best is not None:
                return best
        return None
