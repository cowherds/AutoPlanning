"""Compatibility wrapper for algorithm-named grid A*."""

from __future__ import annotations

from typing import List, Set, Tuple

from grid_astar import GridAStar as _GridAStarImpl

GridCell = Tuple[int, int]


class GridAStar:
    """Backward-compatible A* API that returns path list only."""

    def __init__(self) -> None:
        self._impl = _GridAStarImpl()

    def search(
        self,
        start: GridCell,
        goal: GridCell,
        blocked: Set[GridCell],
        max_range_cells: int,
    ) -> List[GridCell]:
        return self._impl.search(start, goal, blocked, max_range_cells).path
