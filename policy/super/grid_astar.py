"""Grid A* with horizon fallback and partial-path return."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, List, Set, Tuple

GridCell = Tuple[int, int]


class SearchStatus(Enum):
    REACH_GOAL = auto()
    REACH_HORIZON = auto()
    NO_PATH = auto()


@dataclass
class SearchResult:
    path: List[GridCell]
    status: SearchStatus


class GridAStar:
    """2D A* matching SUPER behavior with horizon fallback."""

    NEIGH = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    AXIAL = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    def search(
        self,
        start: GridCell,
        goal: GridCell,
        blocked: Set[GridCell],
        max_range_cells: int,
    ) -> SearchResult:
        if start in blocked:
            blocked = set(blocked)
            blocked.discard(start)
        if goal in blocked:
            return SearchResult([], SearchStatus.NO_PATH)

        def in_bound(c: GridCell) -> bool:
            return (
                abs(c[0] - start[0]) <= max_range_cells
                and abs(c[1] - start[1]) <= max_range_cells
            )

        openset: List[Tuple[float, GridCell]] = [(0.0, start)]
        came_from: Dict[GridCell, GridCell] = {}
        g_score: Dict[GridCell, float] = {start: 0.0}
        visited: Set[GridCell] = set()
        best_node = start
        best_h = math.hypot(goal[0] - start[0], goal[1] - start[1])

        while openset:
            _, cur = heapq.heappop(openset)
            if cur in visited:
                continue
            visited.add(cur)

            cur_h = math.hypot(goal[0] - cur[0], goal[1] - cur[1])
            if cur_h < best_h:
                best_h = cur_h
                best_node = cur

            if cur == goal:
                return SearchResult(self._reconstruct(came_from, cur), SearchStatus.REACH_GOAL)

            for dx, dy in self.NEIGH:
                nxt = (cur[0] + dx, cur[1] + dy)
                if nxt in blocked or not in_bound(nxt):
                    continue
                step = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
                tentative = g_score[cur] + step
                if tentative < g_score.get(nxt, 1e18):
                    came_from[nxt] = cur
                    g_score[nxt] = tentative
                    h = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heapq.heappush(openset, (tentative + h, nxt))

        if best_node != start:
            return SearchResult(self._reconstruct(came_from, best_node), SearchStatus.REACH_HORIZON)
        return SearchResult([], SearchStatus.NO_PATH)

    def escape_search(
        self,
        start: GridCell,
        blocked: Set[GridCell],
        max_range_cells: int,
    ) -> List[GridCell]:
        """Find shortest escape path to nearest unblocked cell."""
        if start not in blocked:
            return [start]
        from collections import deque

        q = deque([start])
        parent: Dict[GridCell, GridCell] = {}
        visited: Set[GridCell] = {start}

        def in_bound(c: GridCell) -> bool:
            return (
                abs(c[0] - start[0]) <= max_range_cells
                and abs(c[1] - start[1]) <= max_range_cells
            )

        while q:
            cur = q.popleft()
            for dx, dy in self.AXIAL:
                nxt = (cur[0] + dx, cur[1] + dy)
                if nxt in visited or not in_bound(nxt):
                    continue
                visited.add(nxt)
                parent[nxt] = cur
                if nxt not in blocked:
                    return self._reconstruct(parent, nxt)
                q.append(nxt)
        return []

    @staticmethod
    def _reconstruct(came_from: Dict[GridCell, GridCell], cur: GridCell) -> List[GridCell]:
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path
