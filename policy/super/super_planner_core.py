"""SUPER-like planner core with safe A* and path optimization."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Set, Tuple

from backup_traj_optimizer import BackupTrajectoryOptimizer
from committed_trajectory import CommittedTrajectory, TrajectoryState
from corridor_builder import CorridorBuilder
from exp_traj_optimizer import ExpTrajectoryOptimizer
from grid_astar import GridAStar, SearchStatus
from path_optimizer import path_delta
from super_map import MapConfig, RollingOccupancyMap
from trajectory_optimizer import optimize_trajectory_path
from yaw_traj_optimizer import YawPoint, YawTrajectoryOptimizer

PathXY = List[Tuple[float, float]]


class PlanRetCode(Enum):
    SUCCESS = auto()
    FAILED = auto()
    NO_NEED = auto()
    EMER = auto()
    FINISH = auto()


@dataclass
class PlannerCoreConfig:
    map: MapConfig = field(default_factory=MapConfig)
    goal_tolerance: float = 0.8
    path_commit_delta_m: float = 0.8
    receding_horizon_m: float = 8.0
    sample_spacing_m: float = 0.35
    use_raw_map_fallback: bool = True
    offtrack_replan_m: float = 2.5


@dataclass
class PlanState:
    current_path: PathXY = field(default_factory=list)
    pending_path: Optional[PathXY] = None
    path_ready: bool = False
    goal_xy: Tuple[float, float] = (0.0, 0.0)
    goal_valid: bool = False
    connected_to_goal: bool = False
    committed_traj: Optional[CommittedTrajectory] = None
    backup_traj: Optional[CommittedTrajectory] = None
    yaw_traj: List[YawPoint] = field(default_factory=list)
    traj_stamp: float = 0.0


class SuperPlannerCore:
    """Safe SUPER-style planning front-end."""

    def __init__(self, cfg: Optional[PlannerCoreConfig] = None) -> None:
        self.cfg = cfg or PlannerCoreConfig()
        self.map = RollingOccupancyMap(self.cfg.map)
        self.astar = GridAStar()
        self.corridor = CorridorBuilder(robot_radius=self.cfg.map.obstacle_inflation)
        self.exp_opt = ExpTrajectoryOptimizer(max_speed=5.0)
        self.backup_opt = BackupTrajectoryOptimizer()
        self.yaw_opt = YawTrajectoryOptimizer()
        self.state = PlanState()

    def set_goal(self, gx: float, gy: float) -> bool:
        blocked = self._blocked_set((gx, gy))
        snapped = self.map.nearest_free_xy(gx, gy, blocked)
        if snapped is None:
            self.state.goal_valid = False
            return False
        self.state.goal_xy = snapped
        self.state.goal_valid = True
        self.state.connected_to_goal = False
        self.state.committed_traj = None
        self.state.backup_traj = None
        self.state.yaw_traj = []
        return True

    def close_to_goal(self, sx: float, sy: float) -> bool:
        gx, gy = self.state.goal_xy
        return math.hypot(gx - sx, gy - sy) < self.cfg.goal_tolerance

    def plan_from_rest(
        self,
        sx: float,
        sy: float,
        now_sec: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
    ) -> PlanRetCode:
        if not self.state.goal_valid:
            return PlanRetCode.FAILED
        if self.close_to_goal(sx, sy):
            self.state.current_path = [(sx, sy)]
            self.state.path_ready = True
            self.state.connected_to_goal = True
            self._build_trajectory_bundle(now_sec=now_sec, sx=sx, sy=sy, vx=vx, vy=vy)
            return PlanRetCode.FINISH

        blocked = self._blocked_set((sx, sy))
        start_free = self.map.nearest_free_xy(sx, sy, blocked)
        if start_free is None:
            return PlanRetCode.EMER
        sx, sy = start_free

        path = self._search_path(sx, sy)
        if not path:
            return PlanRetCode.EMER

        self.state.connected_to_goal = path[-1] == self.state.goal_xy
        self.state.current_path = path
        self.state.path_ready = True
        self.state.pending_path = None
        self._build_trajectory_bundle(now_sec=now_sec, sx=sx, sy=sy, vx=vx, vy=vy)
        return PlanRetCode.SUCCESS

    def replan_once(
        self,
        sx: float,
        sy: float,
        new_goal: bool,
        now_sec: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
    ) -> PlanRetCode:
        if not self.state.goal_valid:
            return PlanRetCode.FAILED
        if self.close_to_goal(sx, sy):
            self.state.connected_to_goal = True
            self._build_trajectory_bundle(now_sec=now_sec, sx=sx, sy=sy, vx=vx, vy=vy)
            return PlanRetCode.FINISH

        if (
            not new_goal
            and self.state.path_ready
            and self.state.connected_to_goal
            and self.state.current_path
            and self._distance_to_path((sx, sy), self.state.current_path) < self.cfg.offtrack_replan_m
        ):
            return PlanRetCode.NO_NEED

        path = self._search_path(sx, sy)
        if not path:
            if self.state.path_ready and self.state.current_path:
                # SUPER-like behavior: keep following the committed traj/path
                # instead of entering EMER immediately on one failed replan.
                return PlanRetCode.NO_NEED
            return PlanRetCode.FAILED

        self.state.connected_to_goal = path[-1] == self.state.goal_xy
        self._commit_pending(path)
        if self.state.path_ready:
            self._build_trajectory_bundle(now_sec=now_sec, sx=sx, sy=sy, vx=vx, vy=vy)
        return PlanRetCode.SUCCESS if self.state.path_ready else PlanRetCode.FAILED

    def reset_path_on_new_goal(self) -> None:
        self.state.current_path = []
        self.state.pending_path = None
        self.state.path_ready = False
        self.state.connected_to_goal = False
        self.state.committed_traj = None
        self.state.backup_traj = None
        self.state.yaw_traj = []
        self.state.traj_stamp = 0.0

    def _blocked_set(self, center_xy: Tuple[float, float]) -> Set[Tuple[int, int]]:
        raw = self.map.cells_in_horizon(center_xy)
        blocked = self.map.inflate(raw)
        start = self.map.world_to_grid(center_xy[0], center_xy[1])
        return self.map.clear_around(blocked, start)

    def _optimize_path(self, path: PathXY, blocked: Set[Tuple[int, int]]) -> PathXY:
        return optimize_trajectory_path(
            path,
            lambda a, b: self.map.segment_is_free(a, b, blocked),
            self.cfg.sample_spacing_m,
        )

    @staticmethod
    def _truncate_path_length(path: PathXY, max_len: float) -> PathXY:
        if len(path) <= 1 or max_len <= 0.0:
            return path
        out: PathXY = [path[0]]
        remain = max_len
        for i in range(len(path) - 1):
            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg <= 1e-6:
                continue
            if seg <= remain:
                out.append((x1, y1))
                remain -= seg
                if remain <= 1e-6:
                    break
            else:
                r = remain / seg
                out.append((x0 + r * (x1 - x0), y0 + r * (y1 - y0)))
                break
        return out

    def _search_path(self, sx: float, sy: float) -> PathXY:
        gx, gy = self.state.goal_xy
        start = self.map.world_to_grid(sx, sy)
        goal = self.map.world_to_grid(gx, gy)
        blocked = self._blocked_set((sx, sy))
        max_cell = int(self.cfg.map.planning_horizon / self.cfg.map.resolution) + 2
        escape_prefix: List[Tuple[int, int]] = []

        # SUPER-style escape pass: if start is blocked, move to nearest free first.
        if start in blocked:
            esc = self.astar.escape_search(start, blocked, max_cell)
            if not esc:
                return []
            escape_prefix = esc
            start = esc[-1]

        result = self.astar.search(start, goal, blocked, max_cell)
        cells = result.path

        if (not cells or result.status == SearchStatus.NO_PATH) and self.cfg.use_raw_map_fallback:
            raw = self.map.cells_in_horizon((sx, sy))
            raw = self.map.clear_around(raw, start)
            result = self.astar.search(start, goal, raw, max_cell)
            cells = result.path
            blocked = self.map.inflate(raw)
            blocked = self.map.clear_around(blocked, start)

        # Adaptive-inflation fallback: reduce inflation when local map is too tight.
        if not cells:
            raw = self.map.cells_in_horizon((sx, sy))
            raw = self.map.clear_around(raw, start)
            for scale in (0.8, 0.6, 0.45):
                relaxed = self._inflate_with_radius(raw, self.cfg.map.obstacle_inflation * scale)
                relaxed = self.map.clear_around(relaxed, start)
                result = self.astar.search(start, goal, relaxed, max_cell)
                cells = result.path
                if cells:
                    blocked = relaxed
                    break

        if not cells:
            return []

        if escape_prefix:
            cells = escape_prefix[:-1] + cells
        path = [self.map.grid_to_world(c[0], c[1]) for c in cells]
        path = self._optimize_path(path, blocked)
        # SUPER-style receding behavior: if not connected to goal, only commit
        # a bounded local segment to avoid long opposite-direction exploration.
        if result.status != SearchStatus.REACH_GOAL:
            path = self._truncate_path_length(path, self.cfg.receding_horizon_m)
        if result.status == SearchStatus.REACH_GOAL and path:
            path[-1] = (gx, gy)
        return path

    def _inflate_with_radius(
        self, obstacles: Set[Tuple[int, int]], radius_m: float
    ) -> Set[Tuple[int, int]]:
        if radius_m <= 1e-6:
            return set(obstacles)
        inf_cells = int(math.ceil(radius_m / self.cfg.map.resolution))
        out: Set[Tuple[int, int]] = set()
        for ox, oy in obstacles:
            for dx in range(-inf_cells, inf_cells + 1):
                for dy in range(-inf_cells, inf_cells + 1):
                    if dx * dx + dy * dy <= inf_cells * inf_cells:
                        out.add((ox + dx, oy + dy))
        return out

    def _commit_pending(self, pending: PathXY) -> None:
        if not pending:
            return
        if not self.state.current_path:
            self.state.current_path = pending
            self.state.path_ready = True
            self.state.pending_path = None
            return
        if path_delta(self.state.current_path, pending) < self.cfg.path_commit_delta_m:
            self.state.pending_path = None
            return
        self.state.current_path = pending
        self.state.path_ready = True
        self.state.pending_path = None

    def _build_trajectory_bundle(self, now_sec: float, sx: float, sy: float, vx: float, vy: float) -> None:
        if not self.state.current_path:
            self.state.committed_traj = None
            self.state.backup_traj = None
            self.state.yaw_traj = []
            self.state.traj_stamp = now_sec
            return
        corridor = self.corridor.build(self.state.current_path)
        committed = self.exp_opt.optimize(self.state.current_path, corridor)
        now = TrajectoryState(x=sx, y=sy, vx=vx, vy=vy, ax=0.0, ay=0.0)
        backup = self.backup_opt.generate(committed, now)
        yaw = self.yaw_opt.optimize(committed)
        self.state.committed_traj = committed
        self.state.backup_traj = backup
        self.state.yaw_traj = yaw
        self.state.traj_stamp = now_sec

    @staticmethod
    def _distance_to_path(pos_xy: Tuple[float, float], path: PathXY) -> float:
        if not path:
            return 1e9
        px, py = pos_xy
        best = 1e18
        for x, y in path:
            d = (x - px) * (x - px) + (y - py) * (y - py)
            if d < best:
                best = d
        return math.sqrt(best)
