"""SUPER mission_planner waypoint sequencer (Python port)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class MissionConfig:
    switch_dis_default: float = 3.0
    odom_timeout: float = 0.5
    publish_dt: float = 1.0


@dataclass
class WaypointMission:
    """
    Mirrors SUPER/mission_planner WaypointPlanner:
    - load waypoint list from file
    - advance when within switch_dis of current waypoint
    - expose active goal for the local planner
    """

    cfg: MissionConfig = field(default_factory=MissionConfig)
    waypoints: List[Vec3] = field(default_factory=list)
    switch_dis: List[float] = field(default_factory=list)
    waypoint_idx: int = 0
    triggered: bool = False
    new_goal: bool = False
    finished: bool = False
    last_odom_time: float = 0.0
    cur_position: Vec3 = (0.0, 0.0, 0.0)
    has_odom: bool = False

    @classmethod
    def from_file(cls, path: Path, cfg: Optional[MissionConfig] = None) -> "WaypointMission":
        mission = cls(cfg=cfg or MissionConfig())
        mission.load(path)
        return mission

    def load(self, path: Path) -> None:
        self.waypoints.clear()
        self.switch_dis.clear()
        text = path.read_text(encoding="utf-8").splitlines()
        for line in text:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            x, y, z, sw = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
            self.waypoints.append((x, y, z))
            self.switch_dis.append(sw)
        if not self.waypoints:
            raise ValueError(f"No waypoints parsed from {path}")
        self.reset()

    def reset(self) -> None:
        self.waypoint_idx = 0
        self.triggered = True
        self.new_goal = True
        self.finished = False

    def trigger(self) -> None:
        self.triggered = True
        self.new_goal = True
        self.waypoint_idx = 0
        self.finished = False

    def update_odom(self, x: float, y: float, z: float, stamp_sec: float) -> None:
        self.cur_position = (x, y, z)
        self.last_odom_time = stamp_sec
        self.has_odom = True

    def _close_to(self, wp: Vec3, switch_m: float) -> bool:
        dx = wp[0] - self.cur_position[0]
        dy = wp[1] - self.cur_position[1]
        dz = wp[2] - self.cur_position[2]
        return (dx * dx + dy * dy + dz * dz) ** 0.5 < switch_m

    def tick(self, now_sec: float) -> Optional[Vec3]:
        """Return goal xyz to publish when mission is active."""
        if not self.triggered or self.finished or not self.has_odom:
            return None
        if now_sec - self.last_odom_time > self.cfg.odom_timeout:
            return None

        sw = (
            self.switch_dis[self.waypoint_idx]
            if self.waypoint_idx < len(self.switch_dis)
            else self.cfg.switch_dis_default
        )
        if self._close_to(self.waypoints[self.waypoint_idx], sw):
            self.waypoint_idx += 1
            self.new_goal = True
            if self.waypoint_idx >= len(self.waypoints):
                self.waypoint_idx = len(self.waypoints) - 1
                self.finished = True
                self.triggered = False
                return None

        if self.new_goal:
            self.new_goal = False
            return self.waypoints[self.waypoint_idx]
        return None

    @property
    def active_goal_xy(self) -> Optional[Tuple[float, float]]:
        if not self.waypoints or self.finished:
            return None
        wp = self.waypoints[self.waypoint_idx]
        return wp[0], wp[1]
