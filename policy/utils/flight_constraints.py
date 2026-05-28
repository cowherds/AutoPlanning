"""Forest-map flight limits (aligned with Simulator/src/config/config.yaml)."""

from __future__ import annotations

from typing import Sequence, Tuple, Union

import numpy as np

# x_length / y_length = 60 m, map centered at origin
MAP_HALF_EXTENT_X = 30.0
MAP_HALF_EXTENT_Y = 30.0
MAP_MARGIN = 1.0

MIN_FLIGHT_HEIGHT = 0.5
MAX_FLIGHT_HEIGHT = 4.0
DEFAULT_FLIGHT_HEIGHT = 3.0

MAP_X_MIN = -MAP_HALF_EXTENT_X + MAP_MARGIN
MAP_X_MAX = MAP_HALF_EXTENT_X - MAP_MARGIN
MAP_Y_MIN = -MAP_HALF_EXTENT_Y + MAP_MARGIN
MAP_Y_MAX = MAP_HALF_EXTENT_Y - MAP_MARGIN


def cruise_height(flight_height: float) -> float:
    # return float(np.clip(flight_height, MIN_FLIGHT_HEIGHT, MAX_FLIGHT_HEIGHT))
    return float(np.clip(flight_height, DEFAULT_FLIGHT_HEIGHT, DEFAULT_FLIGHT_HEIGHT))


def clamp_height(z: float) -> float:
    return float(np.clip(z, MIN_FLIGHT_HEIGHT, MAX_FLIGHT_HEIGHT))


def clamp_xy(x: float, y: float) -> Tuple[float, float]:
    return (
        float(np.clip(x, MAP_X_MIN, MAP_X_MAX)),
        float(np.clip(y, MAP_Y_MIN, MAP_Y_MAX)),
    )


def clamp_goal_xy(goal_xy: Sequence[float]) -> np.ndarray:
    x, y = clamp_xy(float(goal_xy[0]), float(goal_xy[1]))
    return np.array([x, y], dtype=float)


def make_3d_goal(goal_xy: Sequence[float], flight_height: float) -> np.ndarray:
    xy = clamp_goal_xy(goal_xy)
    z = cruise_height(flight_height)
    return np.array([xy[0], xy[1], z], dtype=float)


def clamp_position_xyz(pos: Sequence[float], flight_height: float | None = None) -> np.ndarray:
    p = np.asarray(pos, dtype=float).reshape(-1)
    x, y = clamp_xy(float(p[0]), float(p[1]))
    if flight_height is not None:
        z = cruise_height(flight_height)
    elif len(p) > 2:
        z = clamp_height(float(p[2]))
    else:
        z = DEFAULT_FLIGHT_HEIGHT
    return np.array([x, y, z], dtype=float)


def flat_z_poly_solver(poly5_class, flight_height: float, traj_time: float):
    """Constant-altitude segment: no vertical motion from the planner."""
    z = cruise_height(flight_height)
    return poly5_class(z, 0.0, 0.0, z, 0.0, 0.0, traj_time)
