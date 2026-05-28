"""Compatibility wrapper for algorithm-named path utilities."""

from __future__ import annotations

from typing import List, Tuple

from path_optimizer import (
    lookahead_target,
    path_delta,
    resample_path,
    shortcut_optimize,
    simplify_collinear,
)

PathXY = List[Tuple[float, float]]


def smooth_path(path: PathXY) -> PathXY:
    path = simplify_collinear(path)
    return resample_path(path, 0.35)


__all__ = [
    "PathXY",
    "smooth_path",
    "path_delta",
    "lookahead_target",
    "shortcut_optimize",
    "simplify_collinear",
    "resample_path",
]
