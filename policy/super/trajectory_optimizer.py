"""Trajectory optimization primitives inspired by SUPER trajectory modules."""

from __future__ import annotations

from typing import Callable, List, Tuple

from path_optimizer import resample_path, shortcut_optimize, simplify_collinear

PathXY = List[Tuple[float, float]]


def _minimum_jerk_blend(s: float) -> float:
    # Quintic polynomial: 10s^3 - 15s^4 + 6s^5
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


def minimum_jerk_interpolate(path: PathXY, samples_per_seg: int = 4) -> PathXY:
    if len(path) <= 1:
        return list(path)
    out: PathXY = [path[0]]
    n = max(2, samples_per_seg)
    for i in range(len(path) - 1):
        x0, y0 = path[i]
        x1, y1 = path[i + 1]
        for k in range(1, n):
            s = k / n
            b = _minimum_jerk_blend(s)
            out.append((x0 + b * (x1 - x0), y0 + b * (y1 - y0)))
        out.append((x1, y1))
    return out


def optimize_trajectory_path(
    path: PathXY,
    is_segment_free: Callable[[Tuple[float, float], Tuple[float, float]], bool],
    spacing_m: float,
) -> PathXY:
    if len(path) <= 1:
        return list(path)
    compact = simplify_collinear(path)
    compact = shortcut_optimize(compact, is_segment_free)
    smooth = minimum_jerk_interpolate(compact, samples_per_seg=4)

    # Keep only safe links; if smoothing introduces unsafe link, drop to compact.
    safe = True
    for i in range(len(smooth) - 1):
        if not is_segment_free(smooth[i], smooth[i + 1]):
            safe = False
            break
    if not safe:
        smooth = compact
    return resample_path(smooth, spacing_m)
