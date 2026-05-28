"""Path simplification, shortcut optimization and target selection."""

from __future__ import annotations

import math
from typing import Callable, List, Tuple

PathXY = List[Tuple[float, float]]


def simplify_collinear(path: PathXY) -> PathXY:
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    for i in range(1, len(path) - 1):
        x0, y0 = out[-1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        v1 = (x1 - x0, y1 - y0)
        v2 = (x2 - x1, y2 - y1)
        if abs(v1[0] * v2[1] - v1[1] * v2[0]) > 1e-3:
            out.append(path[i])
    out.append(path[-1])
    return out


def shortcut_optimize(path: PathXY, is_segment_free: Callable[[Tuple[float, float], Tuple[float, float]], bool]) -> PathXY:
    if len(path) <= 2:
        return list(path)
    out: PathXY = [path[0]]
    i = 0
    n = len(path)
    while i < n - 1:
        jump = i + 1
        for j in range(n - 1, i, -1):
            if is_segment_free(path[i], path[j]):
                jump = j
                break
        out.append(path[jump])
        i = jump
    return simplify_collinear(out)


def resample_path(path: PathXY, spacing: float) -> PathXY:
    if len(path) <= 1 or spacing <= 1e-6:
        return list(path)
    out: PathXY = [path[0]]
    carry = 0.0
    for i in range(len(path) - 1):
        x0, y0 = path[i]
        x1, y1 = path[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-6:
            continue
        travel = spacing - carry
        while travel <= seg:
            r = travel / seg
            out.append((x0 + r * (x1 - x0), y0 + r * (y1 - y0)))
            travel += spacing
        carry = seg - (travel - spacing)
    if out[-1] != path[-1]:
        out.append(path[-1])
    return out


def path_delta(path_a: PathXY, path_b: PathXY) -> float:
    if not path_a or not path_b:
        return 1e9
    try:
        d_start = math.hypot(path_a[0][0] - path_b[0][0], path_a[0][1] - path_b[0][1])
        d_end = math.hypot(path_a[-1][0] - path_b[-1][0], path_a[-1][1] - path_b[-1][1])
        ma = path_a[len(path_a) // 2]
        mb = path_b[len(path_b) // 2]
        d_mid = math.hypot(ma[0] - mb[0], ma[1] - mb[1])
    except (IndexError, TypeError):
        return 1e9
    return max(d_start, d_end, d_mid)


def lookahead_target(pos_xy: Tuple[float, float], path_xy: PathXY, lookahead: float) -> Tuple[float, float]:
    px, py = pos_xy
    best_i = 0
    best_d = 1e18
    for i, (x, y) in enumerate(path_xy):
        d = (x - px) * (x - px) + (y - py) * (y - py)
        if d < best_d:
            best_d = d
            best_i = i
    travel = 0.0
    for i in range(best_i, len(path_xy) - 1):
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        if travel + seg >= lookahead:
            ratio = (lookahead - travel) / max(seg, 1e-6)
            return x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0)
        travel += seg
    return path_xy[-1]
