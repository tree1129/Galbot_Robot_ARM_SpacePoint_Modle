"""Conservative end-effector corridor checks before vendor motion planning."""

from __future__ import annotations

import numpy as np

from .models import AABBObstacle


def point_aabb_distance(point: np.ndarray, obstacle: AABBObstacle) -> float:
    low = np.asarray(obstacle.minimum_base_m, dtype=np.float64)
    high = np.asarray(obstacle.maximum_base_m, dtype=np.float64)
    delta = np.maximum(np.maximum(low - point, point - high), 0.0)
    return float(np.linalg.norm(delta))


def corridor_is_clear(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    obstacles: list[AABBObstacle],
    clearance_m: float,
    sample_step_m: float = 0.01,
) -> tuple[bool, str | None]:
    a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    count = max(2, int(np.ceil(np.linalg.norm(b - a) / sample_step_m)) + 1)
    for point in np.linspace(a, b, count):
        for obstacle in obstacles:
            if point_aabb_distance(point, obstacle) < clearance_m:
                return False, obstacle.obstacle_id
    return True, None

