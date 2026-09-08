"""Strict JSON boundary for replay files and the shadow HTTP service."""

from __future__ import annotations

from typing import Any

from .models import AABBObstacle, ObjectCandidate, SafetySnapshot


def parse_scene(raw: dict[str, Any]) -> tuple[
    list[ObjectCandidate],
    list[AABBObstacle],
    SafetySnapshot,
    dict[str, tuple[float, float, float]],
]:
    candidates = [
        ObjectCandidate(
            candidate_id=_short_text(item["candidate_id"], "candidate_id"),
            label=_short_text(item["label"], "label"),
            confidence=_bounded(item["confidence"], 0, 1, "confidence"),
            center_base_m=_vector3(item["center_base_m"], "center_base_m"),
            size_m=_positive_vector3(item["size_m"], "size_m"),
            source_cameras=tuple(_short_text(value, "camera") for value in item.get("source_cameras", [])),
            depth_coverage=_bounded(item.get("depth_coverage", 1.0), 0, 1, "depth_coverage"),
        )
        for item in raw["candidates"]
    ]
    if len(candidates) > 100:
        raise ValueError("scene contains more than 100 candidates")
    identifiers = [item.candidate_id for item in candidates]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError("candidate ids must be unique")

    obstacles = [
        AABBObstacle(
            obstacle_id=_short_text(item["obstacle_id"], "obstacle_id"),
            minimum_base_m=_vector3(item["minimum_base_m"], "minimum_base_m"),
            maximum_base_m=_vector3(item["maximum_base_m"], "maximum_base_m"),
        )
        for item in raw.get("obstacles", [])
    ]
    if len(obstacles) > 500:
        raise ValueError("scene contains more than 500 obstacles")
    for obstacle in obstacles:
        if any(a >= b for a, b in zip(obstacle.minimum_base_m, obstacle.maximum_base_m)):
            raise ValueError(f"invalid AABB for {obstacle.obstacle_id}")

    snapshot = raw["safety"]
    safety = SafetySnapshot(
        frame_age_ms=_bounded(snapshot["frame_age_ms"], 0, 60_000, "frame_age_ms"),
        calibration_residual_mm=_bounded(
            snapshot["calibration_residual_mm"], 0, 10_000, "calibration_residual_mm"
        ),
        depth_coverage=_bounded(snapshot["depth_coverage"], 0, 1, "depth_coverage"),
        nearest_human_m=_bounded(snapshot["nearest_human_m"], 0, 100, "nearest_human_m"),
        estop_available=bool(snapshot["estop_available"]),
        robot_state_fresh=bool(snapshot["robot_state_fresh"]),
    )
    current_tcp = {
        arm: _vector3(raw["current_tcp_base_m"][arm], f"current_tcp_base_m.{arm}")
        for arm in ("left", "right")
    }
    return candidates, obstacles, safety, current_tcp


def _short_text(value: Any, field: str) -> str:
    text = str(value).strip()
    if not text or len(text) > 128:
        raise ValueError(f"{field} must contain 1..128 characters")
    return text


def _bounded(value: Any, low: float, high: float, field: str) -> float:
    number = float(value)
    if not low <= number <= high:
        raise ValueError(f"{field} must be in [{low}, {high}]")
    return number


def _vector3(value: Any, field: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field} must be a 3-vector")
    return tuple(_bounded(number, -20, 20, field) for number in value)


def _positive_vector3(value: Any, field: str) -> tuple[float, float, float]:
    result = _vector3(value, field)
    if any(number <= 0 or number > 2 for number in result):
        raise ValueError(f"{field} must contain values in (0, 2]")
    return result

