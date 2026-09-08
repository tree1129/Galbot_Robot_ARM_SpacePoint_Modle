"""Hard safety gates. A VLM can never override these checks."""

from __future__ import annotations

from .models import ObjectCandidate, SafetySnapshot


def validate_snapshot(snapshot: SafetySnapshot, target: ObjectCandidate | None = None) -> list[str]:
    violations: list[str] = []
    if snapshot.frame_age_ms > 180:
        violations.append("camera frame is stale (>180 ms)")
    if snapshot.calibration_residual_mm > 12:
        violations.append("hand-eye calibration residual exceeds 12 mm")
    if snapshot.depth_coverage < 0.75:
        violations.append("scene depth coverage is below 75%")
    if snapshot.nearest_human_m < 1.0:
        violations.append("person detected inside 1.0 m safety radius")
    if not snapshot.estop_available:
        violations.append("emergency stop is not reported available")
    if not snapshot.robot_state_fresh:
        violations.append("robot joint state is stale")
    if target is not None and target.depth_coverage < 0.80:
        violations.append("target depth coverage is below 80%")
    return violations

