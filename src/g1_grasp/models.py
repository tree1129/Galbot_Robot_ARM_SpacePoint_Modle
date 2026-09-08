"""Transport-neutral data models used by the shadow grasp pipeline."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True)
class ObjectCandidate:
    candidate_id: str
    label: str
    confidence: float
    center_base_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    source_cameras: tuple[str, ...] = ()
    depth_coverage: float = 1.0


@dataclass(frozen=True)
class AABBObstacle:
    obstacle_id: str
    minimum_base_m: tuple[float, float, float]
    maximum_base_m: tuple[float, float, float]


@dataclass(frozen=True)
class SafetySnapshot:
    frame_age_ms: float
    calibration_residual_mm: float
    depth_coverage: float
    nearest_human_m: float
    estop_available: bool
    robot_state_fresh: bool


@dataclass(frozen=True)
class ReachabilityHit:
    arm: str
    reachable: bool
    query_base_m: tuple[float, float, float]
    nearest_center_base_m: tuple[float, float, float] | None = None
    distance_m: float | None = None
    representative_q_rad: tuple[float, ...] = ()
    joint_names: tuple[str, ...] = ()


@dataclass
class GraspPlan:
    status: str
    command: str
    target_id: str | None = None
    target_label: str | None = None
    arm: str | None = None
    pregrasp_base_m: tuple[float, float, float] | None = None
    grasp_base_m: tuple[float, float, float] | None = None
    retreat_base_m: tuple[float, float, float] | None = None
    semantic_source: str = "deterministic"
    violations: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    representative_q_rad: tuple[float, ...] = ()
    joint_names: tuple[str, ...] = ()
    execution_permitted: bool = False
    requires_vendor_planner: bool = True

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

