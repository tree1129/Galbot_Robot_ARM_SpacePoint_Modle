"""Deterministic grasp planning with optional, non-authoritative VLM semantics."""

from __future__ import annotations

import math
from typing import Protocol

from .models import AABBObstacle, GraspPlan, ObjectCandidate, SafetySnapshot
from .obstacles import corridor_is_clear
from .reachability import DualArmReachability
from .safety import validate_snapshot


class SemanticSelector(Protocol):
    def select(self, command: str, candidates: list[ObjectCandidate], scene_jpeg: bytes) -> str: ...


class GraspCoordinator:
    """Produces reviewable plans only; it has no actuator dependency by design."""

    def __init__(self, reachability: DualArmReachability):
        self.reachability = reachability

    def plan(
        self,
        command: str,
        candidates: list[ObjectCandidate],
        obstacles: list[AABBObstacle],
        safety: SafetySnapshot,
        current_tcp_base_m: dict[str, tuple[float, float, float]],
        semantic_selector: SemanticSelector | None = None,
        scene_jpeg: bytes | None = None,
    ) -> GraspPlan:
        violations = validate_snapshot(safety)
        if violations:
            return GraspPlan("REJECTED", command, violations=violations)
        if not command.strip() or not candidates:
            return GraspPlan("REJECTED", command, violations=["no command or object candidates"])

        selected, source = self._select_target(command, candidates, semantic_selector, scene_jpeg)
        if selected is None:
            return GraspPlan(
                "NEEDS_SEMANTIC_REVIEW",
                command,
                violations=["target is ambiguous and VLM selection was unavailable or failed"],
            )
        violations = validate_snapshot(safety, selected)
        if violations:
            return GraspPlan(
                "REJECTED", command, selected.candidate_id, selected.label, violations=violations
            )

        x, y, z = selected.center_base_m
        half_height = selected.size_m[2] / 2.0
        grasp = (x, y, z + min(0.01, half_height * 0.25))
        pregrasp = (x, y, grasp[2] + 0.12)
        retreat = (x, y, grasp[2] + 0.20)
        other_obstacles = [item for item in obstacles if item.obstacle_id != selected.candidate_id]

        options: list[tuple[float, str, object]] = []
        option_notes: list[str] = []
        for arm in ("left", "right"):
            hit_grasp = self.reachability.query(arm, grasp)
            hit_pre = self.reachability.query(arm, pregrasp)
            if not hit_grasp.reachable or not hit_pre.reachable:
                option_notes.append(f"{arm}: grasp/pregrasp is outside sampled reachable voxels")
                continue
            clear_1, blocker_1 = corridor_is_clear(
                current_tcp_base_m[arm], pregrasp, other_obstacles, clearance_m=0.07
            )
            clear_2, blocker_2 = corridor_is_clear(
                pregrasp, grasp, other_obstacles, clearance_m=0.055
            )
            if not clear_1 or not clear_2:
                option_notes.append(f"{arm}: end-effector corridor blocked by {blocker_1 or blocker_2}")
                continue
            travel = math.dist(current_tcp_base_m[arm], pregrasp)
            cross_body = 0.18 if (arm == "left" and x < -0.05) or (arm == "right" and x > 0.05) else 0.0
            options.append((travel + cross_body, arm, hit_grasp))

        if not options:
            return GraspPlan(
                "NO_SAFE_ARM",
                command,
                selected.candidate_id,
                selected.label,
                semantic_source=source,
                violations=["no arm passed reachability and corridor prechecks"],
                notes=option_notes,
            )

        _, arm, hit = min(options, key=lambda item: item[0])
        return GraspPlan(
            status="READY_FOR_SHADOW_REVIEW",
            command=command,
            target_id=selected.candidate_id,
            target_label=selected.label,
            arm=arm,
            pregrasp_base_m=pregrasp,
            grasp_base_m=grasp,
            retreat_base_m=retreat,
            semantic_source=source,
            notes=option_notes
            + [
                "endpoint-only corridor precheck passed",
                "vendor IK/full-link collision planning is still mandatory",
                "physical execution is intentionally unavailable in this build",
            ],
            representative_q_rad=hit.representative_q_rad,
            joint_names=hit.joint_names,
            execution_permitted=False,
            requires_vendor_planner=True,
        )

    @staticmethod
    def _select_target(
        command: str,
        candidates: list[ObjectCandidate],
        selector: SemanticSelector | None,
        scene_jpeg: bytes | None,
    ) -> tuple[ObjectCandidate | None, str]:
        normalized = command.casefold()
        matches = [item for item in candidates if item.label.casefold() in normalized]
        if len(matches) == 1:
            return matches[0], "deterministic-label"
        if len(candidates) == 1 and not matches:
            return candidates[0], "single-candidate"
        if selector is None or scene_jpeg is None:
            return None, "unresolved"
        try:
            candidate_id = selector.select(command, candidates, scene_jpeg)
        except Exception:
            return None, "vlm-failed-closed"
        return next((item for item in candidates if item.candidate_id == candidate_id), None), "vlm"

