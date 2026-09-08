import json
import unittest
from pathlib import Path

from g1_grasp.models import AABBObstacle, ObjectCandidate, SafetySnapshot
from g1_grasp.pipeline import GraspCoordinator
from g1_grasp.reachability import DualArmReachability


ROOT = Path(__file__).resolve().parents[1]


class PipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.coordinator = GraspCoordinator(DualArmReachability(ROOT / "data"))
        raw = json.loads((ROOT / "examples/shadow_scene.json").read_text())
        cls.candidates = [
            ObjectCandidate(
                item["candidate_id"],
                item["label"],
                item["confidence"],
                tuple(item["center_base_m"]),
                tuple(item["size_m"]),
                tuple(item["source_cameras"]),
                item["depth_coverage"],
            )
            for item in raw["candidates"]
        ]
        cls.obstacles = [
            AABBObstacle(
                item["obstacle_id"],
                tuple(item["minimum_base_m"]),
                tuple(item["maximum_base_m"]),
            )
            for item in raw["obstacles"]
        ]
        cls.safe = SafetySnapshot(**raw["safety"])
        cls.tcp = {key: tuple(value) for key, value in raw["current_tcp_base_m"].items()}

    def test_fast_path_selects_left_arm_around_blocker(self):
        plan = self.coordinator.plan(
            "抓取苹果", self.candidates, self.obstacles, self.safe, self.tcp
        )
        self.assertEqual(plan.status, "READY_FOR_SHADOW_REVIEW")
        self.assertEqual(plan.arm, "left")
        self.assertFalse(plan.execution_permitted)
        self.assertTrue(any("right:" in note for note in plan.notes))

    def test_human_proximity_rejects_before_planning(self):
        unsafe = SafetySnapshot(**{**self.safe.__dict__, "nearest_human_m": 0.6})
        plan = self.coordinator.plan("grasp apple", self.candidates, [], unsafe, self.tcp)
        self.assertEqual(plan.status, "REJECTED")
        self.assertTrue(any("person" in violation for violation in plan.violations))

    def test_ambiguous_command_requires_semantic_review(self):
        plan = self.coordinator.plan("把那个拿起来", self.candidates, [], self.safe, self.tcp)
        self.assertEqual(plan.status, "NEEDS_SEMANTIC_REVIEW")
        self.assertFalse(plan.execution_permitted)


if __name__ == "__main__":
    unittest.main()

