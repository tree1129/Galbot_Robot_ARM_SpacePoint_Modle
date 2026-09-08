import unittest

from g1_grasp.serialization import parse_scene


def valid_scene():
    return {
        "candidates": [
            {
                "candidate_id": "object_1",
                "label": "object",
                "confidence": 0.9,
                "center_base_m": [0.5, 0.2, 0.8],
                "size_m": [0.1, 0.1, 0.1],
            }
        ],
        "obstacles": [],
        "safety": {
            "frame_age_ms": 20,
            "calibration_residual_mm": 4,
            "depth_coverage": 0.9,
            "nearest_human_m": 2,
            "estop_available": True,
            "robot_state_fresh": True,
        },
        "current_tcp_base_m": {"left": [0, 0.3, 1], "right": [0, -0.3, 1]},
    }


class SerializationTests(unittest.TestCase):
    def test_valid_scene(self):
        candidates, obstacles, safety, tcp = parse_scene(valid_scene())
        self.assertEqual(candidates[0].candidate_id, "object_1")
        self.assertEqual(obstacles, [])
        self.assertEqual(safety.depth_coverage, 0.9)
        self.assertEqual(tcp["left"], (0.0, 0.3, 1.0))

    def test_duplicate_ids_rejected(self):
        scene = valid_scene()
        scene["candidates"].append(dict(scene["candidates"][0]))
        with self.assertRaisesRegex(ValueError, "unique"):
            parse_scene(scene)

    def test_inverted_obstacle_rejected(self):
        scene = valid_scene()
        scene["obstacles"] = [
            {
                "obstacle_id": "bad",
                "minimum_base_m": [1, 1, 1],
                "maximum_base_m": [0, 0, 0],
            }
        ]
        with self.assertRaisesRegex(ValueError, "invalid AABB"):
            parse_scene(scene)


if __name__ == "__main__":
    unittest.main()

