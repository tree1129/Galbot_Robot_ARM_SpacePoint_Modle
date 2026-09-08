import unittest
from pathlib import Path

import numpy as np

from g1_grasp.reachability import VoxelReachabilityMap


ROOT = Path(__file__).resolve().parents[1]


class ReachabilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reachability = VoxelReachabilityMap(
            ROOT / "data/whole-body_left_020mm.npz", "left"
        )

    def test_known_voxel_is_reachable(self):
        data = np.load(ROOT / "data/whole-body_left_020mm.npz", allow_pickle=False)
        point = tuple(float(value) for value in data["centers_m"][12345])
        result = self.reachability.query(point)
        self.assertTrue(result.reachable)
        self.assertLess(result.distance_m, 1e-6)
        self.assertEqual(len(result.representative_q_rad), 12)

    def test_far_point_is_not_reachable(self):
        self.assertFalse(self.reachability.query((9.0, 9.0, 9.0)).reachable)


if __name__ == "__main__":
    unittest.main()

