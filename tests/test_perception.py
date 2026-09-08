import unittest

import numpy as np

from g1_grasp.perception import (
    CameraIntrinsics,
    RigidTransform,
    fuse_points_base,
    pixel_to_base,
    robust_depth_m,
)


class PerceptionTests(unittest.TestCase):
    def test_median_depth_rejects_outlier(self):
        depth = np.full((9, 9), 0.8, np.float32)
        depth[4, 4] = 3.5
        self.assertAlmostEqual(robust_depth_m(depth, 4, 4), 0.8, places=5)

    def test_deprojection_and_transform(self):
        intrinsics = CameraIntrinsics(100, 100, 10, 10)
        transform = np.eye(4)
        transform[:3, 3] = [0.5, 0.2, 0.1]
        result = pixel_to_base(20, 10, 1.0, intrinsics, RigidTransform(transform))
        np.testing.assert_allclose(result, [0.6, 0.2, 1.1])

    def test_fusion_fails_closed_on_disagreement(self):
        with self.assertRaises(ValueError):
            fuse_points_base([(0, 0, 1), (0.2, 0, 1)])


if __name__ == "__main__":
    unittest.main()

