"""Fast indexed queries over the precomputed G1 base_link voxel maps."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .models import ReachabilityHit

_BIAS = 1 << 20
_MASK = (1 << 21) - 1


def _pack(indices: np.ndarray) -> np.ndarray:
    values = np.asarray(indices, dtype=np.int64) + _BIAS
    if np.any(values < 0) or np.any(values > _MASK):
        raise ValueError("voxel index exceeds packed-key range")
    return (values[:, 0] << 42) | (values[:, 1] << 21) | values[:, 2]


class VoxelReachabilityMap:
    def __init__(self, path: str | Path, arm: str):
        self.path = Path(path)
        self.arm = arm
        data = np.load(self.path, allow_pickle=False)
        if str(data["base_frame"]) != "base_link":
            raise ValueError("grasp pipeline requires a whole-body base_link map")
        self.voxel_size_m = float(data["voxel_size_m"])
        self.centers = np.asarray(data["centers_m"], dtype=np.float32)
        self.q = np.asarray(data["representative_q_rad"], dtype=np.float32)
        self.joint_names = tuple(data["joint_names"].astype(str).tolist())
        keys = _pack(data["voxel_indices"])
        order = np.argsort(keys)
        self._keys = keys[order]
        self._rows = order

    def query(
        self, point_base_m: tuple[float, float, float], tolerance_m: float = 0.035
    ) -> ReachabilityHit:
        point = np.asarray(point_base_m, dtype=np.float64)
        center_index = np.floor(point / self.voxel_size_m).astype(np.int64)
        cells = max(0, int(np.ceil(tolerance_m / self.voxel_size_m)))
        offsets = np.array(
            [
                (x, y, z)
                for x in range(-cells, cells + 1)
                for y in range(-cells, cells + 1)
                for z in range(-cells, cells + 1)
            ],
            dtype=np.int64,
        )
        query_keys = _pack(center_index[None, :] + offsets)
        positions = np.searchsorted(self._keys, query_keys)
        valid = positions < self._keys.size
        positions = positions[valid]
        query_keys = query_keys[valid]
        matched = positions[self._keys[positions] == query_keys]
        if matched.size == 0:
            return ReachabilityHit(self.arm, False, tuple(point.tolist()))
        rows = self._rows[matched]
        distances = np.linalg.norm(self.centers[rows] - point, axis=1)
        nearest = int(rows[int(np.argmin(distances))])
        distance = float(np.min(distances))
        if distance > tolerance_m:
            return ReachabilityHit(self.arm, False, tuple(point.tolist()), distance_m=distance)
        return ReachabilityHit(
            arm=self.arm,
            reachable=True,
            query_base_m=tuple(point.tolist()),
            nearest_center_base_m=tuple(float(v) for v in self.centers[nearest]),
            distance_m=distance,
            representative_q_rad=tuple(float(v) for v in self.q[nearest]),
            joint_names=self.joint_names,
        )


class DualArmReachability:
    def __init__(self, data_dir: str | Path):
        root = Path(data_dir)
        self.maps = {
            "left": VoxelReachabilityMap(root / "whole-body_left_020mm.npz", "left"),
            "right": VoxelReachabilityMap(root / "whole-body_right_020mm.npz", "right"),
        }

    def query(self, arm: str, point: tuple[float, float, float]) -> ReachabilityHit:
        return self.maps[arm].query(point)

