"""Camera geometry for RGB-D detections; no detector-specific dependency."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class RigidTransform:
    """Homogeneous transform from a camera optical frame to base_link."""

    matrix: np.ndarray

    def __post_init__(self) -> None:
        matrix = np.asarray(self.matrix, dtype=np.float64)
        if matrix.shape != (4, 4) or not np.allclose(matrix[3], [0, 0, 0, 1]):
            raise ValueError("transform must be a 4x4 homogeneous matrix")
        object.__setattr__(self, "matrix", matrix)


def robust_depth_m(
    depth_m: np.ndarray,
    u: float,
    v: float,
    radius_px: int = 3,
    minimum_valid_pixels: int = 5,
) -> float:
    """Return the median positive finite depth around an RGB detection."""

    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("depth image must be HxW")
    x, y = int(round(u)), int(round(v))
    patch = depth[
        max(0, y - radius_px) : min(depth.shape[0], y + radius_px + 1),
        max(0, x - radius_px) : min(depth.shape[1], x + radius_px + 1),
    ]
    values = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 4.0)]
    if values.size < minimum_valid_pixels:
        raise ValueError("insufficient valid depth around detection")
    return float(np.median(values))


def pixel_to_base(
    u: float,
    v: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
    camera_to_base: RigidTransform,
) -> tuple[float, float, float]:
    """Back-project a pixel and transform the point into base_link."""

    if not 0.05 < depth_m < 4.0:
        raise ValueError("depth outside calibrated tabletop range")
    point_camera = np.array(
        [
            (u - intrinsics.cx) * depth_m / intrinsics.fx,
            (v - intrinsics.cy) * depth_m / intrinsics.fy,
            depth_m,
            1.0,
        ]
    )
    point_base = camera_to_base.matrix @ point_camera
    return tuple(float(value) for value in point_base[:3])


def fuse_points_base(
    points: list[tuple[float, float, float]], maximum_spread_m: float = 0.04
) -> tuple[float, float, float]:
    """Fuse head/wrist estimates, rejecting inconsistent calibration or depth."""

    if not points:
        raise ValueError("at least one point is required")
    array = np.asarray(points, dtype=np.float64)
    center = np.median(array, axis=0)
    if float(np.max(np.linalg.norm(array - center, axis=1))) > maximum_spread_m:
        raise ValueError("camera estimates disagree beyond fusion threshold")
    return tuple(float(value) for value in center)

