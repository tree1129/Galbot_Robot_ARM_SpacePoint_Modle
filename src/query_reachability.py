#!/usr/bin/env python3
"""Query the nearest occupied voxel in a generated G1 reachability map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map", type=Path, help="one generated .npz map")
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument(
        "--tolerance-mm",
        type=float,
        default=None,
        help="default is half the voxel diagonal",
    )
    args = parser.parse_args()
    data = np.load(args.map, allow_pickle=False)
    centers = data["centers_m"]
    target = np.asarray([args.x, args.y, args.z], dtype=np.float64)
    distance, index = cKDTree(centers).query(target, k=1)
    voxel_size = float(data["voxel_size_m"])
    tolerance = (
        args.tolerance_mm / 1000.0
        if args.tolerance_mm is not None
        else math_sqrt3_over_2() * voxel_size
    )
    result = {
        "query_m": target.tolist(),
        "reachable_within_tolerance": bool(distance <= tolerance),
        "tolerance_m": tolerance,
        "nearest_voxel_center_m": centers[index].astype(float).tolist(),
        "distance_m": float(distance),
        "base_frame": str(data["base_frame"]),
        "tip_frame": str(data["tip_frame"]),
        "joint_names": data["joint_names"].astype(str).tolist(),
        "representative_q_rad": data["representative_q_rad"][index].astype(float).tolist(),
        "sample_count_in_voxel": int(data["sample_count"][index]),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["reachable_within_tolerance"] else 1


def math_sqrt3_over_2() -> float:
    return 0.8660254037844386


if __name__ == "__main__":
    raise SystemExit(main())
