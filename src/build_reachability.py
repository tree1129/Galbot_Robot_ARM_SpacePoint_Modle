#!/usr/bin/env python3
"""Build a sampled G1 TCP reachability map without commanding the robot.

The continuous workspace cannot be enumerated.  This program samples the
official URDF joint limits with a deterministic Sobol sequence and stores the
occupied Cartesian voxels plus one representative joint configuration per
voxel.  It is deliberately offline-only: there is no Galbot SDK import and no
actuator or network interface.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.stats import qmc


DEFAULT_URDF = Path(
    "/home/galbot/vla_client/vla_tree/galbot_one_golf_description/urdf/"
    "galbot_g1_v2_2_1.urdf"
)
ARMS = ("left", "right")


@dataclass(frozen=True)
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None


def _floats(value: str | None, default: Iterable[float]) -> np.ndarray:
    return np.asarray(
        list(default) if value is None else [float(item) for item in value.split()],
        dtype=np.float64,
    )


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw rotation Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _origin_matrix(node: ET.Element | None) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    if node is None:
        return transform
    transform[:3, :3] = _rpy_matrix(_floats(node.get("rpy"), (0.0, 0.0, 0.0)))
    transform[:3, 3] = _floats(node.get("xyz"), (0.0, 0.0, 0.0))
    return transform


def load_joints(urdf_path: Path) -> dict[str, Joint]:
    root = ET.parse(urdf_path).getroot()
    joints: dict[str, Joint] = {}
    for node in root.findall("joint"):
        name = node.attrib["name"]
        joint_type = node.attrib["type"]
        parent = node.find("parent")
        child = node.find("child")
        if parent is None or child is None:
            raise ValueError(f"joint {name!r} has no parent or child")
        limit = node.find("limit")
        lower = float(limit.get("lower")) if limit is not None and limit.get("lower") else None
        upper = float(limit.get("upper")) if limit is not None and limit.get("upper") else None
        axis_node = node.find("axis")
        axis = _floats(axis_node.get("xyz") if axis_node is not None else None, (1, 0, 0))
        norm = float(np.linalg.norm(axis))
        if norm == 0:
            raise ValueError(f"joint {name!r} has a zero axis")
        joints[child.attrib["link"]] = Joint(
            name=name,
            joint_type=joint_type,
            parent=parent.attrib["link"],
            child=child.attrib["link"],
            origin=_origin_matrix(node.find("origin")),
            axis=axis / norm,
            lower=lower,
            upper=upper,
        )
    return joints


def chain_between(joints_by_child: dict[str, Joint], base: str, tip: str) -> list[Joint]:
    reverse: list[Joint] = []
    link = tip
    visited: set[str] = set()
    while link != base:
        if link in visited:
            raise ValueError(f"cycle while finding {base} -> {tip}")
        visited.add(link)
        if link not in joints_by_child:
            raise ValueError(f"{base!r} is not an ancestor of {tip!r}; stopped at {link!r}")
        joint = joints_by_child[link]
        reverse.append(joint)
        link = joint.parent
    return list(reversed(reverse))


def _axis_rotation_batch(axis: np.ndarray, angle: np.ndarray) -> np.ndarray:
    """Return homogeneous Rodrigues rotations with shape (N, 4, 4)."""
    x, y, z = axis
    c = np.cos(angle)
    s = np.sin(angle)
    one_c = 1.0 - c
    count = len(angle)
    out = np.zeros((count, 4, 4), dtype=np.float64)
    out[:, 0, 0] = c + x * x * one_c
    out[:, 0, 1] = x * y * one_c - z * s
    out[:, 0, 2] = x * z * one_c + y * s
    out[:, 1, 0] = y * x * one_c + z * s
    out[:, 1, 1] = c + y * y * one_c
    out[:, 1, 2] = y * z * one_c - x * s
    out[:, 2, 0] = z * x * one_c - y * s
    out[:, 2, 1] = z * y * one_c + x * s
    out[:, 2, 2] = c + z * z * one_c
    out[:, 3, 3] = 1.0
    return out


def forward_positions(chain: list[Joint], sampled_names: list[str], q: np.ndarray) -> np.ndarray:
    """Vectorized URDF FK; unspecified movable joints are held at zero."""
    count = len(q)
    transform = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
    column = {name: index for index, name in enumerate(sampled_names)}
    for joint in chain:
        transform = transform @ joint.origin
        if joint.joint_type in ("revolute", "continuous"):
            angle = q[:, column[joint.name]] if joint.name in column else np.zeros(count)
            transform = transform @ _axis_rotation_batch(joint.axis, angle)
        elif joint.joint_type == "prismatic":
            distance = q[:, column[joint.name]] if joint.name in column else np.zeros(count)
            motion = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
            motion[:, :3, 3] = distance[:, None] * joint.axis[None, :]
            transform = transform @ motion
        elif joint.joint_type != "fixed":
            raise ValueError(f"unsupported joint type {joint.joint_type!r} ({joint.name})")
    return transform[:, :3, 3]


def _pack_voxels(indices: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    minimum = indices.min(axis=0).astype(np.int64)
    shifted = indices.astype(np.int64) - minimum
    spans = shifted.max(axis=0) + 1
    if int(np.prod(spans, dtype=np.int64)) >= np.iinfo(np.int64).max:
        raise OverflowError("voxel index range cannot be packed into int64")
    keys = (shifted[:, 0] * spans[1] + shifted[:, 1]) * spans[2] + shifted[:, 2]
    return keys, minimum, spans


def _write_ply(path: Path, centers: np.ndarray, counts: np.ndarray) -> None:
    maximum = max(int(counts.max()), 1)
    intensity = np.log1p(counts.astype(np.float64)) / math.log1p(maximum)
    red = np.asarray(30 + 225 * intensity, dtype=np.uint8)
    green = np.asarray(180 - 120 * intensity, dtype=np.uint8)
    blue = np.asarray(255 - 220 * intensity, dtype=np.uint8)
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write(f"element vertex {len(centers)}\n")
        stream.write("property float x\nproperty float y\nproperty float z\n")
        stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        stream.write("property uint sample_count\nend_header\n")
        for point, r, g, b, count in zip(centers, red, green, blue, counts):
            stream.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(r)} {int(g)} {int(b)} {int(count)}\n"
            )


def _write_csv(
    path: Path,
    centers: np.ndarray,
    counts: np.ndarray,
    representative_q: np.ndarray,
    joint_names: list[str],
) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["x_m", "y_m", "z_m", "sample_count", *[f"{name}_rad" for name in joint_names]])
        for point, count, joint_values in zip(centers, counts, representative_q):
            writer.writerow(
                [
                    f"{point[0]:.6f}",
                    f"{point[1]:.6f}",
                    f"{point[2]:.6f}",
                    int(count),
                    *[f"{value:.9f}" for value in joint_values],
                ]
            )


def build_arm_map(
    *,
    arm: str,
    mode: str,
    urdf_path: Path,
    output_dir: Path,
    sample_power: int,
    voxel_size: float,
    margin_rad: float,
    chunk_size: int,
) -> dict[str, object]:
    joints_by_child = load_joints(urdf_path)
    base_frame = "torso_base_link" if mode == "arm-only" else "base_link"
    tip_frame = f"{arm}_gripper_tcp_link"
    chain = chain_between(joints_by_child, base_frame, tip_frame)

    movable = [joint for joint in chain if joint.joint_type in ("revolute", "prismatic", "continuous")]
    if mode == "arm-only":
        movable = [joint for joint in movable if joint.name.startswith(f"{arm}_arm_joint")]
    else:
        movable = [
            joint
            for joint in movable
            if joint.name.startswith("leg_joint") or joint.name.startswith(f"{arm}_arm_joint")
        ]
    joint_names = [joint.name for joint in movable]
    lower = np.asarray([joint.lower for joint in movable], dtype=np.float64)
    upper = np.asarray([joint.upper for joint in movable], dtype=np.float64)
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        raise ValueError(f"all sampled joints need finite limits: {joint_names}")
    lower = lower + margin_rad
    upper = upper - margin_rad
    if np.any(lower >= upper):
        raise ValueError("joint margin removes an entire joint range")

    total_samples = 1 << sample_power
    sampler = qmc.Sobol(d=len(movable), scramble=True, seed=20260907)
    all_points: list[np.ndarray] = []
    all_q: list[np.ndarray] = []
    started = time.monotonic()
    completed = 0
    while completed < total_samples:
        count = min(chunk_size, total_samples - completed)
        unit = sampler.random(count)
        q = lower + unit * (upper - lower)
        points = forward_positions(chain, joint_names, q)
        all_points.append(points.astype(np.float32))
        all_q.append(q.astype(np.float32))
        completed += count
        elapsed = time.monotonic() - started
        print(
            f"[{mode}/{arm}] {completed:,}/{total_samples:,} samples "
            f"({completed / max(elapsed, 1e-9):,.0f}/s)",
            flush=True,
        )

    points = np.concatenate(all_points, axis=0)
    configurations = np.concatenate(all_q, axis=0)
    voxel_indices = np.floor(points / voxel_size).astype(np.int32)
    keys, minimum, spans = _pack_voxels(voxel_indices)
    _, first, counts = np.unique(keys, return_index=True, return_counts=True)
    unique_indices = voxel_indices[first]
    centers = (unique_indices.astype(np.float64) + 0.5) * voxel_size
    representative_q = configurations[first]

    stem = f"{mode}_{arm}_{round(voxel_size * 1000):03d}mm"
    csv_path = output_dir / f"{stem}.csv"
    ply_path = output_dir / f"{stem}.ply"
    npz_path = output_dir / f"{stem}.npz"
    _write_csv(csv_path, centers, counts, representative_q, joint_names)
    _write_ply(ply_path, centers, counts)
    np.savez_compressed(
        npz_path,
        voxel_indices=unique_indices,
        centers_m=centers.astype(np.float32),
        sample_count=counts.astype(np.uint32),
        representative_q_rad=representative_q,
        joint_names=np.asarray(joint_names),
        voxel_size_m=np.asarray(voxel_size),
        base_frame=np.asarray(base_frame),
        tip_frame=np.asarray(tip_frame),
    )

    bounds = np.stack((centers.min(axis=0), centers.max(axis=0)), axis=1)
    result: dict[str, object] = {
        "arm": arm,
        "mode": mode,
        "base_frame": base_frame,
        "tip_frame": tip_frame,
        "joint_names": joint_names,
        "joint_lower_rad": lower.tolist(),
        "joint_upper_rad": upper.tolist(),
        "sample_count": total_samples,
        "voxel_size_m": voxel_size,
        "occupied_voxel_count": int(len(centers)),
        "occupied_voxel_volume_m3": float(len(centers) * voxel_size**3),
        "bounds_m": {
            "x": bounds[0].tolist(),
            "y": bounds[1].tolist(),
            "z": bounds[2].tolist(),
        },
        "files": {
            "csv": csv_path.name,
            "ply": ply_path.name,
            "npz": npz_path.name,
        },
        "elapsed_s": time.monotonic() - started,
        "voxel_pack_minimum": minimum.tolist(),
        "voxel_pack_spans": spans.tolist(),
    }
    print(
        f"[{mode}/{arm}] {len(centers):,} occupied voxels; "
        f"bounds x={bounds[0].tolist()} y={bounds[1].tolist()} z={bounds[2].tolist()}",
        flush=True,
    )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output", type=Path, default=Path("reachability_output"))
    parser.add_argument("--mode", choices=("arm-only", "whole-body", "both"), default="both")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="both")
    parser.add_argument(
        "--sample-power",
        type=int,
        default=20,
        help="samples per arm are 2**N (default: 20 = 1,048,576)",
    )
    parser.add_argument("--voxel-mm", type=float, default=20.0)
    parser.add_argument("--joint-margin-deg", type=float, default=0.0)
    parser.add_argument("--chunk-size", type=int, default=65536)
    args = parser.parse_args()
    if not 8 <= args.sample_power <= 27:
        parser.error("--sample-power must be in [8, 27]")
    if not 1.0 <= args.voxel_mm <= 100.0:
        parser.error("--voxel-mm must be in [1, 100]")
    if args.chunk_size <= 0:
        parser.error("--chunk-size must be positive")
    if args.joint_margin_deg < 0:
        parser.error("--joint-margin-deg cannot be negative")
    return args


def main() -> int:
    args = parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.is_file():
        print(f"URDF does not exist: {urdf_path}", file=sys.stderr)
        return 2
    output_dir = args.output.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    modes = ("arm-only", "whole-body") if args.mode == "both" else (args.mode,)
    arms = ARMS if args.arm == "both" else (args.arm,)
    voxel_size = args.voxel_mm / 1000.0
    summaries: list[dict[str, object]] = []
    for mode in modes:
        for arm in arms:
            summaries.append(
                build_arm_map(
                    arm=arm,
                    mode=mode,
                    urdf_path=urdf_path,
                    output_dir=output_dir,
                    sample_power=args.sample_power,
                    voxel_size=voxel_size,
                    margin_rad=math.radians(args.joint_margin_deg),
                    chunk_size=args.chunk_size,
                )
            )
    summary = {
        "schema_version": 1,
        "generator": Path(__file__).name,
        "generated_at_unix": time.time(),
        "urdf": str(urdf_path),
        "urdf_sha256": __import__("hashlib").sha256(urdf_path.read_bytes()).hexdigest(),
        "offline_only": True,
        "collision_checked": False,
        "notes": [
            "This is a voxelized approximation of a continuous kinematic workspace.",
            "Joint limits come from the URDF; self-collision and scene obstacles are not filtered.",
            "A point in this map is not permission to command the physical robot.",
        ],
        "maps": summaries,
    }
    summary_path = output_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(f"wrote {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
