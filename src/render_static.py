#!/usr/bin/env python3
"""Render publication-ready PNG previews from G1 reachability NPZ files."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


COLORS = {
    "background": (7, 19, 29, 255),
    "panel": (12, 29, 41, 255),
    "grid": (36, 73, 93, 185),
    "text": (234, 247, 245, 255),
    "muted": (154, 176, 187, 255),
    "left": (35, 213, 181, 145),
    "right": (255, 138, 101, 145),
    "robot": (196, 211, 223, 255),
    "x": (242, 84, 91, 255),
    "y": (98, 210, 111, 255),
    "z": (73, 167, 255, 255),
}


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        "/System/Library/Fonts/PingFang.ttc",
        "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for candidate in candidates:
        if Path(candidate).exists():
            try:
                return ImageFont.truetype(candidate, size=size, index=1 if bold and "PingFang" in candidate else 0)
            except OSError:
                continue
    return ImageFont.load_default()


def project(points: np.ndarray, yaw_deg: float, elevation_deg: float) -> tuple[np.ndarray, np.ndarray]:
    yaw = math.radians(yaw_deg)
    elevation = math.radians(elevation_deg)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    horizontal = np.cos(yaw) * x - np.sin(yaw) * y
    depth_plane = np.sin(yaw) * x + np.cos(yaw) * y
    vertical = np.cos(elevation) * z - np.sin(elevation) * depth_plane
    depth = np.sin(elevation) * z + np.cos(elevation) * depth_plane
    return np.column_stack((horizontal, vertical)), depth


def load_centers(data_dir: Path, mode: str, arm: str, limit: int, seed: int) -> np.ndarray:
    data = np.load(data_dir / f"{mode}_{arm}_020mm.npz", allow_pickle=False)
    points = data["centers_m"].astype(np.float64)
    if len(points) <= limit:
        return points
    rng = np.random.default_rng(seed)
    return points[rng.choice(len(points), limit, replace=False)]


def map_pixels(projected: np.ndarray, bounds: tuple[np.ndarray, np.ndarray], rect: tuple[int, int, int, int]) -> np.ndarray:
    minimum, maximum = bounds
    left, top, right, bottom = rect
    span = np.maximum(maximum - minimum, 1e-9)
    x = left + (projected[:, 0] - minimum[0]) / span[0] * (right - left)
    y = bottom - (projected[:, 1] - minimum[1]) / span[1] * (bottom - top)
    return np.column_stack((x, y))


def render(mode: str, data_dir: Path, segments_path: Path, output: Path, width: int = 1800, height: int = 1120) -> None:
    left_points = load_centers(data_dir, mode, "left", 150_000, 14)
    right_points = load_centers(data_dir, mode, "right", 150_000, 29)
    robot_data = json.loads(segments_path.read_text(encoding="utf-8"))[mode]
    segments = np.asarray(robot_data["segments"], dtype=np.float64)
    robot_points = segments.reshape(-1, 3) if len(segments) else np.zeros((0, 3))

    yaw, elevation = (-42.0, 22.0) if mode == "arm-only" else (-38.0, 18.0)
    all_points = np.vstack((left_points, right_points, robot_points))
    projected_all, _ = project(all_points, yaw, elevation)
    minimum, maximum = projected_all.min(axis=0), projected_all.max(axis=0)
    padding = (maximum - minimum) * 0.09
    bounds = (minimum - padding, maximum + padding)
    plot_rect = (90, 190, width - 90, height - 100)

    canvas = Image.new("RGBA", (width, height), COLORS["background"])
    draw = ImageDraw.Draw(canvas)
    draw.rounded_rectangle((50, 155, width - 50, height - 55), radius=24, fill=COLORS["panel"])

    title = "G1 纯机械臂 TCP 可达空间" if mode == "arm-only" else "G1 上身机构＋机械臂 TCP 可达空间"
    frame = "torso_base_link" if mode == "arm-only" else "base_link"
    draw.text((62, 42), title, fill=COLORS["text"], font=font(42, True))
    draw.text((64, 101), f"坐标系：{frame}  ·  体素：20 mm  ·  末端：gripper_tcp_link", fill=COLORS["muted"], font=font(24))

    grid_extent = 2.4 if mode == "whole-body" else 1.35
    grid_values = np.arange(-grid_extent, grid_extent + 0.001, 0.2)
    for value in grid_values:
        for line_3d in (
            np.array([[value, -grid_extent, 0], [value, grid_extent, 0]]),
            np.array([[-grid_extent, value, 0], [grid_extent, value, 0]]),
        ):
            line, _ = project(line_3d, yaw, elevation)
            pixels = map_pixels(line, bounds, plot_rect)
            draw.line([tuple(pixels[0]), tuple(pixels[1])], fill=COLORS["grid"], width=1)

    point_layer = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    point_draw = ImageDraw.Draw(point_layer)
    for points, color in ((left_points, COLORS["left"]), (right_points, COLORS["right"])):
        projected, _ = project(points, yaw, elevation)
        pixels = map_pixels(projected, bounds, plot_rect).astype(np.int32)
        pixel_list = [tuple(item) for item in pixels]
        point_draw.point(pixel_list, fill=color)
        point_draw.point([(x + 1, y) for x, y in pixel_list], fill=color)
    canvas = Image.alpha_composite(canvas, point_layer)
    draw = ImageDraw.Draw(canvas)

    if len(segments):
        for segment in segments:
            projected, _ = project(segment, yaw, elevation)
            pixels = map_pixels(projected, bounds, plot_rect)
            draw.line([tuple(pixels[0]), tuple(pixels[1])], fill=COLORS["robot"], width=18)
            for pixel in pixels:
                x, y = pixel
                draw.ellipse((x - 11, y - 11, x + 11, y + 11), fill=COLORS["robot"])

    torso = np.asarray([robot_data["torso"]], dtype=float)
    torso_pixel = map_pixels(project(torso, yaw, elevation)[0], bounds, plot_rect)[0]
    tx, ty = torso_pixel
    draw.rounded_rectangle((tx - 58, ty - 45, tx + 58, ty + 45), radius=14, fill=COLORS["robot"])
    if mode == "whole-body":
        base_pixel = map_pixels(project(np.zeros((1, 3)), yaw, elevation)[0], bounds, plot_rect)[0]
        bx, by = base_pixel
        draw.ellipse((bx - 65, by - 24, bx + 65, by + 24), fill=COLORS["robot"])

    origin = np.zeros((1, 3))
    axis_length = 0.42 if mode == "arm-only" else 0.65
    for label, endpoint, color in (
        ("X", np.array([[axis_length, 0, 0]]), COLORS["x"]),
        ("Y", np.array([[0, axis_length, 0]]), COLORS["y"]),
        ("Z", np.array([[0, 0, axis_length]]), COLORS["z"]),
    ):
        line = np.vstack((origin, endpoint))
        pixels = map_pixels(project(line, yaw, elevation)[0], bounds, plot_rect)
        draw.line([tuple(pixels[0]), tuple(pixels[1])], fill=color, width=5)
        draw.text((pixels[1, 0] + 8, pixels[1, 1] - 14), label, fill=color, font=font(22, True))

    legend_y = 172
    for x, color, label in ((85, COLORS["left"], "左臂 TCP"), (245, COLORS["right"], "右臂 TCP"), (410, COLORS["robot"], "G1 连杆模型")):
        draw.ellipse((x, legend_y, x + 18, legend_y + 18), fill=color)
        draw.text((x + 27, legend_y - 7), label, fill=COLORS["text"], font=font(21))
    draw.text((width - 540, legend_y - 7), "点云为完整地图的高密度可视化抽样", fill=COLORS["muted"], font=font(20))

    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.convert("RGB").save(output, quality=94, optimize=True)
    print(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument("--segments", type=Path, default=Path("data/robot_segments.json"))
    parser.add_argument("--output-dir", type=Path, default=Path("docs/images"))
    args = parser.parse_args()
    render("arm-only", args.data_dir, args.segments, args.output_dir / "arm-only-isometric.png")
    render("whole-body", args.data_dir, args.segments, args.output_dir / "whole-body-isometric.png")


if __name__ == "__main__":
    main()
