#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image


SCRIPT_DIR = Path(__file__).resolve().parent

DEFAULT_PCD_FILE = (
    SCRIPT_DIR
    / "dependencies_and_tools"
    / "FAST_LIVO2_relocation_revise"
    / "src"
    / "FAST-LIVO2"
    / "Log"
    / "PCD"
    / "all_raw_points.pcd"
)

DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "robot_functionality" / "leg_bringup" / "maps"

OCCUPIED = 0
FREE = 254
UNKNOWN = 205


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate a Nav2 PGM/YAML map from a FAST-LIVO PCD map."
    )
    parser.add_argument(
        "pcd_file",
        nargs="?",
        default=str(DEFAULT_PCD_FILE),
        help="Input PCD file. Default: FAST-LIVO Log/PCD/all_raw_points.pcd",
    )
    parser.add_argument(
        "--output_dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Output directory. Default: robot_functionality/leg_bringup/maps",
    )
    parser.add_argument(
        "--map_name",
        default="test_map",
        help="Output map name without extension. Default: test_map",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.03,
        help="Map resolution in meters per pixel. Default: 0.05",
    )

    # These defaults mirror mapping_3D_and_2D.launch.py.
    parser.add_argument(
        "--min_z",
        type=float,
        default=0.50,
        help="Minimum z used from the PCD. Default: -0.20",
    )
    parser.add_argument(
        "--max_z",
        type=float,
        default=1.00,
        help="Maximum z used from the PCD. Default: 1.80",
    )
    parser.add_argument(
        "--bounds_percentile",
        type=float,
        default=0.2,
        help="Trim this XY percentile at each map edge to ignore outliers. 0 disables. Default: 0.2",
    )
    parser.add_argument(
        "--padding",
        type=float,
        default=0.5,
        help="Extra map padding in meters. Default: 0.5",
    )
    parser.add_argument(
        "--crop_radius",
        type=float,
        default=0.0,
        help="Optional XY crop radius around the cloud median. 0 disables. Default: 0",
    )

    # Occupancy rules.
    parser.add_argument(
        "--min_points_per_cell",
        type=int,
        default=3,
        help="Minimum points in a grid cell before using its z statistics. Default: 3",
    )
    parser.add_argument(
        "--wall_min_height",
        type=float,
        default=0.60,
        help="Mark a cell occupied if max_z - min_z is at least this height. Default: 0.60",
    )
    parser.add_argument(
        "--max_step_height",
        type=float,
        default=0.35,
        help=(
            "Mark local height discontinuities larger than this value as occupied. "
            "Raise it if stairs/ramps should be traversable. Default: 0.35"
        ),
    )
    parser.add_argument(
        "--mark_height_jumps",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable/disable occupancy from neighboring min-z height jumps. Default: enabled",
    )
    parser.add_argument(
        "--neighbor_radius",
        type=float,
        default=0.10,
        help="Radius used for sparse noise filtering around occupied cells. Default: 0.10",
    )
    parser.add_argument(
        "--min_neighbor_points",
        type=int,
        default=8,
        help="Minimum points in a cell neighborhood before it can be occupied. Default: 8",
    )

    # Image post-processing.
    parser.add_argument(
        "--inflate_radius",
        type=float,
        default=0.05,
        help="Inflate occupied cells by this radius in meters. Default: 0.05",
    )
    parser.add_argument(
        "--close_radius",
        type=float,
        default=0.0,
        help="Close tiny gaps between occupied cells by this radius in meters. Default: 0.0",
    )
    parser.add_argument(
        "--unknown",
        action="store_true",
        help="Write unobserved cells as 205 unknown instead of free.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Also save debug images for cell height span and min-z elevation.",
    )
    return parser.parse_args()


def clamp_percentile(value):
    return min(max(float(value), 0.0), 49.0)


def percentile_bounds(values, padding, bounds_percentile):
    pct = clamp_percentile(bounds_percentile)
    if pct == 0.0:
        return float(np.min(values) - padding), float(np.max(values) + padding)
    low, high = np.percentile(values, [pct, 100.0 - pct])
    return float(low - padding), float(high + padding)


def shifted_views(mask, dy, dx):
    height, width = mask.shape
    src_y0 = max(0, -dy)
    src_y1 = min(height, height - dy)
    src_x0 = max(0, -dx)
    src_x1 = min(width, width - dx)
    dst_y0 = max(0, dy)
    dst_y1 = min(height, height + dy)
    dst_x0 = max(0, dx)
    dst_x1 = min(width, width + dx)
    return (src_y0, src_y1, src_x0, src_x1, dst_y0, dst_y1, dst_x0, dst_x1)


def disk_offsets(radius_cells):
    if radius_cells <= 0:
        return [(0, 0)]
    offsets = []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                offsets.append((dy, dx))
    return offsets


def dilate(mask, radius_cells):
    if radius_cells <= 0:
        return mask
    result = mask.copy()
    for dy, dx in disk_offsets(radius_cells):
        if dy == 0 and dx == 0:
            continue
        sy0, sy1, sx0, sx1, dy0, dy1, dx0, dx1 = shifted_views(mask, dy, dx)
        result[dy0:dy1, dx0:dx1] |= mask[sy0:sy1, sx0:sx1]
    return result


def erode(mask, radius_cells):
    if radius_cells <= 0:
        return mask
    result = mask.copy()
    for dy, dx in disk_offsets(radius_cells):
        if dy == 0 and dx == 0:
            continue
        shifted = np.zeros_like(mask, dtype=bool)
        sy0, sy1, sx0, sx1, dy0, dy1, dx0, dx1 = shifted_views(mask, dy, dx)
        shifted[dy0:dy1, dx0:dx1] = mask[sy0:sy1, sx0:sx1]
        result &= shifted
    return result


def close(mask, radius_cells):
    return erode(dilate(mask, radius_cells), radius_cells)


def neighbor_sum(count_grid, radius_cells):
    if radius_cells <= 0:
        return count_grid
    summed = np.zeros_like(count_grid, dtype=np.int32)
    for dy, dx in disk_offsets(radius_cells):
        sy0, sy1, sx0, sx1, dy0, dy1, dx0, dx1 = shifted_views(count_grid, dy, dx)
        summed[dy0:dy1, dx0:dx1] += count_grid[sy0:sy1, sx0:sx1]
    return summed


def height_jump_mask(min_z, valid):
    jump = np.zeros_like(valid, dtype=bool)
    for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
        sy0, sy1, sx0, sx1, dy0, dy1, dx0, dx1 = shifted_views(valid, dy, dx)
        both_valid = valid[dy0:dy1, dx0:dx1] & valid[sy0:sy1, sx0:sx1]
        diff = np.zeros_like(both_valid, dtype=np.float32)
        diff[both_valid] = np.abs(min_z[dy0:dy1, dx0:dx1][both_valid] - min_z[sy0:sy1, sx0:sx1][both_valid])
        jump[dy0:dy1, dx0:dx1] |= both_valid & (diff > 0.0)
    return jump


def write_yaml(yaml_path, pgm_filename, resolution, origin_x, origin_y):
    yaml_path.write_text(
        f"""image: {pgm_filename}
mode: trinary
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    )


def save_debug_image(path, values, valid, low=None, high=None):
    img = np.zeros(values.shape, dtype=np.uint8)
    if np.any(valid):
        vals = values[valid]
        if low is None:
            low = float(np.percentile(vals, 2.0))
        if high is None:
            high = float(np.percentile(vals, 98.0))
        if high <= low:
            high = low + 1e-6
        scaled = (np.clip(values, low, high) - low) / (high - low)
        img[valid] = (scaled[valid] * 254).astype(np.uint8)
    Image.fromarray(255 - img, mode="L").save(path)


def main():
    args = parse_args()
    pcd_path = Path(args.pcd_file).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()

    if not pcd_path.exists():
        raise FileNotFoundError(f"PCD file not found: {pcd_path}")
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Input PCD: {pcd_path}")
    print(f"[INFO] Output dir: {output_dir}")

    pcd = o3d.io.read_point_cloud(str(pcd_path))
    points = np.asarray(pcd.points)
    if points.size == 0:
        raise RuntimeError("The PCD file has no points.")

    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        raise RuntimeError("No finite points in PCD.")

    print(f"[INFO] Valid points: {len(points)}")
    for name, index in [("x", 0), ("y", 1), ("z", 2)]:
        qs = np.percentile(points[:, index], [0, 1, 5, 50, 95, 99, 100])
        print(
            f"[INFO] {name} percentiles: "
            f"min={qs[0]:.3f}, p01={qs[1]:.3f}, p05={qs[2]:.3f}, "
            f"p50={qs[3]:.3f}, p95={qs[4]:.3f}, p99={qs[5]:.3f}, max={qs[6]:.3f}"
        )

    if args.crop_radius > 0.0:
        center_x = float(np.median(points[:, 0]))
        center_y = float(np.median(points[:, 1]))
        dx = points[:, 0] - center_x
        dy = points[:, 1] - center_y
        points = points[(dx * dx + dy * dy) <= args.crop_radius * args.crop_radius]
        print(f"[INFO] Points after crop_radius={args.crop_radius}: {len(points)}")
        if len(points) == 0:
            raise RuntimeError("No points left after crop.")

    z_mask = (points[:, 2] >= args.min_z) & (points[:, 2] <= args.max_z)
    points = points[z_mask]
    print(f"[INFO] Points inside z window [{args.min_z}, {args.max_z}]: {len(points)}")
    if len(points) == 0:
        raise RuntimeError("No points left inside z window.")

    min_x, max_x = percentile_bounds(points[:, 0], args.padding, args.bounds_percentile)
    min_y, max_y = percentile_bounds(points[:, 1], args.padding, args.bounds_percentile)
    width = int(math.ceil((max_x - min_x) / args.resolution))
    height = int(math.ceil((max_y - min_y) / args.resolution))
    if width <= 0 or height <= 0:
        raise RuntimeError("Invalid map size.")

    print(f"[INFO] Map bounds x: {min_x:.3f} ~ {max_x:.3f}")
    print(f"[INFO] Map bounds y: {min_y:.3f} ~ {max_y:.3f}")
    print(f"[INFO] Map size: {width} x {height}, resolution={args.resolution}")

    ix = ((points[:, 0] - min_x) / args.resolution).astype(np.int32)
    iy = ((points[:, 1] - min_y) / args.resolution).astype(np.int32)
    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    ix = ix[valid]
    iy = iy[valid]
    z = points[:, 2][valid]

    # Image rows are top-to-bottom; map Y is bottom-to-top.
    row = height - 1 - iy
    flat_ids = row * width + ix
    flat_size = width * height

    count = np.bincount(flat_ids, minlength=flat_size).astype(np.int32)
    min_z = np.full(flat_size, np.inf, dtype=np.float32)
    max_z = np.full(flat_size, -np.inf, dtype=np.float32)
    np.minimum.at(min_z, flat_ids, z)
    np.maximum.at(max_z, flat_ids, z)

    count_grid = count.reshape(height, width)
    min_z_grid = min_z.reshape(height, width)
    max_z_grid = max_z.reshape(height, width)
    valid_cell = count_grid >= max(args.min_points_per_cell, 1)
    z_span = max_z_grid - min_z_grid

    neighbor_cells = int(math.ceil(args.neighbor_radius / args.resolution))
    neighbor_points = neighbor_sum(count_grid, neighbor_cells)
    dense_enough = neighbor_points >= max(args.min_neighbor_points, 1)

    wall_like = valid_cell & dense_enough & (z_span >= args.wall_min_height)

    if args.mark_height_jumps:
        jump_like = dense_enough & (max_neighbor_height_jump(min_z_grid, valid_cell) >= args.max_step_height)
    else:
        jump_like = np.zeros_like(valid_cell, dtype=bool)

    occupied = wall_like | jump_like

    close_cells = int(math.ceil(args.close_radius / args.resolution))
    inflate_cells = int(math.ceil(args.inflate_radius / args.resolution))
    if close_cells > 0:
        occupied = close(occupied, close_cells)
    if inflate_cells > 0:
        occupied = dilate(occupied, inflate_cells)

    base_value = UNKNOWN if args.unknown else FREE
    grid = np.full((height, width), base_value, dtype=np.uint8)
    if args.unknown:
        # Cells with any observed point are known free unless occupied.
        grid[count_grid > 0] = FREE
    grid[occupied] = OCCUPIED

    occupied_count = int(np.count_nonzero(occupied))
    observed_count = int(np.count_nonzero(count_grid > 0))
    print(f"[INFO] Observed cells: {observed_count} ({observed_count / flat_size * 100.0:.2f}%)")
    print(f"[INFO] Wall-like cells before postprocess: {int(np.count_nonzero(wall_like))}")
    print(f"[INFO] Height-jump cells before postprocess: {int(np.count_nonzero(jump_like))}")
    print(f"[INFO] Occupied cells final: {occupied_count} ({occupied_count / flat_size * 100.0:.2f}%)")

    pgm_path = output_dir / f"{args.map_name}.pgm"
    yaml_path = output_dir / f"{args.map_name}.yaml"
    Image.fromarray(grid, mode="L").save(pgm_path)
    write_yaml(yaml_path, pgm_path.name, args.resolution, min_x, min_y)

    if args.debug:
        debug_span = output_dir / f"{args.map_name}_debug_span.pgm"
        debug_min_z = output_dir / f"{args.map_name}_debug_min_z.pgm"
        save_debug_image(debug_span, z_span, valid_cell, low=0.0, high=max(args.wall_min_height * 2.0, 0.1))
        save_debug_image(debug_min_z, min_z_grid, valid_cell)
        print(f"[INFO] Debug span image: {debug_span}")
        print(f"[INFO] Debug min-z image: {debug_min_z}")

    print("[DONE] Nav2 static map generated:")
    print(f"       PGM : {pgm_path}")
    print(f"       YAML: {yaml_path}")


def max_neighbor_height_jump(min_z, valid):
    max_jump = np.zeros_like(min_z, dtype=np.float32)
    for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
        sy0, sy1, sx0, sx1, dy0, dy1, dx0, dx1 = shifted_views(valid, dy, dx)
        both_valid = valid[dy0:dy1, dx0:dx1] & valid[sy0:sy1, sx0:sx1]
        diff = np.zeros_like(both_valid, dtype=np.float32)
        diff[both_valid] = np.abs(min_z[dy0:dy1, dx0:dx1][both_valid] - min_z[sy0:sy1, sx0:sx1][both_valid])
        max_jump[dy0:dy1, dx0:dx1] = np.maximum(max_jump[dy0:dy1, dx0:dx1], diff)
    return max_jump


if __name__ == "__main__":
    main()
