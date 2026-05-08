#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image


# 当前脚本所在目录
SCRIPT_DIR = Path(__file__).resolve().parent

# 默认输入 PCD：脚本目录/pcd/map.pcd
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

# 默认输出目录：脚本目录/maps
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "robot_functionality" / "leg_bringup" / "maps"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert PCD point cloud map to Nav2 PGM + YAML map."
    )

    parser.add_argument(
        "pcd_file",
        nargs="?",
        default=str(DEFAULT_PCD_FILE),
        help="Input .pcd file. Default: ./pcd/map.pcd relative to this script."
    )

    parser.add_argument(
        "--output_dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Output directory. Default: ./maps relative to this script."
    )

    parser.add_argument(
        "--map_name",
        default="test_map",
        help="Output map name without extension. Default: test_map"
    )

    parser.add_argument(
        "--resolution",
        type=float,
        default=0.05,
        help="Map resolution in meters per pixel. Default: 0.05"
    )

    parser.add_argument(
        "--z_min",
        type=float,
        default=0.10,
        help="Minimum z value treated as obstacle. Default: 0.10"
    )

    parser.add_argument(
        "--z_max",
        type=float,
        default=2.00,
        help="Maximum z value treated as obstacle. Default: 2.00"
    )

    parser.add_argument(
        "--padding",
        type=float,
        default=1.0,
        help="Extra boundary padding in meters. Default: 1.0"
    )

    parser.add_argument(
        "--inflate_radius",
        type=float,
        default=0.10,
        help="Inflate occupied cells by radius in meters. Default: 0.10"
    )

    parser.add_argument(
        "--unknown",
        action="store_true",
        help="Use unknown cells as gray 205. If not set, non-obstacle cells are free."
    )

    return parser.parse_args()


def inflate_occupied(grid, occupied_value, radius_cells):
    if radius_cells <= 0:
        return grid

    inflated = grid.copy()
    occupied_indices = np.argwhere(grid == occupied_value)

    height, width = grid.shape

    offsets = []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                offsets.append((dy, dx))

    for y, x in occupied_indices:
        for dy, dx in offsets:
            ny = y + dy
            nx = x + dx
            if 0 <= ny < height and 0 <= nx < width:
                inflated[ny, nx] = occupied_value

    return inflated


def write_yaml(yaml_path, pgm_filename, resolution, origin_x, origin_y):
    content = f"""image: {pgm_filename}
mode: trinary
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    yaml_path.write_text(content)


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

    print(f"[INFO] Total points: {len(points)}")

    # 去掉 NaN / Inf 点
    valid_mask = np.isfinite(points).all(axis=1)
    points = points[valid_mask]

    if len(points) == 0:
        raise RuntimeError("No valid finite points in PCD.")

    # 用全部点决定地图边界
    min_x = float(np.min(points[:, 0]) - args.padding)
    max_x = float(np.max(points[:, 0]) + args.padding)
    min_y = float(np.min(points[:, 1]) - args.padding)
    max_y = float(np.max(points[:, 1]) + args.padding)

    width = int(math.ceil((max_x - min_x) / args.resolution))
    height = int(math.ceil((max_y - min_y) / args.resolution))

    print(f"[INFO] Map bounds:")
    print(f"       x: {min_x:.3f} ~ {max_x:.3f}")
    print(f"       y: {min_y:.3f} ~ {max_y:.3f}")
    print(f"[INFO] Map size: {width} x {height}")
    print(f"[INFO] Resolution: {args.resolution}")

    if width <= 0 or height <= 0:
        raise RuntimeError("Invalid map size. Please check PCD data and resolution.")

    # PGM 像素值：
    # 0   = occupied / black
    # 254 = free / white
    # 205 = unknown / gray
    occupied_value = 0
    free_value = 254
    unknown_value = 205

    if args.unknown:
        grid = np.full((height, width), unknown_value, dtype=np.uint8)
    else:
        grid = np.full((height, width), free_value, dtype=np.uint8)

    # 按高度过滤障碍物点
    z = points[:, 2]
    obstacle_mask = (z >= args.z_min) & (z <= args.z_max)
    obstacle_points = points[obstacle_mask]

    print(f"[INFO] z_min: {args.z_min}")
    print(f"[INFO] z_max: {args.z_max}")
    print(f"[INFO] Obstacle points after z filter: {len(obstacle_points)}")

    if len(obstacle_points) == 0:
        raise RuntimeError(
            "No obstacle points after z filtering. "
            "Try lowering --z_min or increasing --z_max."
        )

    obs_x = obstacle_points[:, 0]
    obs_y = obstacle_points[:, 1]

    ix = ((obs_x - min_x) / args.resolution).astype(np.int32)
    iy = ((obs_y - min_y) / args.resolution).astype(np.int32)

    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)

    ix = ix[valid]
    iy = iy[valid]

    # ROS map 的 origin 在左下角，但图像坐标原点在左上角，所以 y 要翻转
    img_y = height - 1 - iy

    grid[img_y, ix] = occupied_value

    if args.inflate_radius > 0.0:
        radius_cells = int(math.ceil(args.inflate_radius / args.resolution))
        print(
            f"[INFO] Inflate radius: {args.inflate_radius} m "
            f"= {radius_cells} cells"
        )
        grid = inflate_occupied(grid, occupied_value, radius_cells)

    pgm_path = output_dir / f"{args.map_name}.pgm"
    yaml_path = output_dir / f"{args.map_name}.yaml"

    image = Image.fromarray(grid, mode="L")
    image.save(pgm_path)

    write_yaml(
        yaml_path=yaml_path,
        pgm_filename=pgm_path.name,
        resolution=args.resolution,
        origin_x=min_x,
        origin_y=min_y,
    )

    print("[DONE] Nav2 map generated:")
    print(f"       PGM : {pgm_path}")
    print(f"       YAML: {yaml_path}")


if __name__ == "__main__":
    main()