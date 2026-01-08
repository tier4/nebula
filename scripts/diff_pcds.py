#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "matplotlib",
#     "numpy",
#     "open3d",
# ]
# ///

"""
This script visualizes the difference between two point clouds.

Difference here is defined as the nearest neighbor distance for each point in the second point cloud
to the first point cloud.

The output is a pseudo range image (an azimuth/elevation scatter plot) where each point is colored
by that difference.

The script also prints statistics, such as
* the number of points in each point cloud
* the minimum and maximum distance difference

Limitations:

    Since the difference is based on nearest neighbor distance, pointclouds have to be similar in
    size and transformation for the comparison to be meaningful.

Usage:

    ```bash
        ./scripts/diff_pcds.py <pointcloud1.pcd> <pointcloud2.pcd>
    ```

Usage as a Git difftool:

    ```bash
    git difftool -y --extcmd=./scripts/diff_pcds.py <ref> <ref> -- <path/to/pointcloud.pcd>
    ```
"""

from argparse import ArgumentParser
from pathlib import Path

from matplotlib import pyplot as plt
from matplotlib.colors import SymLogNorm
import numpy as np
import open3d as o3d


def print_stats(pts1, pts2, dists):
    print(f"PCD1: {len(pts1)} points")
    print(f"PCD2: {len(pts2)} points")

    if len(pts1) != len(pts2):
        print(f"Diff: {len(pts1) - len(pts2)} points")
    else:
        print("Same size")

    dist_min = dists.min()
    dist_max = dists.max()
    print(f"Distance range (mm): [{dist_min * 1e3:.3f}, {dist_max * 1e3:.3f}]")


def to_polar(pts):
    x = pts[:, 0]
    y = pts[:, 1]
    z = pts[:, 2]
    r = np.sqrt(x * x + y * y + z * z)

    # Sensor angle conventions are assumed to be:
    # Azimuth: angle in XY plane from Y axis toward X axis
    azimuths = np.arctan2(x, y)
    elevations = np.arcsin(z / r)

    # Convert to degrees for easier reading
    azimuths_deg = np.degrees(azimuths)
    elevations_deg = np.degrees(elevations)

    return azimuths_deg, elevations_deg


def plot_diff(pts1, pts2, dists, pcd1_path: str, pcd2_path: str):
    # Setup plot
    # 16:9 aspect ratio
    fig_width = 16
    fig_height = 9
    plt.figure(figsize=(fig_width, fig_height))

    azimuths1, elevations1 = to_polar(pts1)
    plt.scatter(azimuths1, elevations1, c="grey", s=1, alpha=0.8)

    azimuths_deg, elevations_deg = to_polar(pts2)
    dists_mm = dists * 1000
    c = dists_mm
    vmax = dists_mm.max()

    scatter = plt.scatter(
        azimuths_deg,
        elevations_deg,
        c=c,
        cmap="plasma",
        s=1,
        alpha=0.8,
        norm=SymLogNorm(linthresh=1e-3, vmin=0, vmax=vmax, clip=True),
    )

    plt.xlabel("Azimuth (degrees)")
    plt.ylabel("Elevation (degrees)")
    plt.title(
        f"Pseudo Range Image Diff\n" 
        f"{Path(pcd1_path).name} vs {Path(pcd2_path).name}\n"
        f"Distance range (mm): [{dists_mm.min():.3f}, {dists_mm.max():.3f}]"
    )

    cbar = plt.colorbar(scatter)
    cbar.set_label("Distance Difference [mm]")

    plt.axis("tight")
    plt.grid(True, linestyle="--", alpha=0.5)

    out_png = f"diff_{Path(pcd1_path).stem}_{Path(pcd2_path).stem}.png"

    plt.savefig(out_png, dpi=150)
    print(f"Saved {out_png}")


def calculate_dists(pcd1, pcd2):
    pcd1_tree = o3d.geometry.KDTreeFlann(pcd1)
    dists = []
    for pt in pcd2.points:
        [_, _, dist_sq] = pcd1_tree.search_knn_vector_3d(pt, 1)
        dist = np.sqrt(dist_sq[0])
        dists.append(dist)
    return np.asarray(dists)


def main(pcd1_path: str, pcd2_path: str, visualize: bool):
    pcd1 = o3d.io.read_point_cloud(pcd1_path)
    pcd2 = o3d.io.read_point_cloud(pcd2_path)

    pts1 = np.asarray(pcd1.points)
    pts2 = np.asarray(pcd2.points)

    dists = calculate_dists(pcd1, pcd2)

    if visualize:
        plot_diff(pts1, pts2, dists, pcd1_path, pcd2_path)

    print_stats(pts1, pts2, dists)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("pcd1", type=str, help="Path to first point cloud")
    parser.add_argument("pcd2", type=str, help="Path to second point cloud")
    parser.add_argument(
        "--no-visualize", action="store_true", help="Do not visualize the point cloud"
    )
    args = parser.parse_args()

    print(f"Comparing {args.pcd1} to {args.pcd2}")

    main(args.pcd1, args.pcd2, not args.no_visualize)
