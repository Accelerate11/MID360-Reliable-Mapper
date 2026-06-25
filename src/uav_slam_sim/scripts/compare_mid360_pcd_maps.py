#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import numpy as np

from analyze_mid360_pcd_map import analyze, read_ascii_pcd


def voxel_keys(points, voxel):
    if points.size == 0:
        return set()
    keys = np.floor(points[:, :3] / voxel).astype(np.int64)
    return {tuple(row) for row in keys}


def compare_maps(points_a, points_b, path_a, path_b, voxel_size):
    report = {
        "map_a": str(Path(path_a).resolve()),
        "map_b": str(Path(path_b).resolve()),
        "voxel_size": float(voxel_size),
        "a": analyze(points_a, path_a, [voxel_size]),
        "b": analyze(points_b, path_b, [voxel_size]),
    }

    if points_a.size == 0 or points_b.size == 0:
        report["overlap"] = {
            "intersection_voxels": 0,
            "union_voxels": 0,
            "jaccard": 0.0,
            "a_coverage_by_b": 0.0,
            "b_coverage_by_a": 0.0,
        }
        return report

    keys_a = voxel_keys(points_a, voxel_size)
    keys_b = voxel_keys(points_b, voxel_size)
    intersection = keys_a & keys_b
    union = keys_a | keys_b

    centroid_a = points_a[:, :3].mean(axis=0)
    centroid_b = points_b[:, :3].mean(axis=0)
    centroid_delta = centroid_b - centroid_a

    report.update({
        "centroid_delta_b_minus_a_m": [float(v) for v in centroid_delta],
        "overlap": {
            "intersection_voxels": len(intersection),
            "union_voxels": len(union),
            "jaccard": len(intersection) / float(max(len(union), 1)),
            "a_coverage_by_b": len(intersection) / float(max(len(keys_a), 1)),
            "b_coverage_by_a": len(intersection) / float(max(len(keys_b), 1)),
        },
        "point_count_ratio_b_over_a": (
            float(points_b.shape[0]) / float(points_a.shape[0])
            if points_a.shape[0] else 0.0
        ),
    })
    return report


def write_markdown(report, output):
    lines = [
        "# MID360 Map Comparison",
        "",
        f"- map_a: `{report['map_a']}`",
        f"- map_b: `{report['map_b']}`",
        f"- voxel_size: `{report['voxel_size']:.3f}` m",
        "",
        "## Counts",
        "",
        f"- a_points: `{report['a'].get('points', 0)}`",
        f"- b_points: `{report['b'].get('points', 0)}`",
        f"- point_count_ratio_b_over_a: `{report.get('point_count_ratio_b_over_a', 0.0):.3f}`",
        "",
        "## Overlap",
        "",
    ]
    overlap = report.get("overlap", {})
    lines.extend([
        f"- intersection_voxels: `{overlap.get('intersection_voxels', 0)}`",
        f"- union_voxels: `{overlap.get('union_voxels', 0)}`",
        f"- jaccard: `{overlap.get('jaccard', 0.0):.3f}`",
        f"- a_coverage_by_b: `{overlap.get('a_coverage_by_b', 0.0):.3f}`",
        f"- b_coverage_by_a: `{overlap.get('b_coverage_by_a', 0.0):.3f}`",
        "",
    ])

    if "centroid_delta_b_minus_a_m" in report:
        dx, dy, dz = report["centroid_delta_b_minus_a_m"]
        lines.append("## Centroid Delta")
        lines.append("")
        lines.append(f"- b_minus_a_m: x=`{dx:.3f}`, y=`{dy:.3f}`, z=`{dz:.3f}`")
        lines.append("")

    output.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Compare two MID360 ascii PCD maps with voxel overlap metrics.")
    parser.add_argument("map_a")
    parser.add_argument("map_b")
    parser.add_argument("--voxel-size", type=float, default=0.20)
    parser.add_argument("--report", default="")
    parser.add_argument("--markdown", default="")
    args = parser.parse_args()

    points_a, _ = read_ascii_pcd(args.map_a)
    points_b, _ = read_ascii_pcd(args.map_b)
    report = compare_maps(points_a, points_b, args.map_a, args.map_b, args.voxel_size)

    report_path = Path(args.report) if args.report else Path(args.map_b).with_suffix(".compare.json")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    if args.markdown:
        markdown_path = Path(args.markdown)
        markdown_path.parent.mkdir(parents=True, exist_ok=True)
        write_markdown(report, markdown_path)
        print(f"Saved markdown: {markdown_path}")

    overlap = report.get("overlap", {})
    print(f"Saved comparison: {report_path}")
    print(
        "overlap "
        f"jaccard={overlap.get('jaccard', 0.0):.3f} "
        f"a_coverage_by_b={overlap.get('a_coverage_by_b', 0.0):.3f} "
        f"b_coverage_by_a={overlap.get('b_coverage_by_a', 0.0):.3f}"
    )


if __name__ == "__main__":
    main()
