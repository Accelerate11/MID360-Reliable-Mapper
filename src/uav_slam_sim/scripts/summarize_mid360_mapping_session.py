#!/usr/bin/env python3
import argparse
import json
import re
from pathlib import Path


def load_json(path):
    if not path.exists():
        return None
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def parse_bag_metadata(path):
    if not path.exists():
        return {}

    text = path.read_text(encoding="utf-8", errors="replace")
    result = {}

    duration = re.search(r"duration:\s*\n\s+nanoseconds:\s+(\d+)", text)
    if duration:
        result["duration_sec"] = int(duration.group(1)) / 1e9

    message_count = re.search(r"message_count:\s+(\d+)", text)
    if message_count:
        result["message_count"] = int(message_count.group(1))

    topics = []
    for match in re.finditer(
        r"topic_metadata:\s*\n\s+name:\s+([^\n]+)\n\s+type:\s+([^\n]+).*?message_count:\s+(\d+)",
        text,
        flags=re.DOTALL,
    ):
        topics.append({
            "name": match.group(1).strip(),
            "type": match.group(2).strip(),
            "count": int(match.group(3)),
        })
    result["topics"] = topics
    return result


def find_first(root, pattern):
    matches = sorted(root.glob(pattern))
    return matches[0] if matches else None


def append_map_report(lines, title, report):
    lines.append(f"## {title}")
    lines.append("")
    if not report:
        lines.append("No map report found.")
        lines.append("")
        return

    lines.append(f"- topic: `{report.get('topic', '')}`")
    lines.append(f"- frame: `{report.get('frame_id', '')}`")
    lines.append(f"- points: `{report.get('points', 0)}`")
    span = report.get("span_m") or {}
    bounds = report.get("bounds") or {}
    if span:
        lines.append(
            "- span_m: "
            f"x=`{span.get('x', 0):.3f}`, "
            f"y=`{span.get('y', 0):.3f}`, "
            f"z=`{span.get('z', 0):.3f}`"
        )
    if bounds:
        lines.append(f"- bounds: `{bounds}`")
    lines.append("")


def append_analysis_report(lines, report):
    lines.append("## Map Analysis")
    lines.append("")
    if not report:
        lines.append("No map analysis found.")
        lines.append("")
        return

    lines.append(f"- points: `{report.get('points', 0)}`")
    span = report.get("span_m") or {}
    if span:
        lines.append(
            "- span_m: "
            f"x=`{span.get('x', 0):.3f}`, "
            f"y=`{span.get('y', 0):.3f}`, "
            f"z=`{span.get('z', 0):.3f}`"
        )

    z_percentiles = report.get("z_percentiles") or {}
    if z_percentiles:
        lines.append(
            "- z_percentiles: "
            f"p01=`{z_percentiles.get('p01', 0):.3f}`, "
            f"p50=`{z_percentiles.get('p50', 0):.3f}`, "
            f"p99=`{z_percentiles.get('p99', 0):.3f}`"
        )

    voxel_stats = report.get("voxel_stats") or []
    if voxel_stats:
        lines.append("")
        lines.append("| Voxel (m) | Occupied Voxels | Points / Voxel |")
        lines.append("| ---: | ---: | ---: |")
        for item in voxel_stats:
            lines.append(
                f"| {item.get('voxel_size', 0):.2f} | "
                f"{item.get('occupied_voxels', 0)} | "
                f"{item.get('points_per_voxel', 0):.2f} |"
            )

    if report.get("snapshot"):
        lines.append("")
        lines.append(f"- snapshot: `{report['snapshot']}`")
    lines.append("")


def append_readiness_report(lines, report):
    lines.append("## Mapping Readiness")
    lines.append("")
    if not report:
        lines.append("No readiness report found.")
        lines.append("")
        return

    lines.append(f"- verdict: `{report.get('verdict', '')}`")
    lines.append(f"- score: `{report.get('score', 0)}`")
    summary = report.get("summary") or {}
    if summary:
        lines.append(f"- points: `{summary.get('points', 0)}`")
        lines.append(f"- xy_span_m: `{summary.get('xy_span_m', 0.0):.3f}`")
        lines.append(f"- z_span_m: `{summary.get('z_span_m', 0.0):.3f}`")
        lines.append(f"- occupied_voxels_20cm: `{summary.get('occupied_voxels_20cm', 0)}`")

    issues = report.get("issues") or []
    if issues:
        lines.append("")
        lines.append("| Level | Code | Message |")
        lines.append("| --- | --- | --- |")
        for issue in issues:
            lines.append(f"| `{issue.get('level', '')}` | `{issue.get('code', '')}` | {issue.get('message', '')} |")
    lines.append("")


def main():
    parser = argparse.ArgumentParser(description="Create a markdown summary for a MID360 mapping session.")
    parser.add_argument("session_dir")
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    session_dir = Path(args.session_dir).resolve()
    output = Path(args.output).resolve() if args.output else session_dir / "summary.md"

    map_report_candidates = [
        path for path in sorted((session_dir / "maps").glob("*.json"))
        if not path.name.endswith(".analysis.json") and not path.name.endswith(".compare.json")
    ]
    map_report = map_report_candidates[0] if map_report_candidates else None
    analysis_report = find_first(session_dir, "maps/*.analysis.json")
    quality_path = session_dir / "logs" / "final_quality.txt"
    bag_metadata = find_first(session_dir, "bag/*/metadata.yaml")
    source_bag_path = session_dir / "source_bag.txt"
    readiness_path = session_dir / "mapping_readiness.json"

    map_data = load_json(map_report) if map_report else None
    analysis_data = load_json(analysis_report) if analysis_report else None
    bag_data = parse_bag_metadata(bag_metadata) if bag_metadata else {}
    readiness_data = load_json(readiness_path) if readiness_path.exists() else None

    lines = [
        "# MID360 Mapping Session Summary",
        "",
        f"- session: `{session_dir}`",
        f"- map_report: `{map_report}`" if map_report else "- map_report: missing",
        f"- analysis_report: `{analysis_report}`" if analysis_report else "- analysis_report: missing",
        f"- readiness_report: `{readiness_path}`" if readiness_path.exists() else "- readiness_report: missing",
        f"- quality_log: `{quality_path}`" if quality_path.exists() else "- quality_log: missing",
        f"- bag_metadata: `{bag_metadata}`" if bag_metadata else "- bag_metadata: missing",
        f"- source_bag: `{source_bag_path.read_text(encoding='utf-8').strip()}`"
        if source_bag_path.exists() else "- source_bag: not applicable",
        "",
        "## Bag",
        "",
    ]

    if bag_data:
        lines.append(f"- duration_sec: `{bag_data.get('duration_sec', 0):.3f}`")
        lines.append(f"- messages: `{bag_data.get('message_count', 0)}`")
        lines.append("")
        lines.append("| Topic | Count | Type |")
        lines.append("| --- | ---: | --- |")
        for topic in bag_data.get("topics", []):
            lines.append(f"| `{topic['name']}` | {topic['count']} | `{topic['type']}` |")
        lines.append("")
    else:
        lines.append("No bag metadata found.")
        lines.append("")

    append_map_report(lines, "Map", map_data)
    append_analysis_report(lines, analysis_data)
    append_readiness_report(lines, readiness_data)

    lines.append("## Final Quality")
    lines.append("")
    if quality_path.exists():
        lines.append("```text")
        lines.append(quality_path.read_text(encoding="utf-8", errors="replace").strip())
        lines.append("```")
    else:
        lines.append("No final quality log found.")
    lines.append("")

    output.write_text("\n".join(lines), encoding="utf-8")
    print(f"Saved summary: {output}")


if __name__ == "__main__":
    main()
