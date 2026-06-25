#!/usr/bin/env python3
import argparse
import json
import math
import re
from pathlib import Path


RANK = {"PASS": 0, "WARN": 1, "FAIL": 2}


def load_json(path):
    path = Path(path)
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def find_first(root, pattern):
    matches = sorted(Path(root).glob(pattern))
    return matches[0] if matches else None


def find_map_report(session_dir):
    maps_dir = Path(session_dir) / "maps"
    for path in sorted(maps_dir.glob("*.json")):
        if path.name.endswith(".analysis.json"):
            continue
        if path.name.endswith(".compare.json"):
            continue
        return path
    return None


def parse_scalar(value):
    value = value.strip()
    try:
        if re.fullmatch(r"[-+]?\d+", value):
            return int(value)
        return float(value)
    except ValueError:
        return value


def parse_quality_log(path):
    path = Path(path)
    if not path.exists():
        return {}

    topics = {}
    current = None
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.rstrip()
        topic_match = re.match(r"topic:\s*(\S+)", line)
        if topic_match:
            current = topic_match.group(1)
            topics[current] = {}
            continue
        if current is None:
            continue
        item_match = re.match(r"\s+([A-Za-z0-9_]+):\s*(.+)", line)
        if item_match:
            topics[current][item_match.group(1)] = parse_scalar(item_match.group(2))
    return topics


def add_issue(issues, level, code, message, detail=None):
    item = {
        "level": level,
        "code": code,
        "message": message,
    }
    if detail is not None:
        item["detail"] = detail
    issues.append(item)


def worst_verdict(issues):
    verdict = "PASS"
    for issue in issues:
        if RANK[issue["level"]] > RANK[verdict]:
            verdict = issue["level"]
    return verdict


def score_from_issues(issues):
    score = 100
    for issue in issues:
        score -= 30 if issue["level"] == "FAIL" else 10
    return max(0, score)


def get_voxel_stat(analysis, target):
    for item in analysis.get("voxel_stats", []):
        if abs(float(item.get("voxel_size", 0.0)) - target) < 1e-6:
            return item
    return {}


def evaluate_map(issues, analysis, map_report, args):
    if not analysis and not map_report:
        add_issue(issues, "FAIL", "missing_map_report", "No exported map report or analysis report was found.")
        return

    points = 0
    if analysis:
        points = int(analysis.get("points", 0))
    elif map_report:
        points = int(map_report.get("points", 0))

    if points < args.min_points_fail:
        add_issue(issues, "FAIL", "too_few_points", "Map has too few points for mapping validation.", points)
    elif points < args.min_points_warn:
        add_issue(issues, "WARN", "low_point_count", "Map point count is low; coverage may be incomplete.", points)

    span = (analysis or map_report or {}).get("span_m") or {}
    span_x = float(span.get("x", 0.0))
    span_y = float(span.get("y", 0.0))
    span_z = float(span.get("z", 0.0))
    xy_span = max(span_x, span_y)

    if xy_span < args.min_xy_span_fail:
        add_issue(issues, "FAIL", "map_too_small", "XY map span is too small; the sensor probably did not move enough.", span)
    elif xy_span < args.min_xy_span_warn:
        add_issue(issues, "WARN", "limited_xy_coverage", "XY map span is limited for a useful room-scale dataset.", span)

    if xy_span > args.max_xy_span_fail:
        add_issue(issues, "FAIL", "xy_span_unrealistic", "XY map span is unrealistically large; check drift or frame corruption.", span)
    elif xy_span > args.max_xy_span_warn:
        add_issue(issues, "WARN", "large_xy_span", "XY map span is large for the current indoor mapping target.", span)

    if span_z > args.max_z_span_fail:
        add_issue(issues, "FAIL", "z_span_unrealistic", "Z span is unrealistically large; check drift or outliers.", span)
    elif span_z > args.max_z_span_warn:
        add_issue(issues, "WARN", "large_z_span", "Z span is large; ceiling/floor or outliers may dominate the map.", span)

    if analysis:
        voxel_20 = get_voxel_stat(analysis, 0.20)
        occupied = int(voxel_20.get("occupied_voxels", 0))
        if occupied < args.min_occupied_voxels_fail:
            add_issue(
                issues, "FAIL", "too_few_occupied_voxels",
                "20 cm voxel occupancy is too low for reliable map preparation.", occupied)
        elif occupied < args.min_occupied_voxels_warn:
            add_issue(
                issues, "WARN", "low_occupied_voxels",
                "20 cm voxel occupancy is low; mapping coverage may be sparse.", occupied)


def evaluate_quality(issues, quality, args):
    if not quality:
        add_issue(issues, "WARN", "missing_quality_log", "No final topic quality log was found.")
        return

    required_topics = ["/cloud_registered_filtered", "/fastlio_denoised_map"]
    for topic in required_topics:
        if topic not in quality:
            add_issue(issues, "FAIL", "missing_topic_quality", f"Missing quality block for {topic}.")
            continue

        item = quality[topic]
        messages = int(item.get("messages", 0))
        nonzero_messages = int(item.get("nonzero_messages", 0))
        hz = float(item.get("hz", 0.0))
        avg_points = float(item.get("avg_points", 0.0))
        last_points = float(item.get("last_points", 0.0))

        if messages <= 0 or nonzero_messages <= 0:
            add_issue(issues, "FAIL", "empty_topic", f"{topic} did not publish non-empty point clouds.", item)
            continue
        if hz < args.min_hz_fail:
            add_issue(issues, "FAIL", "topic_rate_failed", f"{topic} publish rate is too low.", item)
        elif hz < args.min_hz_warn:
            add_issue(issues, "WARN", "topic_rate_low", f"{topic} publish rate is below target.", item)

        if topic == "/cloud_registered_filtered" and avg_points < args.min_scan_points_warn:
            add_issue(issues, "WARN", "filtered_scan_sparse", "Filtered scan points are sparse.", item)
        if topic == "/fastlio_denoised_map":
            if last_points < args.min_points_fail:
                add_issue(issues, "FAIL", "final_map_topic_too_small", "Final map topic is too small.", item)
            elif last_points < args.min_points_warn:
                add_issue(issues, "WARN", "final_map_topic_low_points", "Final map topic point count is low.", item)


def evaluate_compare(issues, compare_report, args):
    if not compare_report:
        return
    overlap = compare_report.get("overlap") or {}
    jaccard = float(overlap.get("jaccard", 0.0))
    a_coverage = float(overlap.get("a_coverage_by_b", 0.0))
    b_coverage = float(overlap.get("b_coverage_by_a", 0.0))

    if jaccard < args.min_compare_jaccard_warn:
        add_issue(issues, "WARN", "low_map_overlap", "Map overlap is low at the comparison voxel size.", overlap)
    if min(a_coverage, b_coverage) < args.min_compare_coverage_warn:
        add_issue(issues, "WARN", "low_map_coverage", "One map covers too little of the other map.", overlap)

    delta = compare_report.get("centroid_delta_b_minus_a_m") or []
    if len(delta) == 3:
        norm = math.sqrt(sum(float(v) * float(v) for v in delta))
        if norm > args.max_centroid_delta_warn:
            add_issue(
                issues, "WARN", "large_centroid_delta",
                "Compared maps have a large centroid delta.", {"norm_m": norm, "delta_m": delta})


def write_markdown(report, output):
    lines = [
        "# MID360 Mapping Readiness",
        "",
        f"- verdict: `{report['verdict']}`",
        f"- score: `{report['score']}`",
        f"- session: `{report.get('session_dir', '')}`",
        f"- map_analysis: `{report.get('analysis_report', '')}`",
        f"- quality_log: `{report.get('quality_log', '')}`",
        "",
        "## Issues",
        "",
    ]
    if report["issues"]:
        lines.append("| Level | Code | Message |")
        lines.append("| --- | --- | --- |")
        for issue in report["issues"]:
            lines.append(f"| `{issue['level']}` | `{issue['code']}` | {issue['message']} |")
    else:
        lines.append("No blocking or warning issues detected.")
    lines.append("")

    summary = report.get("summary") or {}
    if summary:
        lines.extend([
            "## Summary",
            "",
            f"- points: `{summary.get('points', 0)}`",
            f"- xy_span_m: `{summary.get('xy_span_m', 0.0):.3f}`",
            f"- z_span_m: `{summary.get('z_span_m', 0.0):.3f}`",
            f"- occupied_voxels_20cm: `{summary.get('occupied_voxels_20cm', 0)}`",
            "",
        ])

    output.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Evaluate whether a MID360 mapping session is ready for downstream map preparation.")
    parser.add_argument("session_dir")
    parser.add_argument("--output", default="")
    parser.add_argument("--markdown", default="")
    parser.add_argument("--min-points-fail", type=int, default=1000)
    parser.add_argument("--min-points-warn", type=int, default=5000)
    parser.add_argument("--min-xy-span-fail", type=float, default=1.0)
    parser.add_argument("--min-xy-span-warn", type=float, default=2.0)
    parser.add_argument("--max-xy-span-warn", type=float, default=40.0)
    parser.add_argument("--max-xy-span-fail", type=float, default=120.0)
    parser.add_argument("--max-z-span-warn", type=float, default=8.0)
    parser.add_argument("--max-z-span-fail", type=float, default=30.0)
    parser.add_argument("--min-occupied-voxels-fail", type=int, default=50)
    parser.add_argument("--min-occupied-voxels-warn", type=int, default=250)
    parser.add_argument("--min-hz-fail", type=float, default=1.0)
    parser.add_argument("--min-hz-warn", type=float, default=5.0)
    parser.add_argument("--min-scan-points-warn", type=int, default=300)
    parser.add_argument("--min-compare-jaccard-warn", type=float, default=0.30)
    parser.add_argument("--min-compare-coverage-warn", type=float, default=0.35)
    parser.add_argument("--max-centroid-delta-warn", type=float, default=1.0)
    args = parser.parse_args()

    session_dir = Path(args.session_dir).resolve()
    analysis_path = find_first(session_dir, "maps/*.analysis.json")
    map_report_path = find_map_report(session_dir)
    quality_path = session_dir / "logs" / "final_quality.txt"
    compare_path = find_first(session_dir, "maps/*.compare.json")

    analysis = load_json(analysis_path) if analysis_path else None
    map_report = load_json(map_report_path) if map_report_path else None
    quality = parse_quality_log(quality_path)
    compare_report = load_json(compare_path) if compare_path else None

    issues = []
    evaluate_map(issues, analysis, map_report, args)
    evaluate_quality(issues, quality, args)
    evaluate_compare(issues, compare_report, args)

    summary = {}
    source = analysis or map_report or {}
    if source:
        span = source.get("span_m") or {}
        summary = {
            "points": int(source.get("points", 0)),
            "xy_span_m": max(float(span.get("x", 0.0)), float(span.get("y", 0.0))),
            "z_span_m": float(span.get("z", 0.0)),
            "occupied_voxels_20cm": int(get_voxel_stat(analysis or {}, 0.20).get("occupied_voxels", 0)),
        }

    report = {
        "session_dir": str(session_dir),
        "map_report": str(map_report_path) if map_report_path else "",
        "analysis_report": str(analysis_path) if analysis_path else "",
        "quality_log": str(quality_path) if quality_path.exists() else "",
        "compare_report": str(compare_path) if compare_path else "",
        "verdict": worst_verdict(issues),
        "score": score_from_issues(issues),
        "summary": summary,
        "issues": issues,
    }

    output = Path(args.output) if args.output else session_dir / "mapping_readiness.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    markdown = Path(args.markdown) if args.markdown else session_dir / "mapping_readiness.md"
    write_markdown(report, markdown)

    print(f"Saved readiness report: {output}")
    print(f"Saved readiness markdown: {markdown}")
    print(f"verdict={report['verdict']} score={report['score']}")
    for issue in issues:
        print(f"{issue['level']} {issue['code']}: {issue['message']}")


if __name__ == "__main__":
    main()
