#!/usr/bin/env python3
"""Compare replay schema coverage between two directories."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Optional

from diagnose_replay_schema import LEGACY_OPTIONAL_KEYS, REQUIRED_TICK_KEYS, FileStats, analyze_dir


def normalize_name(path: Path) -> str:
    name = path.stem
    name = name.replace("_clean", "")
    return name


def index_by_normalized_name(stats: List[FileStats]) -> Dict[str, FileStats]:
    return {normalize_name(s.path): s for s in stats}


def pct(present: int, total: int) -> float:
    if total <= 0:
        return 0.0
    return 100.0 * present / total


def print_coverage_line(field: str, left: FileStats, right: FileStats) -> None:
    left_present = left.key_present_counts.get(field, 0)
    right_present = right.key_present_counts.get(field, 0)
    left_pct = pct(left_present, left.ticks_parsed)
    right_pct = pct(right_present, right.ticks_parsed)
    delta = left_pct - right_pct
    print(
        f"    {field:<18} "
        f"left={left_present:>6}/{left.ticks_parsed:<6} ({left_pct:>5.1f}%)   "
        f"right={right_present:>6}/{right.ticks_parsed:<6} ({right_pct:>5.1f}%)   "
        f"delta={delta:>+6.1f} pp"
    )


def print_missing_required(left: FileStats, right: FileStats) -> None:
    print("  Required-key missing counts (left vs right):")
    for field in REQUIRED_TICK_KEYS:
        l_missing = left.missing_required_counts.get(field, 0)
        r_missing = right.missing_required_counts.get(field, 0)
        if l_missing == 0 and r_missing == 0:
            continue
        print(f"    {field:<18} left={l_missing:>6} right={r_missing:>6}")


def summarize_field_drops(
    route: str,
    left: FileStats,
    right: FileStats,
    alerts: List[str],
    threshold_pp: float = 25.0,
) -> None:
    for field in LEGACY_OPTIONAL_KEYS:
        l_pct = pct(left.key_present_counts.get(field, 0), left.ticks_parsed)
        r_pct = pct(right.key_present_counts.get(field, 0), right.ticks_parsed)
        if r_pct - l_pct >= threshold_pp:
            alerts.append(
                f"{route}: `{field}` coverage dropped by {r_pct - l_pct:.1f} pp "
                f"({r_pct:.1f}% -> {l_pct:.1f}%)"
            )


def summarize_tick_count_shift(route: str, left: FileStats, right: FileStats, alerts: List[str]) -> None:
    if right.ticks_parsed <= 0:
        return
    ratio = left.ticks_parsed / right.ticks_parsed
    if ratio <= 0.25 or ratio >= 3.0:
        alerts.append(
            f"{route}: tick count changed significantly "
            f"({right.ticks_parsed} -> {left.ticks_parsed}, ratio {ratio:.2f}x)"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare replay schema coverage across two directories.")
    parser.add_argument(
        "--left",
        default="replays/new_folder_test",
        help="Directory under investigation (default: replays/new_folder_test)",
    )
    parser.add_argument(
        "--right",
        default="src/replays",
        help="Baseline replay directory to compare against (default: src/replays)",
    )
    parser.add_argument(
        "--max-anomalies",
        type=int,
        default=10,
        help="Max anomaly messages kept per file while scanning",
    )
    args = parser.parse_args()

    left_dir = Path(args.left)
    right_dir = Path(args.right)
    if not left_dir.is_dir():
        raise SystemExit(f"Invalid left directory: {left_dir}")
    if not right_dir.is_dir():
        raise SystemExit(f"Invalid right directory: {right_dir}")

    left_stats = analyze_dir(left_dir, max_anomalies=args.max_anomalies)
    right_stats = analyze_dir(right_dir, max_anomalies=args.max_anomalies)
    left_by_name = index_by_normalized_name(left_stats)
    right_by_name = index_by_normalized_name(right_stats)

    common_routes = sorted(set(left_by_name.keys()) & set(right_by_name.keys()))
    if not common_routes:
        raise SystemExit("No matching replay names found between directories.")

    print(f"Comparing {len(common_routes)} matching replay files")
    print(f"  left : {left_dir}")
    print(f"  right: {right_dir}")
    print("=" * 100)

    alerts: List[str] = []
    for route in common_routes:
        left = left_by_name[route]
        right = right_by_name[route]
        print(f"\nRoute: {route}")
        print(f"  left file : {left.path.name}")
        print(f"  right file: {right.path.name}")
        print(
            f"  ticks      left={left.ticks_parsed} right={right.ticks_parsed} "
            f"tick_gap(left)={left.tick_gaps} tick_gap(right)={right.tick_gaps}"
        )

        print("  Legacy/optional coverage deltas:")
        for field in LEGACY_OPTIONAL_KEYS:
            print_coverage_line(field, left, right)

        print_missing_required(left, right)
        summarize_field_drops(route, left, right, alerts)
        summarize_tick_count_shift(route, left, right, alerts)

    print("\n" + "=" * 100)
    if alerts:
        print("Potential regressions:")
        for line in alerts:
            print(f"  - {line}")
    else:
        print("No large optional-field coverage drops detected.")


if __name__ == "__main__":
    main()
