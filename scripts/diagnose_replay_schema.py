#!/usr/bin/env python3
"""Diagnose replay JSON schema/data issues without loading whole files in memory."""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Optional, Tuple


RE_TOTAL_TICKS = re.compile(r'"total_ticks"\s*:\s*(\d+)')
RE_SESSION_ID = re.compile(r'"session_id"\s*:\s*"([^"]*)"')

REQUIRED_TICK_KEYS = (
    "tick",
    "observed_readings",
    "observed_heading",
    "post_predict",
    "post_update",
    "post_resample",
    "valid_sensor_count",
    "update_skipped",
    "pose_gated",
    "gate_decision",
)

LEGACY_OPTIONAL_KEYS = (
    "ground_truth",
    "accepted_estimate",
    "active_failures",
    "mcl_error",
    "odom_error",
)

GATE_KEYS = (
    "accepted",
    "reason",
    "radius_90_in",
    "spread_in",
    "jump_in",
    "failed_spread",
    "failed_passability",
    "failed_residual",
    "failed_wall_sum",
    "failed_velocity",
)


@dataclass
class FileStats:
    path: Path
    session_id: str = ""
    total_ticks_declared: Optional[int] = None
    ticks_parsed: int = 0
    key_present_counts: Counter = field(default_factory=Counter)
    missing_required_counts: Counter = field(default_factory=Counter)
    gate_key_present_counts: Counter = field(default_factory=Counter)
    anomalies: List[str] = field(default_factory=list)
    tick_gaps: int = 0
    first_tick_value: Optional[int] = None
    last_tick_value: Optional[int] = None

    def add_anomaly(self, message: str, max_items: int) -> None:
        if len(self.anomalies) < max_items:
            self.anomalies.append(message)


def _try_read_meta(path: Path) -> Tuple[Optional[int], str]:
    declared_total = None
    session_id = ""
    with path.open("r", encoding="utf-8") as f:
        for _ in range(400):
            line = f.readline()
            if not line:
                break
            if declared_total is None:
                m_total = RE_TOTAL_TICKS.search(line)
                if m_total:
                    declared_total = int(m_total.group(1))
            if not session_id:
                m_session = RE_SESSION_ID.search(line)
                if m_session:
                    session_id = m_session.group(1)
            if declared_total is not None and session_id:
                break
    return declared_total, session_id


def iter_ticks(path: Path) -> Iterator[Dict[str, Any]]:
    in_ticks = False
    capturing = False
    brace_depth = 0
    buf: List[str] = []

    with path.open("r", encoding="utf-8") as f:
        for line in f:
            stripped = line.lstrip()
            if not in_ticks:
                if '"ticks"' in stripped and "[" in stripped:
                    in_ticks = True
                continue

            if not capturing:
                if stripped.startswith("{"):
                    capturing = True
                    brace_depth = line.count("{") - line.count("}")
                    buf = [line]
                    if brace_depth == 0:
                        tick_text = "".join(buf).rstrip().rstrip(",")
                        yield json.loads(tick_text)
                        capturing = False
                        buf = []
                elif stripped.startswith("]"):
                    break
                continue

            buf.append(line)
            brace_depth += line.count("{") - line.count("}")
            if brace_depth == 0:
                tick_text = "".join(buf).rstrip().rstrip(",")
                yield json.loads(tick_text)
                capturing = False
                buf = []


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _validate_snapshot(tick: Dict[str, Any], key: str, idx: int, stats: FileStats, max_anomalies: int) -> None:
    snapshot = tick.get(key)
    if not isinstance(snapshot, dict):
        stats.add_anomaly(f"tick {idx}: `{key}` is not an object", max_anomalies)
        return
    est = snapshot.get("estimate")
    if not isinstance(est, dict) or not _is_number(est.get("x")) or not _is_number(est.get("y")):
        stats.add_anomaly(f"tick {idx}: `{key}.estimate` missing numeric x/y", max_anomalies)
    if not _is_number(snapshot.get("n_eff")):
        stats.add_anomaly(f"tick {idx}: `{key}.n_eff` is missing or non-numeric", max_anomalies)
    particles = snapshot.get("particles")
    if not isinstance(particles, list):
        stats.add_anomaly(f"tick {idx}: `{key}.particles` is not an array", max_anomalies)
    elif len(particles) == 0:
        stats.add_anomaly(f"tick {idx}: `{key}.particles` is empty", max_anomalies)


def analyze_file(path: Path, max_anomalies: int = 30) -> FileStats:
    stats = FileStats(path=path)
    stats.total_ticks_declared, stats.session_id = _try_read_meta(path)

    expected_next_tick: Optional[int] = None

    for idx, tick in enumerate(iter_ticks(path)):
        stats.ticks_parsed += 1

        if not isinstance(tick, dict):
            stats.add_anomaly(f"tick index {idx}: tick entry is not an object", max_anomalies)
            continue

        for key in tick.keys():
            stats.key_present_counts[key] += 1

        for req in REQUIRED_TICK_KEYS:
            if req not in tick:
                stats.missing_required_counts[req] += 1

        raw_tick_value = tick.get("tick")
        if isinstance(raw_tick_value, int):
            if stats.first_tick_value is None:
                stats.first_tick_value = raw_tick_value
                expected_next_tick = raw_tick_value
            if expected_next_tick is not None and raw_tick_value != expected_next_tick:
                stats.tick_gaps += 1
                stats.add_anomaly(
                    f"tick index {idx}: expected tick={expected_next_tick}, got tick={raw_tick_value}",
                    max_anomalies,
                )
            expected_next_tick = raw_tick_value + 1
            stats.last_tick_value = raw_tick_value
        else:
            stats.add_anomaly(f"tick index {idx}: missing/non-int `tick` value", max_anomalies)

        observed_readings = tick.get("observed_readings")
        if not isinstance(observed_readings, list) or len(observed_readings) != 4:
            stats.add_anomaly(f"tick index {idx}: `observed_readings` is not a 4-item array", max_anomalies)
        elif not all(_is_number(v) for v in observed_readings):
            stats.add_anomaly(f"tick index {idx}: `observed_readings` contains non-numeric values", max_anomalies)

        if not _is_number(tick.get("observed_heading")):
            stats.add_anomaly(f"tick index {idx}: `observed_heading` missing/non-numeric", max_anomalies)

        for snap_key in ("post_predict", "post_update", "post_resample"):
            _validate_snapshot(tick, snap_key, idx, stats, max_anomalies)

        gate_decision = tick.get("gate_decision")
        if isinstance(gate_decision, dict):
            for gk in GATE_KEYS:
                if gk in gate_decision:
                    stats.gate_key_present_counts[gk] += 1
        else:
            stats.add_anomaly(f"tick index {idx}: `gate_decision` missing/non-object", max_anomalies)

    if stats.total_ticks_declared is not None and stats.total_ticks_declared != stats.ticks_parsed:
        stats.add_anomaly(
            f"declared total_ticks={stats.total_ticks_declared} but parsed ticks={stats.ticks_parsed}",
            max_anomalies,
        )

    return stats


def analyze_dir(directory: Path, max_anomalies: int = 30, pattern: str = "*.json") -> List[FileStats]:
    files = sorted(directory.glob(pattern))
    return [analyze_file(path, max_anomalies=max_anomalies) for path in files]


def _pct(n: int, d: int) -> str:
    if d <= 0:
        return "0.0%"
    return f"{(100.0 * n / d):.1f}%"


def print_report(stats_list: Iterable[FileStats]) -> None:
    stats_list = list(stats_list)
    if not stats_list:
        print("No JSON replay files found.")
        return

    print("Replay schema diagnostics")
    print("=" * 88)
    for st in stats_list:
        print(f"\nFile: {st.path}")
        print(f"  session_id: {st.session_id or '-'}")
        print(f"  ticks parsed: {st.ticks_parsed}")
        declared = st.total_ticks_declared if st.total_ticks_declared is not None else "-"
        print(f"  total_ticks declared: {declared}")
        if st.first_tick_value is not None:
            print(f"  tick sequence: start={st.first_tick_value}, end={st.last_tick_value}, gaps={st.tick_gaps}")
        else:
            print("  tick sequence: unavailable")

        missing_items = [(k, c) for k, c in st.missing_required_counts.items() if c > 0]
        if missing_items:
            print("  missing required keys:")
            for key, count in sorted(missing_items):
                print(f"    - {key}: {count} ticks ({_pct(count, st.ticks_parsed)})")
        else:
            print("  missing required keys: none")

        print("  legacy/optional field coverage:")
        for key in LEGACY_OPTIONAL_KEYS:
            present = st.key_present_counts.get(key, 0)
            print(f"    - {key}: {present}/{st.ticks_parsed} ({_pct(present, st.ticks_parsed)})")

        print("  gate field coverage:")
        for key in GATE_KEYS:
            present = st.gate_key_present_counts.get(key, 0)
            print(f"    - {key}: {present}/{st.ticks_parsed} ({_pct(present, st.ticks_parsed)})")

        if st.anomalies:
            print("  anomalies:")
            for item in st.anomalies:
                print(f"    - {item}")
        else:
            print("  anomalies: none detected")

    total_ticks = sum(s.ticks_parsed for s in stats_list)
    print("\n" + "=" * 88)
    print(f"Files analyzed: {len(stats_list)}")
    print(f"Total ticks analyzed: {total_ticks}")
    files_with_anomalies = sum(1 for s in stats_list if s.anomalies)
    print(f"Files with anomalies: {files_with_anomalies}/{len(stats_list)}")


def to_json(stats_list: Iterable[FileStats]) -> Dict[str, Any]:
    out_files = []
    for st in stats_list:
        out_files.append(
            {
                "file": str(st.path),
                "session_id": st.session_id,
                "ticks_parsed": st.ticks_parsed,
                "total_ticks_declared": st.total_ticks_declared,
                "first_tick": st.first_tick_value,
                "last_tick": st.last_tick_value,
                "tick_gaps": st.tick_gaps,
                "missing_required_counts": dict(st.missing_required_counts),
                "key_present_counts": dict(st.key_present_counts),
                "gate_key_present_counts": dict(st.gate_key_present_counts),
                "anomalies": st.anomalies,
            }
        )
    return {"files": out_files}


def main() -> None:
    parser = argparse.ArgumentParser(description="Diagnose replay JSON schema and data consistency.")
    parser.add_argument(
        "--dir",
        default="replays/new_folder_test",
        help="Directory containing replay JSON files (default: replays/new_folder_test)",
    )
    parser.add_argument(
        "--file",
        default="",
        help="Analyze a single replay file instead of --dir",
    )
    parser.add_argument(
        "--pattern",
        default="*.json",
        help="Glob pattern used with --dir (default: *.json)",
    )
    parser.add_argument(
        "--max-anomalies",
        type=int,
        default=30,
        help="Max anomaly messages stored per file",
    )
    parser.add_argument(
        "--json-out",
        default="",
        help="Optional path to write machine-readable JSON report",
    )
    args = parser.parse_args()

    if args.file:
        replay_file = Path(args.file)
        if not replay_file.is_file():
            raise SystemExit(f"Replay file not found: {replay_file}")
        stats_list = [analyze_file(replay_file, max_anomalies=args.max_anomalies)]
    else:
        replay_dir = Path(args.dir)
        if not replay_dir.exists() or not replay_dir.is_dir():
            raise SystemExit(f"Replay directory not found: {replay_dir}")
        stats_list = analyze_dir(replay_dir, max_anomalies=args.max_anomalies, pattern=args.pattern)
    print_report(stats_list)

    if args.json_out:
        out_path = Path(args.json_out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w", encoding="utf-8") as f:
            json.dump(to_json(stats_list), f, indent=2)
        print(f"\nWrote JSON report: {out_path}")


if __name__ == "__main__":
    main()
