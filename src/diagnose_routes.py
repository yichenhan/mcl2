#!/usr/bin/env python3
"""Diagnose why routes fail to complete: per-route spatial + temporal analysis."""

import json, glob, math, os, sys

REPLAY_DIR = os.path.join(os.path.dirname(__file__), "replays")
ROUTE_DIR = os.path.join(os.path.dirname(__file__), "..", "routes")

def dist(x0, y0, x1, y1):
    return math.sqrt((x1-x0)**2 + (y1-y0)**2)

def wrap180(d):
    d = d % 360
    if d > 180: d -= 360
    return d

def analyze_replay(path):
    with open(path) as f:
        data = json.load(f)

    cfg = data.get("config", {})
    waypoints = cfg.get("waypoints", [])
    obstacles = data.get("obstacles", [])
    ticks = data.get("ticks", [])
    route_name = cfg.get("route_name", os.path.basename(path))
    wp_tol = cfg.get("pure_pursuit", {}).get("waypoint_tolerance", 3.0)

    if not ticks or not waypoints:
        return {"name": route_name, "error": "no ticks or waypoints"}

    # --- Trace ground truth path and accepted estimate path ---
    gt_path = [(t["ground_truth"]["x"], t["ground_truth"]["y"]) for t in ticks]
    ae_path = [(t["accepted_estimate"]["x"], t["accepted_estimate"]["y"]) for t in ticks]

    # --- Waypoint reach analysis (ground truth) ---
    wp_reached_gt = []
    for wi, wp in enumerate(waypoints):
        first_tick = None
        min_dist = float("inf")
        min_tick = 0
        for ti, t in enumerate(ticks):
            d = dist(t["ground_truth"]["x"], t["ground_truth"]["y"], wp["x"], wp["y"])
            if d < min_dist:
                min_dist = d
                min_tick = ti
            if d <= wp_tol and first_tick is None:
                first_tick = ti
        wp_reached_gt.append({
            "wp": wi,
            "target": (wp["x"], wp["y"]),
            "reached_tick": first_tick,
            "closest_dist": round(min_dist, 2),
            "closest_tick": min_tick,
        })

    # --- Waypoint reach analysis (accepted estimate -- what controller sees) ---
    wp_reached_ae = []
    for wi, wp in enumerate(waypoints):
        first_tick = None
        min_dist = float("inf")
        min_tick = 0
        for ti, t in enumerate(ticks):
            ae = t["accepted_estimate"]
            d = dist(ae["x"], ae["y"], wp["x"], wp["y"])
            if d < min_dist:
                min_dist = d
                min_tick = ti
            if d <= wp_tol and first_tick is None:
                first_tick = ti
        wp_reached_ae.append({
            "wp": wi,
            "target": (wp["x"], wp["y"]),
            "reached_tick": first_tick,
            "closest_dist": round(min_dist, 2),
            "closest_tick": min_tick,
        })

    # --- Gate rejection timeline ---
    gate_reject_count = 0
    gate_reject_ticks = []
    velocity_rejects = 0
    spread_rejects = 0
    passability_rejects = 0
    residual_rejects = 0
    wall_sum_rejects = 0
    cardinality_rejects = 0
    for t in ticks:
        gd = t.get("gate_decision", {})
        if not gd.get("accepted", True):
            gate_reject_count += 1
            gate_reject_ticks.append(t["tick"])
            if gd.get("failed_velocity"): velocity_rejects += 1
            if gd.get("failed_spread"): spread_rejects += 1
            if gd.get("failed_passability"): passability_rejects += 1
            if gd.get("failed_residual"): residual_rejects += 1
            if gd.get("failed_wall_sum"): wall_sum_rejects += 1
            if gd.get("failed_cardinality"): cardinality_rejects += 1

    # --- Update skip timeline ---
    update_skip_count = sum(1 for t in ticks if t.get("update_skipped"))

    # --- Active failure timeline ---
    failure_ticks = {}
    for t in ticks:
        for f in t.get("active_failures", []):
            failure_ticks[f] = failure_ticks.get(f, 0) + 1

    # --- MCL error stats ---
    mcl_errors = [t["mcl_error"] for t in ticks]
    mean_mcl = sum(mcl_errors) / len(mcl_errors)
    max_mcl = max(mcl_errors)
    max_mcl_tick = mcl_errors.index(max_mcl)

    # --- Accepted estimate staleness (how many ticks accepted_estimate doesn't change) ---
    stale_count = 0
    for i in range(1, len(ticks)):
        ae0 = ticks[i-1]["accepted_estimate"]
        ae1 = ticks[i]["accepted_estimate"]
        if ae0["x"] == ae1["x"] and ae0["y"] == ae1["y"]:
            stale_count += 1

    # --- Collision / wall stall detection (ground truth doesn't move) ---
    stuck_count = 0
    stuck_runs = []
    cur_run = 0
    for i in range(1, len(ticks)):
        g0 = ticks[i-1]["ground_truth"]
        g1 = ticks[i]["ground_truth"]
        if dist(g0["x"], g0["y"], g1["x"], g1["y"]) < 0.01:
            cur_run += 1
        else:
            if cur_run >= 5:
                stuck_runs.append((i - cur_run, cur_run))
                stuck_count += cur_run
            cur_run = 0
    if cur_run >= 5:
        stuck_runs.append((len(ticks) - cur_run, cur_run))
        stuck_count += cur_run

    # --- Obstacle proximity at end ---
    last_gt = ticks[-1]["ground_truth"]
    near_obstacle = False
    for obs in obstacles:
        ox = (obs["min_x"] + obs["max_x"]) / 2
        oy = (obs["min_y"] + obs["max_y"]) / 2
        if dist(last_gt["x"], last_gt["y"], ox, oy) < 12.0:
            near_obstacle = True
            break

    return {
        "name": route_name,
        "total_ticks": len(ticks),
        "total_waypoints": len(waypoints),
        "wp_tol": wp_tol,
        "waypoints_gt": wp_reached_gt,
        "waypoints_ae": wp_reached_ae,
        "gate_reject_count": gate_reject_count,
        "gate_reject_pct": round(100 * gate_reject_count / len(ticks), 1),
        "velocity_rejects": velocity_rejects,
        "spread_rejects": spread_rejects,
        "passability_rejects": passability_rejects,
        "residual_rejects": residual_rejects,
        "wall_sum_rejects": wall_sum_rejects,
        "cardinality_rejects": cardinality_rejects,
        "update_skip_count": update_skip_count,
        "failure_ticks": failure_ticks,
        "mean_mcl_error": round(mean_mcl, 2),
        "max_mcl_error": round(max_mcl, 2),
        "max_mcl_error_tick": max_mcl_tick,
        "stale_estimate_ticks": stale_count,
        "stale_estimate_pct": round(100 * stale_count / max(1, len(ticks)), 1),
        "stuck_ticks": stuck_count,
        "stuck_runs": stuck_runs[:5],
        "near_obstacle_at_end": near_obstacle,
        "final_gt": (round(last_gt["x"], 1), round(last_gt["y"], 1)),
        "final_ae": (round(ticks[-1]["accepted_estimate"]["x"], 1),
                     round(ticks[-1]["accepted_estimate"]["y"], 1)),
    }


def print_report(r):
    print(f"\n{'='*72}")
    print(f"ROUTE: {r['name']}")
    print(f"  Ticks: {r['total_ticks']}  |  Waypoints: {r['total_waypoints']}  |  Tolerance: {r['wp_tol']} in")
    print(f"  Final ground truth: {r['final_gt']}  |  Final accepted: {r['final_ae']}")
    print()

    # Waypoint table (ground truth vs accepted estimate)
    print(f"  {'WP':>3} {'Target':>14} | {'GT reach':>9} {'GT closest':>11} {'GT tick':>8} | {'AE reach':>9} {'AE closest':>11} {'AE tick':>8}")
    print(f"  {'-'*3} {'-'*14} | {'-'*9} {'-'*11} {'-'*8} | {'-'*9} {'-'*11} {'-'*8}")
    for wg, wa in zip(r["waypoints_gt"], r["waypoints_ae"]):
        gt_r = f"t={wg['reached_tick']}" if wg["reached_tick"] is not None else "MISS"
        ae_r = f"t={wa['reached_tick']}" if wa["reached_tick"] is not None else "MISS"
        tgt = f"({wg['target'][0]:6.1f},{wg['target'][1]:6.1f})"
        print(f"  {wg['wp']:3d} {tgt:>14} | {gt_r:>9} {wg['closest_dist']:>9.1f}in t={wg['closest_tick']:<4d} | {ae_r:>9} {wa['closest_dist']:>9.1f}in t={wa['closest_tick']:<4d}")

    print()
    print(f"  MCL error:  mean={r['mean_mcl_error']:.1f}in  max={r['max_mcl_error']:.1f}in @ tick {r['max_mcl_error_tick']}")
    print(f"  Gate rejects: {r['gate_reject_count']}/{r['total_ticks']} ({r['gate_reject_pct']}%)"
          f"  [vel={r['velocity_rejects']} spr={r['spread_rejects']} pass={r['passability_rejects']}"
          f" res={r['residual_rejects']} wall={r['wall_sum_rejects']} card={r['cardinality_rejects']}]")
    print(f"  Update skipped: {r['update_skip_count']}/{r['total_ticks']}")
    print(f"  Accepted estimate stale: {r['stale_estimate_ticks']}/{r['total_ticks']} ({r['stale_estimate_pct']}%)")
    print(f"  Robot physically stuck: {r['stuck_ticks']} ticks across {len(r['stuck_runs'])} runs"
          + (f"  first: tick {r['stuck_runs'][0][0]} for {r['stuck_runs'][0][1]} ticks" if r['stuck_runs'] else ""))
    if r["failure_ticks"]:
        print(f"  Active failures: {r['failure_ticks']}")
    if r["near_obstacle_at_end"]:
        print(f"  ** Robot near obstacle at final tick **")

    # Diagnosis
    missed_gt = [w for w in r["waypoints_gt"] if w["reached_tick"] is None]
    missed_ae = [w for w in r["waypoints_ae"] if w["reached_tick"] is None]
    print()
    print("  DIAGNOSIS:")
    if not missed_gt:
        print("    Ground truth reached all waypoints (controller/MCL issue only)")
    else:
        for m in missed_gt:
            reasons = []
            if m["closest_dist"] > 20:
                reasons.append(f"never got close (closest {m['closest_dist']:.1f}in)")
            elif m["closest_dist"] > r["wp_tol"]:
                reasons.append(f"approached to {m['closest_dist']:.1f}in but outside {r['wp_tol']}in tolerance")
            if r["stuck_ticks"] > r["total_ticks"] * 0.1:
                reasons.append("robot spent >10% of time physically stuck (collision/wall)")
            if r["stale_estimate_pct"] > 50:
                reasons.append(f"accepted estimate frozen {r['stale_estimate_pct']}% of time (gates rejecting)")
            if r["gate_reject_pct"] > 60:
                reasons.append(f"gate rejection rate {r['gate_reject_pct']}% (controller steering on stale pose)")
            if r["velocity_rejects"] > r["total_ticks"] * 0.3:
                reasons.append("velocity gate rejecting heavily (MCL hasn't converged)")
            if r["spread_rejects"] > r["total_ticks"] * 0.1:
                reasons.append("spread gate rejecting (particles too dispersed)")
            if r["failure_ticks"]:
                reasons.append(f"sensor failures active: {r['failure_ticks']}")
            if not reasons:
                reasons.append("ran out of ticks before reaching waypoint")
            print(f"    WP{m['wp']} ({m['target'][0]:.0f},{m['target'][1]:.0f}): {'; '.join(reasons)}")


if __name__ == "__main__":
    replays = sorted(glob.glob(os.path.join(REPLAY_DIR, "route_*.json")))
    if not replays:
        print("No route replay files found.")
        sys.exit(1)

    results = []
    for rp in replays:
        r = analyze_replay(rp)
        results.append(r)
        print_report(r)

    print(f"\n{'='*72}")
    print("SUMMARY")
    print(f"{'='*72}")
    completed = sum(1 for r in results
                    if all(w["reached_tick"] is not None for w in r["waypoints_gt"]))
    print(f"Routes where GT hit all waypoints: {completed}/{len(results)}")
    completed_ae = sum(1 for r in results
                       if all(w["reached_tick"] is not None for w in r["waypoints_ae"]))
    print(f"Routes where accepted estimate hit all waypoints: {completed_ae}/{len(results)}")

    avg_gate = sum(r["gate_reject_pct"] for r in results) / len(results)
    avg_stale = sum(r["stale_estimate_pct"] for r in results) / len(results)
    avg_stuck = sum(r["stuck_ticks"] for r in results) / len(results)
    print(f"Avg gate rejection rate: {avg_gate:.1f}%")
    print(f"Avg estimate stale rate: {avg_stale:.1f}%")
    print(f"Avg physically-stuck ticks: {avg_stuck:.0f}")
