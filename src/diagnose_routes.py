#!/usr/bin/env python3
"""Diagnose why routes fail to complete: per-route spatial + temporal analysis.

Works with the replay JSON files written by SessionRecorder, which contain
ground_truth, post_predict/post_update/post_resample MCL snapshots, and
observed_heading / observed_readings per tick.
"""

import json, glob, math, os, sys

REPLAY_DIR = os.path.join(os.path.dirname(__file__), "..", "replays")

def dist(x0, y0, x1, y1):
    return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

def wrap180(d):
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d

def heading_to(x0, y0, x1, y1):
    return math.degrees(math.atan2(x1 - x0, y1 - y0))

def mcl_estimate(tick):
    return tick["post_resample"]["estimate"]

def analyze_replay(path):
    with open(path) as f:
        data = json.load(f)

    cfg = data.get("config", {})
    waypoints = cfg.get("waypoints", [])
    obstacles = data.get("obstacles", [])
    ticks = data.get("ticks", [])
    route_name = cfg.get("route_name", os.path.basename(path))
    follower_cfg = cfg.get("follower", cfg.get("pure_pursuit", {}))
    wp_tol = follower_cfg.get("waypoint_tolerance", 3.0)
    field_half = 72.0

    if not ticks or not waypoints:
        return {"name": route_name, "error": "no ticks or waypoints"}

    # --------------- ground truth trajectory ---------------
    gt_path = [(t["ground_truth"]["x"], t["ground_truth"]["y"]) for t in ticks]
    gt_headings = [t["ground_truth"]["heading_deg"] for t in ticks]

    # --------------- MCL estimate trajectory ---------------
    est_path = [(mcl_estimate(t)["x"], mcl_estimate(t)["y"]) for t in ticks]

    # --------------- MCL error over time ---------------
    mcl_errors = [t["mcl_error"] for t in ticks]
    mean_mcl = sum(mcl_errors) / len(mcl_errors)
    max_mcl = max(mcl_errors)
    max_mcl_tick = mcl_errors.index(max_mcl)

    # --------------- MCL convergence analysis ---------------
    # When does r90 first drop below 10? (proxy for "converged")
    converge_tick = None
    for i, t in enumerate(ticks):
        r90 = t["post_resample"].get("radius_90", 999)
        if r90 < 10:
            converge_tick = i
            break

    # Is the MCL converged but to the WRONG place?
    wrong_convergence_ticks = 0
    confident_ticks = 0
    for t in ticks:
        r90 = t["post_resample"].get("radius_90", 999)
        if r90 < 10:
            confident_ticks += 1
            if t["mcl_error"] > 10:
                wrong_convergence_ticks += 1

    # --------------- Waypoint reach (ground truth) ---------------
    wp_reached_gt = []
    for wi, wp in enumerate(waypoints):
        first_tick = None
        min_d = float("inf")
        min_tick = 0
        for ti, t in enumerate(ticks):
            d = dist(t["ground_truth"]["x"], t["ground_truth"]["y"], wp["x"], wp["y"])
            if d < min_d:
                min_d = d
                min_tick = ti
            if d <= wp_tol and first_tick is None:
                first_tick = ti
        wp_reached_gt.append({
            "wp": wi, "target": (wp["x"], wp["y"]),
            "reached_tick": first_tick, "closest_dist": round(min_d, 2),
            "closest_tick": min_tick,
        })

    # --------------- Waypoint reach (MCL estimate) ---------------
    wp_reached_est = []
    for wi, wp in enumerate(waypoints):
        first_tick = None
        min_d = float("inf")
        min_tick = 0
        for ti, t in enumerate(ticks):
            e = mcl_estimate(t)
            d = dist(e["x"], e["y"], wp["x"], wp["y"])
            if d < min_d:
                min_d = d
                min_tick = ti
            if d <= wp_tol and first_tick is None:
                first_tick = ti
        wp_reached_est.append({
            "wp": wi, "target": (wp["x"], wp["y"]),
            "reached_tick": first_tick, "closest_dist": round(min_d, 2),
            "closest_tick": min_tick,
        })

    # --------------- Wall proximity / stuck detection ---------------
    wall_margin = 6.0
    near_wall_ticks = 0
    for gx, gy in gt_path:
        if (abs(gx) > field_half - wall_margin or
                abs(gy) > field_half - wall_margin):
            near_wall_ticks += 1

    stuck_count = 0
    stuck_runs = []
    cur_run = 0
    for i in range(1, len(ticks)):
        g0 = ticks[i - 1]["ground_truth"]
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

    # --------------- Heading analysis ---------------
    # Track whether heading stays constant (robot driving in one direction)
    heading_changes = []
    for i in range(1, len(ticks)):
        dh = abs(wrap180(gt_headings[i] - gt_headings[i - 1]))
        heading_changes.append(dh)
    mean_heading_change = sum(heading_changes) / max(1, len(heading_changes))
    large_turns = sum(1 for dh in heading_changes if dh > 5)

    # --------------- Sensor health ---------------
    dead_sensor_ticks = [0, 0, 0, 0]
    for t in ticks:
        readings = t.get("observed_readings", [])
        for s in range(min(4, len(readings))):
            if readings[s] < 0:
                dead_sensor_ticks[s] += 1

    update_skip_count = sum(1 for t in ticks if t.get("update_skipped"))

    # --------------- Active failures ---------------
    failure_ticks = {}
    for t in ticks:
        for f in t.get("active_failures", []):
            failure_ticks[f] = failure_ticks.get(f, 0) + 1

    # --------------- Position divergence timeline ---------------
    # Sample MCL error at key intervals
    n = len(ticks)
    sample_points = [0, n // 10, n // 4, n // 2, 3 * n // 4, n - 1]
    error_samples = []
    for i in sample_points:
        if i < n:
            t = ticks[i]
            e = mcl_estimate(t)
            gt = t["ground_truth"]
            error_samples.append({
                "tick": i,
                "gt": (round(gt["x"], 1), round(gt["y"], 1)),
                "est": (round(e["x"], 1), round(e["y"], 1)),
                "err": round(t["mcl_error"], 1),
                "heading_gt": round(gt["heading_deg"], 1),
            })

    # --------------- Distance traveled ---------------
    total_distance = 0.0
    for i in range(1, len(gt_path)):
        total_distance += dist(gt_path[i - 1][0], gt_path[i - 1][1],
                               gt_path[i][0], gt_path[i][1])

    last_gt = ticks[-1]["ground_truth"]
    last_est = mcl_estimate(ticks[-1])

    return {
        "name": route_name,
        "total_ticks": n,
        "total_waypoints": len(waypoints),
        "wp_tol": wp_tol,
        "waypoints_gt": wp_reached_gt,
        "waypoints_est": wp_reached_est,
        "mean_mcl_error": round(mean_mcl, 2),
        "max_mcl_error": round(max_mcl, 2),
        "max_mcl_error_tick": max_mcl_tick,
        "converge_tick": converge_tick,
        "confident_ticks": confident_ticks,
        "wrong_convergence_ticks": wrong_convergence_ticks,
        "wrong_convergence_pct": round(100 * wrong_convergence_ticks / max(1, confident_ticks), 1),
        "update_skip_count": update_skip_count,
        "failure_ticks": failure_ticks,
        "near_wall_ticks": near_wall_ticks,
        "near_wall_pct": round(100 * near_wall_ticks / n, 1),
        "stuck_ticks": stuck_count,
        "stuck_runs": stuck_runs[:5],
        "mean_heading_change": round(mean_heading_change, 2),
        "large_turns": large_turns,
        "dead_sensor_ticks": dead_sensor_ticks,
        "total_distance": round(total_distance, 1),
        "error_samples": error_samples,
        "final_gt": (round(last_gt["x"], 1), round(last_gt["y"], 1)),
        "final_gt_heading": round(last_gt["heading_deg"], 1),
        "final_est": (round(last_est["x"], 1), round(last_est["y"], 1)),
        "near_obstacle_at_end": any(
            dist(last_gt["x"], last_gt["y"],
                 (o["min_x"] + o["max_x"]) / 2,
                 (o["min_y"] + o["max_y"]) / 2) < 12.0
            for o in obstacles
        ),
    }


def print_report(r):
    if "error" in r:
        print(f"\n{'=' * 72}")
        print(f"ROUTE: {r['name']}  -- {r['error']}")
        return

    print(f"\n{'=' * 72}")
    print(f"ROUTE: {r['name']}")
    print(f"  Ticks: {r['total_ticks']}  |  Waypoints: {r['total_waypoints']}  "
          f"|  Tolerance: {r['wp_tol']}in  |  Distance traveled: {r['total_distance']}in")
    print(f"  Final GT:  {r['final_gt']}  heading={r['final_gt_heading']:.0f} deg")
    print(f"  Final EST: {r['final_est']}")
    print()

    # ---- Waypoint table ----
    print(f"  {'WP':>3} {'Target':>14} | {'GT reach':>9} {'GT closest':>11} {'GT tick':>8} "
          f"| {'EST reach':>9} {'EST closest':>11} {'EST tick':>8}")
    print(f"  {'-' * 3} {'-' * 14} | {'-' * 9} {'-' * 11} {'-' * 8} "
          f"| {'-' * 9} {'-' * 11} {'-' * 8}")
    for wg, we in zip(r["waypoints_gt"], r["waypoints_est"]):
        gt_r = f"t={wg['reached_tick']}" if wg["reached_tick"] is not None else "MISS"
        est_r = f"t={we['reached_tick']}" if we["reached_tick"] is not None else "MISS"
        tgt = f"({wg['target'][0]:6.1f},{wg['target'][1]:6.1f})"
        print(f"  {wg['wp']:3d} {tgt:>14} | {gt_r:>9} {wg['closest_dist']:>9.1f}in "
              f"t={wg['closest_tick']:<4d} | {est_r:>9} {we['closest_dist']:>9.1f}in "
              f"t={we['closest_tick']:<4d}")

    # ---- MCL health ----
    print()
    print(f"  MCL error:  mean={r['mean_mcl_error']:.1f}in  "
          f"max={r['max_mcl_error']:.1f}in @ tick {r['max_mcl_error_tick']}")
    conv = r["converge_tick"]
    print(f"  MCL convergence (r90<10):  {'tick ' + str(conv) if conv is not None else 'NEVER'}")
    if r["confident_ticks"] > 0:
        print(f"  Confident ticks (r90<10): {r['confident_ticks']}/{r['total_ticks']}  "
              f"| Wrong while confident: {r['wrong_convergence_ticks']} "
              f"({r['wrong_convergence_pct']}%)")

    # ---- Position divergence timeline ----
    print()
    print("  Position timeline:")
    print(f"    {'Tick':>6}  {'GT pos':>16}  {'EST pos':>16}  {'Error':>7}  {'GT heading':>10}")
    for s in r["error_samples"]:
        print(f"    {s['tick']:6d}  ({s['gt'][0]:6.1f},{s['gt'][1]:6.1f})  "
              f"({s['est'][0]:6.1f},{s['est'][1]:6.1f})  {s['err']:6.1f}in  "
              f"{s['heading_gt']:8.1f} deg")

    # ---- Wall and stuck ----
    print()
    print(f"  Near wall (<6in): {r['near_wall_ticks']}/{r['total_ticks']} "
          f"({r['near_wall_pct']}%)")
    print(f"  Physically stuck: {r['stuck_ticks']} ticks across "
          f"{len(r['stuck_runs'])} run(s)"
          + (f"  first @ tick {r['stuck_runs'][0][0]} for "
             f"{r['stuck_runs'][0][1]} ticks" if r['stuck_runs'] else ""))

    # ---- Heading ----
    print(f"  Mean heading change/tick: {r['mean_heading_change']:.2f} deg  "
          f"| Large turns (>5 deg): {r['large_turns']}")

    # ---- Sensors ----
    print(f"  Update skipped: {r['update_skip_count']}/{r['total_ticks']}")
    print(f"  Dead sensor ticks: {r['dead_sensor_ticks']}")
    if r["failure_ticks"]:
        print(f"  Active failures: {r['failure_ticks']}")
    if r["near_obstacle_at_end"]:
        print(f"  ** Robot near obstacle at final tick **")

    # ---- Diagnosis ----
    missed_gt = [w for w in r["waypoints_gt"] if w["reached_tick"] is None]
    print()
    print("  DIAGNOSIS:")

    if not missed_gt:
        missed_est = [w for w in r["waypoints_est"] if w["reached_tick"] is None]
        if missed_est:
            print("    GT reached all waypoints but MCL estimate did not -- "
                  "follower never saw itself arrive.")
        else:
            print("    All waypoints reached.")
        return

    # High-level root cause detection
    root_causes = []

    if r["wrong_convergence_pct"] > 30:
        root_causes.append(
            f"MCL converged to WRONG position {r['wrong_convergence_pct']}% of "
            f"confident ticks -- robot steers toward waypoint from phantom location")

    if r["near_wall_pct"] > 30:
        root_causes.append(
            f"Robot spent {r['near_wall_pct']}% of time near field walls "
            f"(driven there by bad MCL estimate)")

    if r["stuck_ticks"] > r["total_ticks"] * 0.1:
        root_causes.append(
            f"Robot physically stuck {r['stuck_ticks']} ticks "
            f"({round(100 * r['stuck_ticks'] / r['total_ticks'])}% of run)")

    if r["mean_mcl_error"] > 15:
        root_causes.append(
            f"Mean MCL error {r['mean_mcl_error']:.1f}in -- localization "
            f"never reliable (control loop feedback divergence)")

    if r["update_skip_count"] > r["total_ticks"] * 0.4:
        root_causes.append(
            f"MCL update skipped {r['update_skip_count']}/{r['total_ticks']} "
            f"ticks -- particles not getting corrected by sensor data")

    if sum(r["dead_sensor_ticks"]) > r["total_ticks"] * 2:
        root_causes.append(
            f"Heavy sensor loss -- dead sensor ticks {r['dead_sensor_ticks']}")

    if not root_causes:
        root_causes.append("Ran out of ticks before reaching all waypoints")

    for cause in root_causes:
        print(f"    * {cause}")

    for m in missed_gt:
        detail = []
        if m["closest_dist"] > 20:
            detail.append(f"never got close (closest {m['closest_dist']:.1f}in)")
        elif m["closest_dist"] > r["wp_tol"]:
            detail.append(f"approached to {m['closest_dist']:.1f}in but outside tolerance")
        if detail:
            print(f"    WP{m['wp']} ({m['target'][0]:.0f},{m['target'][1]:.0f}): "
                  f"{'; '.join(detail)}")


if __name__ == "__main__":
    replay_dir = REPLAY_DIR
    if len(sys.argv) > 1:
        replay_dir = sys.argv[1]

    replays = sorted(glob.glob(os.path.join(replay_dir, "route_*.json")))
    if not replays:
        print(f"No route replay files found in {replay_dir}")
        sys.exit(1)

    results = []
    for rp in replays:
        try:
            r = analyze_replay(rp)
            results.append(r)
            print_report(r)
        except Exception as e:
            print(f"\nERROR processing {rp}: {e}")

    valid = [r for r in results if "error" not in r]
    if not valid:
        sys.exit(0)

    print(f"\n{'=' * 72}")
    print("SUMMARY")
    print(f"{'=' * 72}")
    completed_gt = sum(1 for r in valid
                       if all(w["reached_tick"] is not None for w in r["waypoints_gt"]))
    completed_est = sum(1 for r in valid
                        if all(w["reached_tick"] is not None for w in r["waypoints_est"]))
    print(f"Routes where GT reached all waypoints:  {completed_gt}/{len(valid)}")
    print(f"Routes where EST reached all waypoints: {completed_est}/{len(valid)}")

    avg_mcl = sum(r["mean_mcl_error"] for r in valid) / len(valid)
    avg_wall = sum(r["near_wall_pct"] for r in valid) / len(valid)
    avg_stuck = sum(r["stuck_ticks"] for r in valid) / len(valid)
    avg_wrong = sum(r["wrong_convergence_pct"] for r in valid) / len(valid)
    print(f"Avg MCL error:         {avg_mcl:.1f}in")
    print(f"Avg near-wall time:    {avg_wall:.1f}%")
    print(f"Avg stuck ticks:       {avg_stuck:.0f}")
    print(f"Avg wrong-convergence: {avg_wrong:.1f}%")
