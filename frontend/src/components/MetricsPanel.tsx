"use client";

import type { TickState } from "@/lib/types";

interface Props {
  tick: TickState | null;
}

function fmt(v: number | null | undefined): string {
  if (v === null || v === undefined || Number.isNaN(v)) return "-";
  return v.toFixed(3);
}

const SENSOR_LABELS = ["Left", "Right", "Front", "Back"] as const;

export function MetricsPanel({ tick }: Props) {
  if (!tick) {
    return (
      <div className="rounded border border-zinc-700 p-3 text-sm text-zinc-400">
        No tick data yet.
      </div>
    );
  }

  const failures = tick.active_failures ?? [];
  const gt = tick.ground_truth;
  const est = tick.post_resample.estimate;
  const ap = tick.accepted_pose;
  const mclPoseDiff =
    ap != null
      ? Math.sqrt((est.x - ap.x) ** 2 + (est.y - ap.y) ** 2)
      : null;

  const gateDecision =
    tick.gate_decision && typeof tick.gate_decision === "object"
      ? (tick.gate_decision as Record<string, unknown>)
      : null;
  const gateAccepted =
    gateDecision && typeof gateDecision.accepted === "boolean"
      ? gateDecision.accepted
      : null;
  const gateReason =
    gateDecision && typeof gateDecision.reason === "string" ? gateDecision.reason : null;
  const radius90In =
    gateDecision && typeof gateDecision.radius_90_in === "number"
      ? gateDecision.radius_90_in
      : null;
  const spreadIn =
    gateDecision && typeof gateDecision.spread_in === "number"
      ? gateDecision.spread_in
      : null;
  const jumpIn =
    gateDecision && typeof gateDecision.jump_in === "number"
      ? gateDecision.jump_in
      : null;

  return (
    <div className="space-y-3 text-sm">
      {/* ---- Pose ---- */}
      <div className="rounded border border-zinc-700 p-3">
        <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-zinc-400">
          Pose
        </h3>
        <div className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-1">
          {gt ? (
            <>
              <span className="text-orange-300">Ground truth</span>
              <span className="font-mono text-xs">
                ({fmt(gt.x)}, {fmt(gt.y)}) @ {fmt(gt.heading_deg)}&deg;
              </span>
            </>
          ) : null}
          <span className="text-blue-300">Estimate</span>
          <span className="font-mono text-xs">
            ({fmt(est.x)}, {fmt(est.y)})
          </span>
          {ap ? (
            <>
              <span className="text-orange-300">Pose</span>
              <span className="font-mono text-xs">
                ({fmt(ap.x)}, {fmt(ap.y)})
              </span>
            </>
          ) : null}
          <span className="text-zinc-300">IMU heading</span>
          <span className="font-mono text-xs">{fmt(tick.observed_heading)}&deg;</span>
          {mclPoseDiff !== null ? (
            <>
              <span className="text-zinc-300">MCL - Pose Diff</span>
              <span
                className={`font-mono text-xs ${
                  mclPoseDiff > 10
                    ? "text-red-400"
                    : mclPoseDiff > 5
                      ? "text-amber-300"
                      : "text-emerald-300"
                }`}
              >
                {fmt(mclPoseDiff)} in
              </span>
            </>
          ) : null}
        </div>
      </div>

      {/* ---- Sensor readings ---- */}
      <div className="rounded border border-zinc-700 p-3">
        <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-zinc-400">
          Sensor readings
        </h3>
        <div className="grid grid-cols-4 gap-1 text-center font-mono text-xs">
          {SENSOR_LABELS.map((label, i) => (
            <div key={label}>
              <div className="text-zinc-400">{label}</div>
              <div
                className={
                  tick.observed_readings[i] < 0
                    ? "text-red-400"
                    : "text-amber-200"
                }
              >
                {tick.observed_readings[i] < 0
                  ? "DEAD"
                  : fmt(tick.observed_readings[i])}
              </div>
            </div>
          ))}
        </div>
        <div className="mt-1 text-center text-xs text-zinc-500">
          Valid: {tick.valid_sensor_count} / 4
          {tick.valid_sensor_count === 0 ? (
            <span className="ml-1 font-semibold text-red-400">BLIND</span>
          ) : null}
        </div>
      </div>

      {/* ---- Metrics ---- */}
      <div className="rounded border border-zinc-700 p-3">
        <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-zinc-400">
          Metrics
        </h3>
        <div className="grid grid-cols-2 gap-x-3 gap-y-1">
          <span>Tick</span>
          <span>{tick.tick}</span>
          <span>MCL error</span>
          <span>{fmt(tick.mcl_error)} in</span>
          <span>Odom error</span>
          <span>{fmt(tick.odom_error)} in</span>
          <span>N eff</span>
          <span>{fmt(tick.post_resample.n_eff)}</span>
        </div>
      </div>

      {/* ---- Gate decision ---- */}
      <div className="rounded border border-zinc-700 p-3">
        <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-zinc-400">
          Gate decision
        </h3>
        <div className="grid grid-cols-2 gap-x-3 gap-y-1">
          <span>Status</span>
          <span
            className={
              gateAccepted === true
                ? "text-emerald-300"
                : gateAccepted === false
                  ? "text-red-300"
                  : "text-zinc-400"
            }
          >
            {gateAccepted === null ? "-" : gateAccepted ? "ACCEPTED" : "REJECTED"}
          </span>
          <span>Reason</span>
          <span>{gateReason ?? "-"}</span>
          <span>Radius 90</span>
          <span>{fmt(radius90In)} in</span>
          <span>Spread</span>
          <span>{fmt(spreadIn)} in</span>
          <span>Jump</span>
          <span>{fmt(jumpIn)} in</span>
        </div>
        {gateDecision ? (
          <div className="mt-2 text-xs text-zinc-300">
            {[
              "failed_spread",
              "failed_passability",
              "failed_residual",
              "failed_wall_sum",
              "failed_velocity",
            ]
              .filter((key) => gateDecision[key] === true)
              .join(", ") || "No gate failures"}
          </div>
        ) : null}
      </div>

      {/* ---- Active failures ---- */}
      {failures.length > 0 ? (
        <div className="rounded border border-amber-800/50 bg-amber-900/10 p-3">
          <h3 className="mb-1 text-xs font-semibold uppercase tracking-wide text-amber-400">
            Active failures ({failures.length})
          </h3>
          <div className="text-xs text-amber-300">{failures.join(", ")}</div>
        </div>
      ) : null}
    </div>
  );
}
