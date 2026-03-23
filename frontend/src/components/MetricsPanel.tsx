"use client";

import { isTickState } from "@/lib/types";
import type { AnyTick } from "@/lib/types";

interface Props {
  tick: AnyTick | null;
}

function formatNumber(v: number | null): string {
  if (v === null || Number.isNaN(v)) return "-";
  return v.toFixed(3);
}

export function MetricsPanel({ tick }: Props) {
  if (!tick) {
    return (
      <div className="rounded border border-zinc-700 p-3 text-sm text-zinc-400">
        No tick data yet.
      </div>
    );
  }

  const tickIndex = isTickState(tick) ? tick.tick : null;
  const nEff = tick.post_resample.n_eff;
  const spread = isTickState(tick) ? tick.post_resample.spread : tick.cluster_stats.spread;
  const radius90 = isTickState(tick) ? tick.post_resample.radius_90 : tick.cluster_stats.radius_90;
  const gateAccepted = isTickState(tick) ? null : tick.gate.accepted;
  const gateReason = isTickState(tick) ? null : tick.gate.reason;

  return (
    <div className="rounded border border-zinc-700 p-3 text-sm">
      <h3 className="mb-2 text-sm font-semibold">Metrics</h3>
      <div className="grid grid-cols-2 gap-x-3 gap-y-1">
        <span>Tick</span>
        <span>{tickIndex ?? "N/A"}</span>
        <span>MCL error</span>
        <span className={isTickState(tick) ? "" : "text-zinc-600"}>
          {isTickState(tick) ? `${formatNumber(tick.mcl_error)} in` : "N/A"}
        </span>
        <span>Odom error</span>
        <span className={isTickState(tick) ? "" : "text-zinc-600"}>
          {isTickState(tick) ? `${formatNumber(tick.odom_error)} in` : "N/A"}
        </span>
        <span>N eff</span>
        <span>{formatNumber(nEff)}</span>
        <span>Spread</span>
        <span>{formatNumber(spread ?? null)} in</span>
        <span>Radius 90</span>
        <span>{formatNumber(radius90 ?? null)} in</span>
        <span>Valid sensors</span>
        <span
          className={
            tick.valid_sensor_count === 0
              ? "font-semibold text-red-400"
              : tick.valid_sensor_count < 4
                ? "text-amber-300"
                : ""
          }
        >
          {tick.valid_sensor_count}
          {tick.valid_sensor_count === 0 ? " (BLIND)" : ""}
        </span>
        <span>Active failures</span>
        <span
          className={
            !isTickState(tick)
              ? "text-zinc-600"
              : tick.active_failures.length === 0
              ? "text-emerald-700"
              : tick.active_failures.some((f) => f.includes("kidnap"))
                ? "text-red-300"
                : "text-amber-300"
          }
        >
          {isTickState(tick) ? tick.active_failures.length : "N/A"}
        </span>
        <span>Gate</span>
        <span className={gateAccepted === null ? "text-zinc-600" : gateAccepted ? "text-emerald-400" : "text-red-300"}>
          {gateAccepted === null ? "N/A" : gateAccepted ? "accepted" : "rejected"}
        </span>
      </div>
      {isTickState(tick) && tick.active_failures.length > 0 ? (
        <div className="mt-2 text-xs text-amber-300">
          {tick.active_failures.join(", ")}
        </div>
      ) : null}
      {!isTickState(tick) && gateReason ? (
        <div className="mt-2 text-xs text-zinc-300">Reason: {gateReason}</div>
      ) : null}
    </div>
  );
}
