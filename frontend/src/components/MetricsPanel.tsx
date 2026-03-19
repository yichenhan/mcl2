"use client";

import type { TickState } from "@/lib/types";

interface Props {
  tick: TickState | null;
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

  return (
    <div className="rounded border border-zinc-700 p-3 text-sm">
      <h3 className="mb-2 text-sm font-semibold">Metrics</h3>
      <div className="grid grid-cols-2 gap-x-3 gap-y-1">
        <span>Tick</span>
        <span>{tick.tick}</span>
        <span>MCL error</span>
        <span>{formatNumber(tick.mcl_error)} in</span>
        <span>Odom error</span>
        <span>{formatNumber(tick.odom_error)} in</span>
        <span>N eff</span>
        <span>{formatNumber(tick.post_resample.n_eff)}</span>
        <span>Valid sensors</span>
        <span>{tick.valid_sensor_count}</span>
      </div>
      {tick.active_failures.length > 0 ? (
        <div className="mt-2 text-xs text-amber-300">
          {tick.active_failures.join(", ")}
        </div>
      ) : null}
    </div>
  );
}
