"use client";

import { Fragment } from "react";

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

  const tickIndex = isTickState(tick) ? tick.tick : (tick.tick_count ?? null);
  const nEff = tick.post_resample?.n_eff ?? (isTickState(tick) ? 0 : (tick.n_eff ?? 0));
  const spread = isTickState(tick) ? tick.post_resample?.spread : tick.cluster_stats?.spread;
  const radius90 = isTickState(tick) ? tick.post_resample?.radius_90 : tick.cluster_stats?.radius_90;
  const gateAccepted = isTickState(tick) ? null : (tick.gate?.accepted ?? null);
  const gateReason = isTickState(tick) ? null : (tick.gate?.reason ?? null);
  const sensorLabels = ["L", "R", "F", "B"] as const;

  const residualClass = (value: number) => {
    if (value < 1.5) return "text-emerald-300";
    if (value < 4.0) return "text-amber-300";
    return "text-red-300";
  };

  return (
    <div className="rounded border border-zinc-700 p-3 text-sm">
      <h3 className="mb-2 text-sm font-semibold">Metrics</h3>
      <div className="grid grid-cols-2 gap-x-3 gap-y-1">
        <span>Tick</span>
        <span>{tickIndex ?? "N/A"}</span>
        <span>MCL error</span>
        <span className={isTickState(tick) && tick.mcl_error != null ? "" : "text-zinc-600"}>
          {isTickState(tick) && tick.mcl_error != null ? `${formatNumber(tick.mcl_error)} in` : "N/A"}
        </span>
        <span>Odom error</span>
        <span className={isTickState(tick) && tick.odom_error != null ? "" : "text-zinc-600"}>
          {isTickState(tick) && tick.odom_error != null ? `${formatNumber(tick.odom_error)} in` : "N/A"}
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
            (tick.valid_sensor_count ?? 0) === 0
              ? "font-semibold text-red-400"
              : (tick.valid_sensor_count ?? 0) < 4
                ? "text-amber-300"
                : ""
          }
        >
          {tick.valid_sensor_count ?? 0}
          {(tick.valid_sensor_count ?? 0) === 0 ? " (BLIND)" : ""}
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

      {/* Position debug section */}
      {isTickState(tick) ? (
        <div className="mt-3 border-t border-zinc-800 pt-2 text-xs">
          <div className="mb-1 font-semibold text-zinc-300">Position (inches)</div>
          <div className="grid grid-cols-4 gap-y-1 text-zinc-400">
            <span />
            <span className="text-center">X</span>
            <span className="text-center">Y</span>
            <span className="text-center">θ</span>

            <span className="text-cyan-400">Chassis</span>
            <span className="text-center text-zinc-200">{tick.chassis_pose ? formatNumber(tick.chassis_pose.x) : "-"}</span>
            <span className="text-center text-zinc-200">{tick.chassis_pose ? formatNumber(tick.chassis_pose.y) : "-"}</span>
            <span className="text-center text-zinc-200">{tick.chassis_pose ? formatNumber(tick.chassis_pose.theta) : "-"}</span>

            <span className="text-blue-400">MCL</span>
            <span className="text-center text-zinc-200">{tick.raw_estimate ? formatNumber(tick.raw_estimate.x) : "-"}</span>
            <span className="text-center text-zinc-200">{tick.raw_estimate ? formatNumber(tick.raw_estimate.y) : "-"}</span>
            <span className="text-center text-zinc-200">{tick.raw_estimate ? formatNumber(tick.raw_estimate.theta) : "-"}</span>

            <span className="text-emerald-400">Raw Odom</span>
            <span className="text-center text-zinc-200">{tick.raw_odom ? formatNumber(tick.raw_odom.x) : "-"}</span>
            <span className="text-center text-zinc-200">{tick.raw_odom ? formatNumber(tick.raw_odom.y) : "-"}</span>
            <span className="text-center text-zinc-200">{tick.raw_odom ? formatNumber(tick.raw_odom.theta) : "-"}</span>
          </div>
        </div>
      ) : null}

      {!isTickState(tick) && tick.observed_readings && tick.mcl_sensor_residuals ? (
        <div className="mt-3 border-t border-zinc-800 pt-2 text-xs">
          <div className="mb-1 font-semibold text-zinc-300">Sensor Diagnostics</div>
          <div className="grid grid-cols-3 gap-y-1 text-zinc-400">
            <span>Sensor</span>
            <span>Reading</span>
            <span>Residual</span>
            {sensorLabels.map((label, i) => {
              const reading = tick.observed_readings?.[i] ?? -1;
              const residual = tick.mcl_sensor_residuals?.[i] ?? 0;
              const readingText = reading < 0 ? "invalid" : `${formatNumber(reading)} in`;
              const residualText = reading < 0 ? "-" : `${formatNumber(residual)} in`;
              return (
                <Fragment key={label}>
                  <span className="text-zinc-200">
                    {label}
                  </span>
                  <span className={reading < 0 ? "text-zinc-600" : "text-zinc-300"}>
                    {readingText}
                  </span>
                  <span className={reading < 0 ? "text-zinc-600" : residualClass(residual)}>
                    {residualText}
                  </span>
                </Fragment>
              );
            })}
          </div>
        </div>
      ) : null}
    </div>
  );
}
