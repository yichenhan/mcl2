"use client";

import { Fragment } from "react";

import { isTickState } from "@/lib/types";
import type { AnyTick } from "@/lib/types";

interface Props {
  tick: AnyTick | null;
}

function fmt(v: unknown, decimals = 3): string {
  if (v == null) return "-";
  const n = typeof v === "number" ? v : Number(v);
  if (!Number.isFinite(n)) return "-";
  return n.toFixed(decimals);
}

function Row({ label, value, unit, className }: { label: string; value: string; unit?: string; className?: string }) {
  return (
    <>
      <span className="text-zinc-400">{label}</span>
      <span className={className ?? "text-zinc-200"}>
        {value}{unit ? ` ${unit}` : ""}
      </span>
    </>
  );
}

function SectionHeader({ children }: { children: React.ReactNode }) {
  return (
    <div className="col-span-2 mt-2 border-t border-zinc-800 pt-2 text-[11px] font-semibold uppercase tracking-wider text-zinc-500">
      {children}
    </div>
  );
}

export function MetricsPanel({ tick }: Props) {
  if (!tick) {
    return (
      <div className="rounded border border-zinc-700 p-3 text-sm text-zinc-400">
        No tick data yet.
      </div>
    );
  }

  const ts = isTickState(tick) ? tick : null;
  const mcl = !isTickState(tick) ? tick : null;
  const tickIndex = ts ? ts.tick : (mcl?.tick_count ?? null);
  const nEff = tick.post_update?.n_eff ?? (mcl?.n_eff ?? 0);
  const totalParticles = tick.post_resample?.n_eff ?? nEff;
  const nEffPct = totalParticles > 0 ? (nEff / totalParticles) * 100 : 0;
  const spread = ts ? ts.post_resample?.spread : mcl?.cluster_stats?.spread;
  const radius90 = ts ? ts.post_resample?.radius_90 : mcl?.cluster_stats?.radius_90;
  const gate = ts?.gate_decision ?? mcl?.gate ?? null;
  const sensorLabels = ["L", "R", "F", "B"] as const;

  const nEffColor =
    nEffPct > 50 ? "text-emerald-400" :
    nEffPct > 20 ? "text-amber-300" :
    "text-red-400";

  const sensorCountColor =
    (tick.valid_sensor_count ?? 0) === 0 ? "font-semibold text-red-400" :
    (tick.valid_sensor_count ?? 0) < 4 ? "text-amber-300" :
    "text-zinc-200";

  const residualColor = (v: number) =>
    v < 1.5 ? "text-emerald-300" : v < 4.0 ? "text-amber-300" : "text-red-300";

  return (
    <div className="rounded border border-zinc-700 p-3 text-xs">
      <div className="grid grid-cols-2 gap-x-4 gap-y-0.5">

        {/* ─── General ─── */}
        <Row label="Tick" value={tickIndex != null ? String(tickIndex) : "N/A"} />
        <Row label="Sensors" value={`${tick.valid_sensor_count ?? 0} / 4`} className={sensorCountColor} />

        {/* ─── Particle Filter ─── */}
        <SectionHeader>Particle Filter</SectionHeader>
        <Row label="N eff" value={`${fmt(nEff, 1)} / ${fmt(totalParticles, 0)}`} className={nEffColor} />
        <Row label="Utilization" value={`${fmt(nEffPct, 1)}%`} className={nEffColor} />
        <Row label="Spread" value={fmt(spread)} unit="in" />
        <Row label="R90" value={fmt(radius90)} unit="in" />

        {/* ─── Gate ─── */}
        <SectionHeader>Gate Decision</SectionHeader>
        <Row
          label="Status"
          value={gate ? (gate.accepted ? "Accepted" : "Rejected") : "N/A"}
          className={!gate ? "text-zinc-600" : gate.accepted ? "text-emerald-400" : "text-red-400"}
        />
        {gate && !gate.accepted && gate.reason ? (
          <Row label="Reason" value={gate.reason} className="text-zinc-300" />
        ) : null}
        {gate ? (
          <>
            <Row label="R90" value={fmt(gate.radius_90_in)} unit="in" className={gate.failed_r90 ? "text-red-400" : "text-zinc-200"} />
            <Row label="Jump" value={fmt(gate.jump_in)} unit="in" className={gate.failed_centroid_jump ? "text-red-400" : "text-zinc-200"} />
            <Row label="Max residual" value={fmt(gate.max_sensor_residual_in)} unit="in" className={gate.failed_residual ? "text-red-400" : "text-zinc-200"} />
            <Row label="Correction" value={fmt(gate.correction_distance_in, 2)} unit="in" className={gate.failed_max_correction ? "text-red-400" : "text-zinc-200"} />
          </>
        ) : null}

        {/* ─── Position ─── */}
        {ts ? (
          <>
            <SectionHeader>Position (inches)</SectionHeader>
            <div className="col-span-2 grid grid-cols-4 gap-y-0.5 text-zinc-400">
              <span />
              <span className="text-center text-[10px] font-medium text-zinc-500">X</span>
              <span className="text-center text-[10px] font-medium text-zinc-500">Y</span>
              <span className="text-center text-[10px] font-medium text-zinc-500">θ</span>

              <span className="text-cyan-400">Chassis</span>
              <span className="text-center text-zinc-200">{ts.chassis_pose ? fmt(ts.chassis_pose.x) : "-"}</span>
              <span className="text-center text-zinc-200">{ts.chassis_pose ? fmt(ts.chassis_pose.y) : "-"}</span>
              <span className="text-center text-zinc-200">{ts.chassis_pose ? fmt(ts.chassis_pose.theta, 1) : "-"}</span>

              <span className="text-blue-400">MCL</span>
              <span className="text-center text-zinc-200">{ts.raw_estimate ? fmt(ts.raw_estimate.x) : "-"}</span>
              <span className="text-center text-zinc-200">{ts.raw_estimate ? fmt(ts.raw_estimate.y) : "-"}</span>
              <span className="text-center text-zinc-200">{ts.raw_estimate ? fmt(ts.raw_estimate.theta, 1) : "-"}</span>

              <span className="text-emerald-400">Raw Odom</span>
              <span className="text-center text-zinc-200">{ts.raw_odom ? fmt(ts.raw_odom.x) : "-"}</span>
              <span className="text-center text-zinc-200">{ts.raw_odom ? fmt(ts.raw_odom.y) : "-"}</span>
              <span className="text-center text-zinc-200">{ts.raw_odom ? fmt(ts.raw_odom.theta, 1) : "-"}</span>
            </div>
          </>
        ) : null}

        {/* ─── Sensors ─── */}
        {(() => {
          const readings = ts?.observed_readings ?? mcl?.observed_readings;
          const residuals = ts?.sensor_residuals ?? mcl?.mcl_sensor_residuals;
          const predicted = ts?.mcl_predicted_readings ?? mcl?.mcl_predicted_readings;
          if (!readings) return null;
          return (
            <>
              <SectionHeader>Sensors</SectionHeader>
              <div className="col-span-2 grid grid-cols-4 gap-y-0.5 text-zinc-400">
                <span className="text-[10px] font-medium text-zinc-500">Sensor</span>
                <span className="text-center text-[10px] font-medium text-zinc-500">Read</span>
                <span className="text-center text-[10px] font-medium text-zinc-500">Pred</span>
                <span className="text-center text-[10px] font-medium text-zinc-500">Resid</span>
                {sensorLabels.map((label, i) => {
                  const reading = readings[i] ?? -1;
                  const pred = predicted?.[i] ?? -1;
                  const residual = residuals?.[i] ?? 0;
                  const invalid = reading < 0;
                  return (
                    <Fragment key={label}>
                      <span className="text-zinc-200">{label}</span>
                      <span className={`text-center ${invalid ? "text-zinc-600" : "text-zinc-300"}`}>
                        {invalid ? "—" : fmt(reading, 1)}
                      </span>
                      <span className={`text-center ${invalid || pred < 0 ? "text-zinc-600" : "text-zinc-300"}`}>
                        {invalid || pred < 0 ? "—" : fmt(pred, 1)}
                      </span>
                      <span className={`text-center ${invalid ? "text-zinc-600" : residualColor(residual)}`}>
                        {invalid ? "—" : fmt(residual, 2)}
                      </span>
                    </Fragment>
                  );
                })}
              </div>
            </>
          );
        })()}

        {/* ─── Failures ─── */}
        {ts && ts.active_failures.length > 0 ? (
          <>
            <SectionHeader>Active Failures</SectionHeader>
            <div className="col-span-2 text-amber-300">
              {ts.active_failures.join(", ")}
            </div>
          </>
        ) : null}
      </div>
    </div>
  );
}
