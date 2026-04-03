"use client";

import { useMemo } from "react";

import { gradeSession, type FieldGrade, type Grade } from "@/lib/grading";
import type { SessionAnalytics } from "@/lib/types";

function formatFieldValue(f: FieldGrade): string {
  const { field, value } = f;
  if (field === "gate_precision" || field === "gate_recall" || field === "sensor_dropout_rate") {
    return `${(value * 100).toFixed(1)}%`;
  }
  if (Number.isInteger(value) && Math.abs(value) < 1e9) return String(value);
  return value.toFixed(2);
}

function gradeClass(g: Grade): string {
  switch (g) {
    case "great":
      return "text-emerald-400";
    case "good":
      return "text-sky-400";
    case "fair":
      return "text-amber-300";
    case "poor":
      return "text-orange-400";
    case "bad":
      return "text-red-400";
    default:
      return "text-zinc-300";
  }
}

interface Props {
  analytics: SessionAnalytics;
}

export function AnalyticsPanel({ analytics }: Props) {
  const graded = useMemo(() => gradeSession(analytics), [analytics]);

  return (
    <section className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-3 text-xs text-zinc-200">
      <h2 className="mb-2 text-sm font-semibold text-zinc-100">Session analytics</h2>
      <div className={`mb-3 rounded border border-zinc-600/80 bg-zinc-950/60 px-3 py-2 ${gradeClass(graded.overall)}`}>
        <div className="text-[11px] font-medium uppercase tracking-wide text-zinc-500">Overall</div>
        <div className="text-base font-semibold capitalize">{graded.overall}</div>
        <p className="mt-1 text-zinc-400">{graded.summary}</p>
      </div>

      <div className="mb-3 grid grid-cols-2 gap-x-3 gap-y-1 text-[11px]">
        <Stat label="Peak accepted err (in)" value={analytics.peak_accepted_error.toFixed(2)} />
        <Stat label="Mean / P95 (in)" value={`${analytics.mean_accepted_error.toFixed(2)} / ${analytics.p95_accepted_error.toFixed(2)}`} />
        <Stat label="Oracle threshold (in)" value={analytics.oracle_threshold.toFixed(2)} />
        <Stat label="Gate precision / recall" value={`${(analytics.gate_precision * 100).toFixed(1)}% / ${(analytics.gate_recall * 100).toFixed(1)}%`} />
        <Stat label="False accepts / rejects" value={`${analytics.false_accept_count} / ${analytics.false_reject_count}`} />
        <Stat label="Longest FA / reject streak" value={`${analytics.longest_false_accept_streak} / ${analytics.longest_rejection_streak} ticks`} />
      </div>

      <div className="mb-2 text-[11px] font-medium text-zinc-400">Per-gate rejects (total / false)</div>
      <ul className="mb-3 grid grid-cols-2 gap-1 text-[11px] text-zinc-300">
        <li>R90: {analytics.r90_reject_count} / {analytics.r90_false_reject_count}</li>
        <li>Residual: {analytics.residual_reject_count} / {analytics.residual_false_reject_count}</li>
        <li>Centroid jump: {analytics.centroid_jump_reject_count} / {analytics.centroid_jump_false_reject_count}</li>
        <li>Passability: {analytics.passability_reject_count} / {analytics.passability_false_reject_count}</li>
      </ul>

      <div className="mb-2 text-[11px] font-medium text-zinc-400">Convergence</div>
      <ul className="mb-3 space-y-0.5 text-[11px] text-zinc-300">
        <li>Cold-start ticks: {analytics.cold_start_convergence_ticks} (err {analytics.cold_start_convergence_error.toFixed(2)} in)</li>
        <li>Reacquisitions: {analytics.reacquisition_count} (worst {analytics.worst_reacquisition_ticks}, mean {analytics.mean_reacquisition_ticks.toFixed(1)})</li>
        <li>Post-conv. σ: {analytics.post_convergence_error_stddev.toFixed(2)} in · Peak R90@accept: {analytics.peak_r90_at_acceptance.toFixed(2)} in</li>
      </ul>

      <div className="mb-2 text-[11px] font-medium text-zinc-400">Graded metrics</div>
      <ul className="max-h-40 space-y-1 overflow-y-auto text-[11px]">
        {graded.fields.map((f) => (
          <li key={f.field} className="flex justify-between gap-2 border-b border-zinc-800/60 py-0.5">
            <span className="min-w-0 flex-1 truncate text-zinc-400" title={f.label}>
              {f.label}
            </span>
            <span className={`shrink-0 tabular-nums capitalize ${gradeClass(f.grade)}`}>
              {formatFieldValue(f)} ({f.grade})
            </span>
          </li>
        ))}
      </ul>
    </section>
  );
}

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div>
      <div className="text-zinc-500">{label}</div>
      <div className="tabular-nums text-zinc-100">{value}</div>
    </div>
  );
}
