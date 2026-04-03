"use client";

import { useCallback, useEffect, useMemo, useState } from "react";

import { api } from "@/lib/api";
import { parseSessionAnalytics } from "@/lib/sessionAnalyticsDefaults";
import type { SessionAnalytics } from "@/lib/types";

// ---------------------------------------------------------------------------
// KPI threshold map — mirrors backend kpi_thresholds() exactly.
// Update both places when changing a threshold.
// ---------------------------------------------------------------------------
type KpiId =
  | "convergenceLatency"
  | "peakAcceptedError"
  | "gatePrecision"
  | "postConvStdDev"
  | "gateRecall"
  | "reacqLatency";

interface KpiThreshold {
  good: number;
  great: number;
  lowerIsBetter: boolean;
}

const KPI_THRESHOLDS: Record<KpiId, KpiThreshold> = {
  convergenceLatency: { good: 10,   great: 4,    lowerIsBetter: true  },
  peakAcceptedError:  { good: 6,    great: 3,    lowerIsBetter: true  },
  gatePrecision:      { good: 0.90, great: 0.97, lowerIsBetter: false },
  postConvStdDev:     { good: 2,    great: 1,    lowerIsBetter: true  },
  gateRecall:         { good: 0.80, great: 0.92, lowerIsBetter: false },
  reacqLatency:       { good: 15,   great: 5,    lowerIsBetter: true  },
};

type KpiRating = "great" | "good" | "failing";

function kpiRating(id: KpiId, v: number | undefined | null): KpiRating {
  if (v == null) return "failing";
  // No reacquisition event occurred (-1) → trivially great
  if (id === "reacqLatency" && v < 0) return "great";
  // Never converged (-1) → failing
  if (id === "convergenceLatency" && v < 0) return "failing";
  const t = KPI_THRESHOLDS[id];
  const isGreat = t.lowerIsBetter ? v <= t.great : v >= t.great;
  const isGood  = t.lowerIsBetter ? v <= t.good  : v >= t.good;
  return isGreat ? "great" : isGood ? "good" : "failing";
}

const RATING_CLASS: Record<KpiRating, string> = {
  great:   "text-emerald-400",
  good:    "text-yellow-400",
  failing: "text-red-400",
};

function kpiClass(id: KpiId, v: number | undefined | null): string {
  return RATING_CLASS[kpiRating(id, v)];
}

// ---------------------------------------------------------------------------
// Sort keys
// ---------------------------------------------------------------------------
type SortKey =
  | "file"
  | "total_ticks"
  | "peak_accepted_error"
  | "false_accept_count"
  | "false_reject_count"
  | "gate_precision"
  | "gate_recall"
  | "cold_start_convergence_ticks"
  | "post_convergence_error_stddev"
  | "worst_reacquisition_ticks";

interface Row {
  file: string;
  meta: Record<string, unknown> | null;
  analytics: SessionAnalytics | null;
  error?: string;
}

interface Props {
  files: string[];
  onLoadReplay: (file: string, peakAcceptedErrorTick: number) => void;
  onRefresh?: () => void;
}

function sortValue(row: Row, key: SortKey): string | number {
  if (key === "file") return row.file.toLowerCase();
  const a = row.analytics;
  if (!a) {
    if (key === "total_ticks") return -1;
    if (key === "gate_precision" || key === "gate_recall") return -1;
    return Number.POSITIVE_INFINITY;
  }
  switch (key) {
    case "total_ticks":                    return a.total_ticks;
    case "peak_accepted_error":            return a.peak_accepted_error;
    case "false_accept_count":             return a.false_accept_count;
    case "false_reject_count":             return a.false_reject_count;
    case "gate_precision":                 return a.gate_precision;
    case "gate_recall":                    return a.gate_recall;
    case "cold_start_convergence_ticks":   return a.cold_start_convergence_ticks;
    case "post_convergence_error_stddev":  return a.post_convergence_error_stddev;
    case "worst_reacquisition_ticks":      return a.worst_reacquisition_ticks;
    default:                               return 0;
  }
}

export function ReplayBrowser({ files, onLoadReplay, onRefresh }: Props) {
  const [rows, setRows] = useState<Row[]>([]);
  const [sortKey, setSortKey] = useState<SortKey>("peak_accepted_error");
  const [sortDir, setSortDir] = useState<"asc" | "desc">("desc");

  useEffect(() => {
    let cancelled = false;
    setRows(files.map((f) => ({ file: f, meta: null, analytics: null })));

    void (async () => {
      const next: Row[] = await Promise.all(
        files.map(async (file) => {
          try {
            const meta = await api.getReplayMeta(file);
            if (cancelled) return { file, meta: null, analytics: null };
            const analytics = parseSessionAnalytics(meta.analytics);
            return { file, meta, analytics };
          } catch (e) {
            return {
              file,
              meta: null,
              analytics: null,
              error: e instanceof Error ? e.message : "meta error",
            };
          }
        }),
      );
      if (!cancelled) setRows(next);
    })();

    return () => { cancelled = true; };
  }, [files]);

  const sorted = useMemo(() => {
    const copy = [...rows];
    copy.sort((a, b) => {
      const va = sortValue(a, sortKey);
      const vb = sortValue(b, sortKey);
      const cmp =
        typeof va === "string" && typeof vb === "string"
          ? va.localeCompare(vb)
          : Number(va) - Number(vb);
      return sortDir === "asc" ? cmp : -cmp;
    });
    return copy;
  }, [rows, sortKey, sortDir]);

  const toggleSort = useCallback((key: SortKey) => {
    setSortKey((prev) => {
      if (prev === key) {
        setSortDir((d) => (d === "asc" ? "desc" : "asc"));
        return prev;
      }
      setSortDir(key === "file" ? "asc" : "desc");
      return key;
    });
  }, []);

  const header = (key: SortKey, label: string) => (
    <th className="whitespace-nowrap px-2 py-2 text-left">
      <button
        type="button"
        className="text-emerald-400/90 hover:underline"
        onClick={() => toggleSort(key)}
      >
        {label}
        {sortKey === key ? (sortDir === "asc" ? " ▲" : " ▼") : ""}
      </button>
    </th>
  );

  return (
    <div className="space-y-3">
      <div className="flex flex-wrap items-center gap-2">
        {onRefresh ? (
          <button
            type="button"
            className="rounded-lg border border-zinc-600 px-3 py-1.5 text-xs text-zinc-200 hover:bg-zinc-800"
            onClick={onRefresh}
          >
            Refresh list
          </button>
        ) : null}
        <p className="text-xs text-zinc-500">Click a row to load and jump to the peak-error tick when known.</p>
        <div className="flex items-center gap-3 text-xs">
          <span className="text-emerald-400">● Great</span>
          <span className="text-yellow-400">● Good</span>
          <span className="text-red-400">● Failing</span>
        </div>
      </div>
      <div className="max-h-[min(50vh,420px)] overflow-auto rounded-lg border border-zinc-700">
        <table className="w-full min-w-[900px] border-collapse text-left text-xs text-zinc-200">
          <thead className="sticky top-0 z-10 bg-zinc-900/95 backdrop-blur">
            <tr className="border-b border-zinc-700">
              {header("file", "File")}
              {header("total_ticks", "Ticks")}
              {header("cold_start_convergence_ticks", "Conv (ticks)")}
              {header("peak_accepted_error", "Peak err (in)")}
              {header("gate_precision", "Precision")}
              {header("post_convergence_error_stddev", "Std Dev (in)")}
              {header("gate_recall", "Recall")}
              {header("worst_reacquisition_ticks", "Reacq (ticks)")}
              {header("false_accept_count", "False acc")}
              {header("false_reject_count", "False rej")}
            </tr>
          </thead>
          <tbody>
            {sorted.length === 0 ? (
              <tr>
                <td colSpan={10} className="px-3 py-6 text-center text-zinc-500">
                  No replays on server.
                </td>
              </tr>
            ) : (
              sorted.map((row) => {
                const a = row.analytics;
                const peakTick = a?.peak_accepted_error_tick ?? 0;
                // reacq -1 means no event → display as "—" but rate as great
                const reacqDisplay =
                  a == null ? "—"
                  : a.worst_reacquisition_ticks < 0 ? "—"
                  : String(a.worst_reacquisition_ticks);
                const convDisplay =
                  a == null ? "—"
                  : a.cold_start_convergence_ticks < 0 ? "∞"
                  : String(a.cold_start_convergence_ticks);
                return (
                  <tr
                    key={row.file}
                    className="cursor-pointer border-b border-zinc-800/80 hover:bg-zinc-800/40"
                    onClick={() => onLoadReplay(row.file, peakTick)}
                  >
                    <td className="max-w-[180px] truncate px-2 py-2 font-mono text-[11px]" title={row.file}>
                      {row.file}
                    </td>
                    <td className="tabular-nums px-2 py-2">{a?.total_ticks ?? "—"}</td>

                    {/* Convergence latency */}
                    <td className={`tabular-nums px-2 py-2 ${kpiClass("convergenceLatency", a?.cold_start_convergence_ticks)}`}>
                      {convDisplay}
                    </td>

                    {/* Peak accepted error */}
                    <td className={`tabular-nums px-2 py-2 ${kpiClass("peakAcceptedError", a?.peak_accepted_error)}`}>
                      {a != null ? a.peak_accepted_error.toFixed(2) : "—"}
                    </td>

                    {/* Gate precision */}
                    <td className={`tabular-nums px-2 py-2 ${kpiClass("gatePrecision", a?.gate_precision)}`}>
                      {a != null ? (a.gate_precision * 100).toFixed(1) + "%" : "—"}
                    </td>

                    {/* Post-conv std dev */}
                    <td className={`tabular-nums px-2 py-2 ${kpiClass("postConvStdDev", a?.post_convergence_error_stddev)}`}>
                      {a != null ? a.post_convergence_error_stddev.toFixed(2) : "—"}
                    </td>

                    {/* Gate recall */}
                    <td className={`tabular-nums px-2 py-2 ${kpiClass("gateRecall", a?.gate_recall)}`}>
                      {a != null ? (a.gate_recall * 100).toFixed(1) + "%" : "—"}
                    </td>

                    {/* Reacquisition latency */}
                    <td className={`tabular-nums px-2 py-2 ${kpiClass("reacqLatency", a?.worst_reacquisition_ticks)}`}>
                      {reacqDisplay}
                    </td>

                    <td className="tabular-nums px-2 py-2">{a?.false_accept_count ?? "—"}</td>
                    <td className="tabular-nums px-2 py-2">{a?.false_reject_count ?? "—"}</td>
                  </tr>
                );
              })
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
}
