"use client";

import { useMemo } from "react";

import { parseFailures } from "@/lib/failures";
import { isTickState } from "@/lib/types";
import type { AnyTick } from "@/lib/types";

interface Props {
  tick: AnyTick | null;
}

function severityClass(sev: "info" | "warning" | "critical"): string {
  if (sev === "critical") return "border-red-500/80 bg-red-950/50 text-red-200";
  if (sev === "warning") return "border-amber-500/70 bg-amber-950/40 text-amber-200";
  return "border-zinc-600 bg-zinc-800/40 text-zinc-200";
}

export function FailureOverlay({ tick }: Props) {
  const parsed = useMemo(
    () => parseFailures(tick && isTickState(tick) ? tick.active_failures : []),
    [tick],
  );
  const worst = useMemo(() => {
    if (parsed.some((p) => p.severity === "critical")) return "critical";
    if (parsed.some((p) => p.severity === "warning")) return "warning";
    return "info";
  }, [parsed]);

  const panelAccent =
    worst === "critical"
      ? "border-l-4 border-red-500 bg-red-950/20"
      : worst === "warning"
        ? "border-l-4 border-amber-500 bg-amber-950/10"
        : "border-l-4 border-zinc-700";

  return (
    <div className={`rounded border border-zinc-700 p-3 ${panelAccent}`}>
      <h3 className="mb-2 text-sm font-semibold">Active Failures</h3>
      {parsed.length === 0 ? (
        <p className="text-xs text-emerald-700">No active failures</p>
      ) : (
        <div className="space-y-2">
          {parsed.map((p, idx) => (
            <div
              key={`${p.label}-${idx}`}
              className={`rounded border px-2 py-1 text-xs transition-all duration-200 ${severityClass(p.severity)}`}
            >
              {p.label}
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

