"use client";

import type { GateDecision } from "@/lib/types";

interface Props {
  gate: GateDecision | null | undefined;
}

function mark(ok: boolean): string {
  return ok ? "✅" : "❌";
}

export function GateStatus({ gate }: Props) {
  return (
    <section className="rounded border border-zinc-700 p-3">
      <h2 className="mb-2 text-sm font-semibold">Gate Status</h2>
      {!gate ? (
        <div className="text-xs text-zinc-500">No gate data</div>
      ) : (
        <div className="space-y-1 text-xs text-zinc-200">
          <div>{mark(!gate.failed_centroid_jump)} Centroid jump gate</div>
          <div>{mark(!gate.failed_r90)} R90 gate</div>
          <div>{mark(!gate.failed_passability)} Passability gate</div>
          <div>{mark(!gate.failed_residual)} Residual gate</div>
          <div>{mark(!gate.failed_max_correction)} Max correction gate
            {gate.correction_distance_in != null ? ` (${gate.correction_distance_in.toFixed(1)} in)` : ""}
          </div>
          <div className={gate.accepted ? "text-emerald-300" : "text-red-300"}>
            {gate.accepted ? "✅ Accepted" : "❌ Rejected"}
            {gate.reason ? ` (${gate.reason})` : ""}
          </div>
        </div>
      )}
    </section>
  );
}
