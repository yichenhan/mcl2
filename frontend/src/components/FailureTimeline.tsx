"use client";

import { useMemo } from "react";

import { parseFailures } from "@/lib/failures";
import type { TickState } from "@/lib/types";

interface Props {
  ticks: TickState[];
  cursor: number;
  onJump: (tick: number) => void;
}

const ROWS = ["S0", "S1", "S2", "S3", "ODM", "HDG", "K"] as const;

function rowIndexForFailure(type: string, sensor?: number): number | null {
  if (type === "sensor_dead" || type === "sensor_stuck" || type === "spurious_reflection") {
    if (sensor === undefined || sensor < 0 || sensor > 3) return null;
    return sensor;
  }
  if (type === "odom_spike") return 4;
  if (type === "heading_bias") return 5;
  if (type === "kidnap") return 6;
  return null;
}

export function FailureTimeline({ ticks, cursor, onJump }: Props) {
  const total = ticks.length;

  const cells = useMemo(() => {
    return ticks.map((t) => parseFailures(t.active_failures));
  }, [ticks]);

  if (total <= 0) {
    return (
      <div className="rounded border border-zinc-700 p-2 text-xs text-zinc-500">
        No timeline data.
      </div>
    );
  }

  return (
    <div className="rounded border border-zinc-700 p-2">
      <div className="mb-1 text-[10px] text-zinc-400">Failure timeline</div>
      <div className="space-y-1">
        {ROWS.map((row, rowIdx) => (
          <div key={row} className="flex items-center gap-2">
            <div className="w-6 text-[10px] text-zinc-500">{row}</div>
            <div className="relative flex-1">
              <div className="flex h-2 w-full overflow-hidden rounded bg-zinc-900">
                {cells.map((failures, idx) => {
                  const here = failures.some((f) => rowIndexForFailure(f.type, f.sensor) === rowIdx);
                  const color = failures.find((f) => rowIndexForFailure(f.type, f.sensor) === rowIdx)?.color;
                  return (
                    <button
                      key={`${row}-${idx}`}
                      type="button"
                      className="h-2 flex-1"
                      style={{ backgroundColor: here ? color : "transparent" }}
                      onClick={() => onJump(idx)}
                      title={here ? `tick ${idx}` : undefined}
                    />
                  );
                })}
              </div>
              <div
                className="pointer-events-none absolute top-0 h-2 w-[2px] bg-white"
                style={{ left: `${(Math.min(cursor, total - 1) / Math.max(1, total - 1)) * 100}%` }}
              />
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

