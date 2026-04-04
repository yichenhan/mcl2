"use client";

import type { ScrubberAnnotation } from "@/lib/types";

interface Props {
  cursor: number;
  totalTicks: number;
  onSeek: (tick: number) => void;
  onStep: (delta: number) => void;
  /** Horizontal toolbar for header — no section title, tighter padding. */
  compact?: boolean;
  /** Fixed replay bar at bottom of viewport — full width, flat top border handled by parent. */
  variant?: "default" | "bottomBar";
  annotations?: ScrubberAnnotation[];
}

export function TransportControls({
  cursor,
  totalTicks,
  onSeek,
  onStep,
  compact = false,
  variant = "default",
  annotations = [],
}: Props) {
  const buttonRow = (
    <div className="flex items-center gap-1.5">
      <button
        type="button"
        className="flex items-center gap-1 rounded bg-zinc-800 px-2.5 py-1 text-xs hover:bg-zinc-700 disabled:opacity-40"
        onClick={() => onStep(-1)}
        disabled={cursor <= 0}
        aria-label="Previous tick"
      >
        <svg width="12" height="12" viewBox="0 0 12 12" fill="currentColor" aria-hidden>
          <path d="M8 1.5 3.5 6 8 10.5V1.5Z" />
        </svg>
        Prev
      </button>
      <button
        type="button"
        className="flex items-center gap-1 rounded bg-zinc-800 px-2.5 py-1 text-xs hover:bg-zinc-700 disabled:opacity-40"
        onClick={() => onStep(1)}
        disabled={cursor >= totalTicks - 1}
        aria-label="Next tick"
      >
        Next
        <svg width="12" height="12" viewBox="0 0 12 12" fill="currentColor" aria-hidden>
          <path d="M4 1.5 8.5 6 4 10.5V1.5Z" />
        </svg>
      </button>
    </div>
  );

  const denom = Math.max(1, totalTicks - 1);

  const markerColor = (t: ScrubberAnnotation["type"]) => {
    switch (t) {
      case "error_peak":
        return "bg-rose-500";
      case "false_accept":
        return "bg-amber-400";
      case "rejection_streak":
        return "bg-zinc-500";
      case "convergence":
        return "bg-emerald-500";
      default:
        return "bg-zinc-400";
    }
  };

  const scrubber = (
    <div className={compact ? "relative min-w-[160px] flex-1 pt-2" : "relative w-full pt-2"}>
      {totalTicks > 1 && annotations.length > 0 ? (
        <div
          className="pointer-events-none absolute left-0 right-0 top-0 z-0 h-3"
          role="group"
          aria-label="Replay event markers"
        >
          {annotations.map((a, i) => {
            const end = a.endTick ?? a.tick;
            const leftPct = (Math.min(a.tick, end) / denom) * 100;
            const widthPct = (Math.abs(end - a.tick) / denom) * 100;
            if (a.endTick !== undefined && a.endTick > a.tick) {
              return (
                <div
                  key={`${a.tick}-${a.endTick}-${i}`}
                  className={`pointer-events-auto absolute top-1 h-1.5 cursor-pointer rounded-sm opacity-70 ${markerColor(a.type)}`}
                  style={{ left: `${leftPct}%`, width: `${Math.max(widthPct, 0.8)}%` }}
                  title={a.label}
                  onClick={() => onSeek(a.tick)}
                  onKeyDown={(e) => {
                    if (e.key === "Enter" || e.key === " ") {
                      e.preventDefault();
                      onSeek(a.tick);
                    }
                  }}
                  role="button"
                  tabIndex={0}
                />
              );
            }
            return (
              <button
                key={`${a.tick}-${a.type}-${i}`}
                type="button"
                className={`pointer-events-auto absolute top-0.5 h-2 w-2 -translate-x-1/2 rounded-full ${markerColor(a.type)} ring-1 ring-zinc-950`}
                style={{ left: `${(a.tick / denom) * 100}%` }}
                title={a.label}
                aria-label={a.label}
                onClick={() => onSeek(a.tick)}
              />
            );
          })}
        </div>
      ) : null}
      <input
        className={
          compact
            ? "relative z-10 h-1.5 w-full min-w-[160px] flex-1 cursor-pointer accent-emerald-500"
            : "relative z-10 w-full"
        }
        type="range"
        min={0}
        max={Math.max(0, totalTicks - 1)}
        value={Math.min(cursor, Math.max(0, totalTicks - 1))}
        onChange={(e) => onSeek(Number(e.target.value))}
      />
    </div>
  );

  const tickLabel = (
    <div className={`text-xs text-zinc-400 ${compact ? "shrink-0 whitespace-nowrap" : "mt-1"}`}>
      Tick {totalTicks === 0 ? 0 : cursor + 1} / {totalTicks}
    </div>
  );

  const compactClass =
    variant === "bottomBar"
      ? "flex w-full min-w-0 flex-col gap-2 border-0 bg-transparent px-0 py-1 sm:flex-row sm:flex-wrap sm:items-center"
      : "flex w-full min-w-0 max-w-2xl flex-col gap-2 rounded-lg border border-zinc-600 bg-zinc-900/90 px-3 py-2 lg:max-w-none lg:flex-row lg:flex-wrap lg:items-center";

  if (compact) {
    return (
      <section className={compactClass} aria-label="Replay playback">
        <span className="shrink-0 text-xs font-medium text-zinc-400">Replay</span>
        {buttonRow}
        <div className="flex min-w-0 flex-1 flex-col gap-1 sm:flex-row sm:items-center sm:gap-3">
          {scrubber}
          {tickLabel}
        </div>
      </section>
    );
  }

  return (
    <section className="rounded border border-zinc-700 p-3">
      <h2 className="mb-2 text-sm font-semibold">Transport</h2>
      <div className="mb-2">{buttonRow}</div>
      {scrubber}
      {tickLabel}
    </section>
  );
}
