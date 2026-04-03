"use client";

import { useId } from "react";

import type { OverlayFlags, TickState } from "@/lib/types";

interface Props {
  flags: OverlayFlags;
  availability: Partial<Record<keyof OverlayFlags, boolean>>;
  onToggle: (key: keyof OverlayFlags, value: boolean) => void;
  tick: TickState | null;
}

const LABELS: Record<keyof OverlayFlags, string> = {
  robotTruth: "Robot True Location",
  odomPose: "Odom Pose",
  mclEstimate: "MCL Raw Estimate",
  acceptedEstimate: "MCL Accepted Estimate",
  r90Circle: "R90 Circle",
  particles: "Particles",
  heatmap: "Heatmap",
  sensorReadings: "Sensor Readings",
  sensorResiduals: "Sensor Residuals",
  diffMclPose: "MCL vs Pose",
  diffMclTruth: "MCL vs Truth",
  diffPoseTruth: "Pose vs Truth",
};

function HeatmapLegendIcon() {
  const uid = useId().replace(/:/g, "_");
  const gid = `legend-hm-${uid}`;
  return (
    <svg className="shrink-0 overflow-visible" width="36" height="18" viewBox="0 0 36 18" aria-hidden>
      <defs>
        <linearGradient id={gid} x1="0" y1="0" x2="1" y2="1">
          <stop offset="0%" stopColor="#14b8a6" stopOpacity="0.85" />
          <stop offset="50%" stopColor="#6366f1" stopOpacity="0.65" />
          <stop offset="100%" stopColor="#f59e0b" stopOpacity="0.55" />
        </linearGradient>
      </defs>
      <rect x="4" y="3" width="28" height="12" rx="2" fill={`url(#${gid})`} opacity="0.9" />
    </svg>
  );
}

/** Mini preview matching `FieldCanvas` stroke/fill styles. */
function OverlayLegendIcon({ overlayKey }: { overlayKey: keyof OverlayFlags }) {
  const common = "shrink-0 overflow-visible";
  switch (overlayKey) {
    case "robotTruth":
      return (
        <svg className={common} width="36" height="18" viewBox="0 0 36 18" aria-hidden>
          <polygon points="18,1 7,17 29,17" fill="#22c55e" />
        </svg>
      );
    case "odomPose":
      return (
        <svg className={common} width="36" height="18" viewBox="0 0 36 18" aria-hidden>
          <circle cx="18" cy="9" r="5" fill="none" stroke="#10b981" strokeWidth="2" />
        </svg>
      );
    case "mclEstimate":
      return (
        <svg className={common} width="36" height="18" viewBox="0 0 36 18" aria-hidden>
          <circle cx="18" cy="9" r="5" fill="none" stroke="#f97316" strokeWidth="2" />
        </svg>
      );
    case "acceptedEstimate":
      return (
        <svg className={common} width="36" height="18" viewBox="0 0 36 18" aria-hidden>
          <circle cx="18" cy="9" r="4" fill="#3b82f6" />
        </svg>
      );
    case "r90Circle":
      return (
        <svg className={common} width="36" height="18" viewBox="0 0 36 18" aria-hidden>
          <circle cx="18" cy="9" r="7" fill="none" stroke="rgba(59,130,246,0.65)" strokeWidth="1.5" />
        </svg>
      );
    case "particles":
      return (
        <svg className={common} width="36" height="18" viewBox="0 0 36 18" aria-hidden>
          <circle cx="10" cy="6" r="1.8" fill="#94a3b8" />
          <circle cx="22" cy="11" r="1.5" fill="#64748b" />
          <circle cx="16" cy="13" r="1.2" fill="#a1a1aa" />
          <circle cx="26" cy="5" r="1.4" fill="#71717a" />
        </svg>
      );
    case "heatmap":
      return <HeatmapLegendIcon />;
    case "sensorReadings":
      return (
        <svg className={common} width="36" height="18" viewBox="0 0 36 18" aria-hidden>
          <line
            x1="6"
            y1="14"
            x2="28"
            y2="4"
            stroke="#f59e0b"
            strokeWidth="1.8"
            strokeDasharray="4 3"
          />
          <circle cx="28" cy="4" r="2" fill="none" stroke="#f59e0b" strokeWidth="1.2" />
        </svg>
      );
    case "sensorResiduals":
      return (
        <svg className={common} width="36" height="18" viewBox="0 0 36 18" aria-hidden>
          <path
            d="M6 12 L10 6 L14 11 L18 5 L22 10 L26 7 L30 12"
            fill="none"
            stroke="#a855f7"
            strokeWidth="1.5"
            strokeLinecap="round"
          />
        </svg>
      );
    case "diffMclPose":
      return (
        <svg className={common} width="36" height="18" viewBox="0 0 36 18" aria-hidden>
          <line
            x1="8"
            y1="12"
            x2="26"
            y2="6"
            stroke="rgba(229,231,235,0.95)"
            strokeWidth="1.8"
            strokeDasharray="4 3"
          />
        </svg>
      );
    case "diffMclTruth":
      return (
        <svg className={common} width="36" height="18" viewBox="0 0 36 18" aria-hidden>
          <line
            x1="8"
            y1="12"
            x2="26"
            y2="6"
            stroke="rgba(244,114,182,0.95)"
            strokeWidth="1.8"
            strokeDasharray="3 3"
          />
        </svg>
      );
    case "diffPoseTruth":
      return (
        <svg className={common} width="36" height="18" viewBox="0 0 36 18" aria-hidden>
          <line
            x1="8"
            y1="12"
            x2="26"
            y2="6"
            stroke="rgba(34,197,94,0.95)"
            strokeWidth="1.8"
            strokeDasharray="3 5"
          />
        </svg>
      );
    default:
      return <span className="inline-block h-[18px] w-9" aria-hidden />;
  }
}

export function Legend({ flags, availability, onToggle, tick }: Props) {
  return (
    <section className="rounded border border-zinc-700 p-3">
      <h2 className="mb-2 text-sm font-semibold">Legend</h2>
      <div className="space-y-1.5 text-xs">
        {(Object.keys(flags) as Array<keyof OverlayFlags>).map((key) => {
          const enabled = availability[key] ?? true;
          return (
            <label
              key={key}
              className={`flex cursor-pointer items-center gap-2 ${enabled ? "text-zinc-200" : "text-zinc-500"}`}
            >
              <input
                type="checkbox"
                checked={flags[key]}
                disabled={!enabled}
                onChange={(e) => onToggle(key, e.target.checked)}
                className="shrink-0"
              />
              <span
                className={`flex shrink-0 items-center justify-center rounded border border-zinc-800 bg-zinc-950/80 px-1 py-0.5 ${!enabled ? "opacity-40" : ""}`}
                aria-hidden
              >
                <OverlayLegendIcon overlayKey={key} />
              </span>
              <span className="min-w-0 flex-1">{LABELS[key]}</span>
            </label>
          );
        })}
      </div>
      <div className="mt-3 border-t border-zinc-800 pt-2 text-xs text-zinc-300">
        <div>MCL error: {tick ? tick.mcl_error.toFixed(2) : "-"} in</div>
        <div>Odom error: {tick ? tick.odom_error.toFixed(2) : "-"} in</div>
        <div>R90: {tick?.post_resample?.radius_90?.toFixed(2) ?? "-"} in</div>
      </div>
    </section>
  );
}
