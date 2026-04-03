"use client";

import type { FailureConfig as FailureConfigType } from "@/lib/types";

interface Props {
  value: FailureConfigType;
  onChange: (next: FailureConfigType) => void;
  disabled?: boolean;
}

function clamp(n: number, lo: number, hi: number) {
  return Math.min(hi, Math.max(lo, n));
}

function pctFromProb(p: number) {
  return Math.round(p * 1000) / 10;
}

function probFromPct(pct: number) {
  return clamp(pct / 100, 0, 1);
}

type FailureRow = {
  id: string;
  label: string;
  hint: string;
  freqKey: keyof FailureConfigType;
  maxKey?: keyof FailureConfigType;
  maxTicksMin: number;
  maxTicksMax: number;
  maxReadOnly?: string;
};

const ROWS: FailureRow[] = [
  {
    id: "sensor_dead",
    label: "Sensor dead",
    hint: "Ray returns no hit for a stretch.",
    freqKey: "sensor_dead_prob",
    maxKey: "sensor_dead_max_duration_ticks",
    maxTicksMin: 1,
    maxTicksMax: 120,
  },
  {
    id: "sensor_stuck",
    label: "Sensor stuck",
    hint: "Reading frozen at last value.",
    freqKey: "sensor_stuck_prob",
    maxKey: "sensor_stuck_max_duration_ticks",
    maxTicksMin: 1,
    maxTicksMax: 120,
  },
  {
    id: "odom_spike",
    label: "Odometry spike",
    hint: "Sudden scale error on wheel odometry.",
    freqKey: "odom_spike_prob",
    maxKey: "odom_spike_max_duration_ticks",
    maxTicksMin: 1,
    maxTicksMax: 80,
  },
  {
    id: "heading_bias",
    label: "Heading bias",
    hint: "Persistent gyro-style offset.",
    freqKey: "heading_bias_prob",
    maxKey: "heading_bias_max_duration_ticks",
    maxTicksMin: 1,
    maxTicksMax: 120,
  },
  {
    id: "spurious_reflection",
    label: "Spurious reflection",
    hint: "False short range on a ray.",
    freqKey: "spurious_reflection_prob",
    maxKey: "spurious_reflection_max_duration_ticks",
    maxTicksMin: 1,
    maxTicksMax: 20,
  },
  {
    id: "kidnap",
    label: "Kidnap",
    hint: "Teleport pose to a random field location.",
    freqKey: "kidnap_prob",
    maxReadOnly: "1 (instant)",
    maxTicksMin: 1,
    maxTicksMax: 1,
  },
];

export function FailureConfig({ value, onChange, disabled = false }: Props) {
  const patch = (partial: Partial<FailureConfigType>) => {
    onChange({ ...value, ...partial });
  };

  return (
    <section className="rounded border border-zinc-700 p-3">
      <h2 className="mb-1 text-sm font-semibold">Failure configuration</h2>
      <p className="mb-3 text-xs text-zinc-400">
        Each row sets how often a failure may start (per tick, expected frequency) and how long it can last.
        Values are applied when you start a live or route session.
      </p>

      <div className="mb-4 rounded border border-zinc-800 bg-zinc-900/50 p-3">
        <label className="text-xs text-zinc-300">
          Minimum duration (ticks)
          <input
            aria-label="Minimum duration ticks"
            className="mt-1 w-full max-w-[12rem] rounded border border-zinc-600 bg-zinc-900 px-2 py-1 text-sm"
            type="number"
            min={1}
            max={200}
            disabled={disabled}
            value={value.min_duration_ticks}
            onChange={(e) =>
              patch({
                min_duration_ticks: clamp(Number(e.target.value) || 1, 1, 200),
              })
            }
          />
          <span className="mt-1 block text-[11px] text-zinc-500">
            Lower bound for sensor, odometry, and heading failure spans when max exceeds min.
          </span>
        </label>
      </div>

      <div className="overflow-x-auto">
        <table className="w-full min-w-[28rem] border-collapse text-left text-xs">
          <thead>
            <tr className="border-b border-zinc-700 text-zinc-400">
              <th className="py-2 pr-2 font-medium">Failure</th>
              <th className="py-2 pr-2 font-medium">Frequency (% / tick)</th>
              <th className="py-2 font-medium">Max ticks</th>
            </tr>
          </thead>
          <tbody>
            {ROWS.map((row) => {
              const freq = value[row.freqKey];
              const pct = typeof freq === "number" ? pctFromProb(freq) : 0;
              const maxVal =
                row.maxKey && typeof value[row.maxKey] === "number"
                  ? (value[row.maxKey] as number)
                  : 0;

              return (
                <tr key={row.id} className="border-b border-zinc-800/80">
                  <td className="py-2 pr-3 align-top">
                    <div className="font-medium text-zinc-200">{row.label}</div>
                    <div className="mt-0.5 text-[11px] text-zinc-500">{row.hint}</div>
                  </td>
                  <td className="py-2 pr-3 align-top">
                    <div className="flex items-center gap-2">
                      <input
                        aria-label={`${row.label} frequency percent`}
                        className="w-20 rounded border border-zinc-600 bg-zinc-900 px-2 py-1 text-sm tabular-nums"
                        type="number"
                        min={0}
                        max={100}
                        step={0.1}
                        disabled={disabled}
                        value={pct}
                        onChange={(e) => {
                          const nextPct = Number(e.target.value);
                          if (Number.isNaN(nextPct)) return;
                          patch({ [row.freqKey]: probFromPct(nextPct) } as Partial<FailureConfigType>);
                        }}
                      />
                      <span className="text-zinc-500">%</span>
                    </div>
                    <input
                      aria-label={`${row.label} frequency slider`}
                      className="mt-2 h-1.5 w-full max-w-[14rem] cursor-pointer accent-emerald-500 disabled:opacity-50"
                      type="range"
                      min={0}
                      max={100}
                      step={0.5}
                      disabled={disabled}
                      value={clamp(pct, 0, 100)}
                      onChange={(e) =>
                        patch({
                          [row.freqKey]: probFromPct(Number(e.target.value)),
                        } as Partial<FailureConfigType>)
                      }
                    />
                  </td>
                  <td className="py-2 align-top">
                    {row.maxReadOnly ? (
                      <span className="text-zinc-500">{row.maxReadOnly}</span>
                    ) : row.maxKey ? (
                      <input
                        aria-label={`${row.label} max ticks`}
                        className="w-20 rounded border border-zinc-600 bg-zinc-900 px-2 py-1 text-sm tabular-nums"
                        type="number"
                        min={row.maxTicksMin}
                        max={row.maxTicksMax}
                        disabled={disabled}
                        value={maxVal}
                        onChange={(e) => {
                          const n = Number(e.target.value);
                          if (Number.isNaN(n)) return;
                          const key = row.maxKey as keyof FailureConfigType;
                          patch({
                            [key]: clamp(
                              Math.floor(n),
                              row.maxTicksMin,
                              row.maxTicksMax,
                            ),
                          } as Partial<FailureConfigType>);
                        }}
                      />
                    ) : null}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </section>
  );
}
