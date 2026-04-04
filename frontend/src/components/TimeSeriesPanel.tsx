"use client";

import { useCallback, useMemo, type ReactNode } from "react";
import {
  Bar,
  BarChart,
  CartesianGrid,
  Cell,
  Legend,
  Line,
  LineChart,
  ReferenceLine,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from "recharts";

import type { TickState } from "@/lib/types";

export interface TimeSeriesPanelProps {
  ticks: TickState[];
  cursor: number;
  onSeek: (index: number) => void;
}

const SYNC_ID = "replayTimeSeries";

type Row = {
  idx: number;
  tc: number;
  oh: number;
  vs: number;
  orL: number | null;
  orR: number | null;
  orF: number | null;
  orB: number | null;
  opX: number | null;
  opY: number | null;
  opT: number | null;
  reX: number | null;
  reY: number | null;
  reT: number | null;
  cpX: number | null;
  cpY: number | null;
  cpT: number | null;
  accepted: number;
  /** Full-height band for accept/reject strip (always 1). */
  gateBand: number;
  r90: number | null;
  jump: number | null;
  maxResidual: number | null;
  residualThresh: number | null;
  correction: number | null;
  /** √(Δx²+Δy²) between raw estimate and chassis pose — correction that would apply before gate. */
  projectedCorrection: number | null;
  /** Same as projected when gate accepted (for green overlay). */
  projectedWhenAccepted: number | null;
  reason: string;
};

function invalidReading(v: number): number | null {
  return v < 0 ? null : v;
}

function projectedCorrectionIn(t: TickState): number | null {
  const re = t.raw_estimate;
  const cp = t.chassis_pose;
  if (!re || !cp) return null;
  const dx = re.x - cp.x;
  const dy = re.y - cp.y;
  return Math.sqrt(dx * dx + dy * dy);
}

function buildRows(ticks: TickState[]): Row[] {
  return ticks.map((t, idx) => {
    const g = t.gate_decision;
    const or = t.observed_readings;
    const accepted = g?.accepted === true;
    const proj = projectedCorrectionIn(t);
    return {
      idx,
      tc: t.tick,
      oh: t.observed_heading,
      vs: t.valid_sensor_count,
      orL: invalidReading(or[0] ?? -1),
      orR: invalidReading(or[1] ?? -1),
      orF: invalidReading(or[2] ?? -1),
      orB: invalidReading(or[3] ?? -1),
      opX: t.raw_odom?.x ?? null,
      opY: t.raw_odom?.y ?? null,
      opT: t.raw_odom?.theta ?? null,
      reX: t.raw_estimate?.x ?? null,
      reY: t.raw_estimate?.y ?? null,
      reT: t.raw_estimate?.theta ?? null,
      cpX: t.chassis_pose?.x ?? null,
      cpY: t.chassis_pose?.y ?? null,
      cpT: t.chassis_pose?.theta ?? null,
      accepted: g?.accepted === true ? 1 : 0,
      gateBand: 1,
      r90: g?.radius_90_in ?? null,
      jump: g?.jump_in ?? null,
      maxResidual: g?.max_sensor_residual_in ?? null,
      residualThresh: g?.residual_threshold_in ?? null,
      correction: g?.correction_distance_in ?? null,
      projectedCorrection: proj,
      projectedWhenAccepted: accepted && proj != null ? proj : null,
      reason: g?.reason ?? "",
    };
  });
}

function tooltipIdx(state: unknown): number | null {
  if (typeof state !== "object" || state === null) return null;
  const s = state as Record<string, unknown>;
  const raw = s.activeTooltipIndex ?? s.activeIndex;
  if (raw === undefined || raw === null) return null;
  const n = typeof raw === "number" ? raw : Number(raw);
  return Number.isFinite(n) ? n : null;
}

const axisStyle = { fontSize: 10, fill: "#a1a1aa" };
const gridStroke = "#3f3f46";

function ChartBlock({
  title,
  subtitle,
  children,
  chartClassName = "h-[88px] w-full",
}: {
  title: string;
  subtitle?: string;
  children: ReactNode;
  /** Chart area height (default 88px). */
  chartClassName?: string;
}) {
  return (
    <div className="border-b border-zinc-800 pb-2 last:border-b-0">
      <div className="mb-1 flex flex-wrap items-baseline gap-2 px-1">
        <span className="text-[11px] font-semibold uppercase tracking-wide text-zinc-400">{title}</span>
        {subtitle ? <span className="text-[10px] text-zinc-500">{subtitle}</span> : null}
      </div>
      <div className={chartClassName}>{children}</div>
    </div>
  );
}

export function TimeSeriesPanel({ ticks, cursor, onSeek }: TimeSeriesPanelProps) {
  const data = useMemo(() => buildRows(ticks), [ticks]);

  const onChartClick = useCallback(
    (state: unknown) => {
      const i = tooltipIdx(state);
      if (i !== null && i >= 0 && i < ticks.length) onSeek(i);
    },
    [onSeek, ticks.length],
  );

  const cursorTc = data[cursor]?.tc ?? cursor;

  if (ticks.length === 0) {
    return (
      <div className="px-3 py-4 text-center text-xs text-zinc-500">No tick data to chart.</div>
    );
  }

  const commonXAxis = (
    <XAxis
      dataKey="idx"
      type="number"
      domain={["dataMin", "dataMax"]}
      tick={{ ...axisStyle }}
      tickLine={false}
      axisLine={{ stroke: "#52525b" }}
      height={18}
      interval="preserveStartEnd"
      tickFormatter={(v: number) => {
        const row = data[v];
        return row ? String(row.tc) : String(v);
      }}
    />
  );

  const refLine = (
    <ReferenceLine
      x={cursor}
      stroke="#f472b6"
      strokeWidth={1.5}
      strokeDasharray="4 3"
      ifOverflow="extendDomain"
    />
  );

  return (
    <div
      className="max-h-[min(280px,calc(100vh-220px))] overflow-y-auto bg-zinc-950/95 px-1 pb-2 pt-1"
      role="region"
      aria-label="Replay time series"
    >
      <p className="mb-2 px-2 text-[10px] leading-snug text-zinc-500">
        <span className="font-medium text-zinc-400">tc</span> — time axis: horizontal position is replay index; axis ticks show sequential{" "}
        <span className="font-mono text-zinc-400">tc</span> from the log (may skip, e.g. 2, 4, 6…).
      </p>

      <ChartBlock title="g · accept" subtitle="Green = accepted · Red = rejected">
        <ResponsiveContainer width="100%" height="100%">
          <BarChart
            data={data}
            margin={{ top: 4, right: 8, left: 0, bottom: 0 }}
            syncId={SYNC_ID}
            onClick={onChartClick}
          >
            <CartesianGrid strokeDasharray="3 3" stroke={gridStroke} />
            {commonXAxis}
            <YAxis type="number" domain={[0, 1]} hide width={0} />
            <Tooltip
              contentStyle={{ background: "#18181b", border: "1px solid #3f3f46", fontSize: 11 }}
              formatter={(_v, _n, item) => {
                const p = item?.payload as Row | undefined;
                if (!p) return ["", ""];
                return [p.accepted ? "accepted" : "rejected", "gate"];
              }}
              labelFormatter={(_, payload) => {
                const pl = Array.isArray(payload) ? payload[0]?.payload : (payload as { payload?: Row } | undefined)?.payload;
                const p = pl as Row | undefined;
                if (!p) return "";
                return `idx ${p.idx} · tc=${p.tc}${p.reason ? ` · ${p.reason}` : ""}`;
              }}
            />
            {refLine}
            <Bar dataKey="gateBand" maxBarSize={6} radius={[0, 0, 0, 0]} isAnimationActive={false}>
              {data.map((entry, i) => (
                <Cell
                  key={`acc-${entry.idx}-${i}`}
                  fill={entry.accepted ? "rgba(34,197,94,0.45)" : "rgba(239,68,68,0.45)"}
                />
              ))}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
      </ChartBlock>

      <ChartBlock
        title="g · cd"
        subtitle="Projected correction = ‖re − cp‖ (in). Green = gate accepted (correction applied). Dashed = reported cd from log."
        chartClassName="h-[104px] w-full"
      >
        <ResponsiveContainer width="100%" height="100%">
          <LineChart
            data={data}
            margin={{ top: 2, right: 10, left: 2, bottom: 0 }}
            syncId={SYNC_ID}
            onClick={onChartClick}
          >
            <CartesianGrid strokeDasharray="3 3" stroke={gridStroke} />
            {commonXAxis}
            <YAxis width={42} tick={{ ...axisStyle }} tickLine={false} axisLine={{ stroke: "#52525b" }} />
            <Legend
              wrapperStyle={{ fontSize: 9, paddingTop: 0 }}
              formatter={(value) => <span className="text-zinc-400">{value}</span>}
            />
            <Tooltip
              contentStyle={{ background: "#18181b", border: "1px solid #3f3f46", fontSize: 11 }}
              formatter={(value, name) => {
                const n = Number(value);
                const text = Number.isFinite(n) ? `${n.toFixed(2)} in` : "—";
                return [text, String(name)];
              }}
              labelFormatter={(_, payload) => {
                const p = payload?.[0]?.payload as Row | undefined;
                if (!p) return "";
                const verdict = p.accepted ? "accepted" : "rejected";
                return `idx ${p.idx} · tc=${p.tc} · ${verdict}${p.reason ? ` · ${p.reason}` : ""}`;
              }}
            />
            {refLine}
            <Line
              type="monotone"
              dataKey="projectedCorrection"
              name="projected"
              stroke="#94a3b8"
              strokeOpacity={0.95}
              dot={false}
              strokeWidth={1.5}
              connectNulls
              isAnimationActive={false}
            />
            <Line
              type="monotone"
              dataKey="projectedWhenAccepted"
              name="accepted"
              stroke="#22c55e"
              strokeOpacity={1}
              dot={{ r: 3, fill: "#22c55e", stroke: "#14532d", strokeWidth: 1 }}
              strokeWidth={2.75}
              connectNulls={false}
              isAnimationActive={false}
            />
            <Line
              type="monotone"
              dataKey="correction"
              name="reported cd"
              stroke="#64748b"
              strokeDasharray="4 3"
              dot={false}
              strokeWidth={1}
              connectNulls
              isAnimationActive={false}
            />
          </LineChart>
        </ResponsiveContainer>
      </ChartBlock>

      <ChartBlock title="g · metrics" subtitle="r90, jump, max residual, residual threshold (in)">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart
            data={data}
            margin={{ top: 4, right: 8, left: 0, bottom: 0 }}
            syncId={SYNC_ID}
            onClick={onChartClick}
          >
            <CartesianGrid strokeDasharray="3 3" stroke={gridStroke} />
            {commonXAxis}
            <YAxis width={40} tick={{ ...axisStyle }} tickLine={false} axisLine={{ stroke: "#52525b" }} />
            <Legend wrapperStyle={{ fontSize: 9 }} />
            <Tooltip
              contentStyle={{ background: "#18181b", border: "1px solid #3f3f46", fontSize: 11 }}
              formatter={(value, name) => {
                const n = Number(value);
                const text = Number.isFinite(n) ? n.toFixed(2) : value != null ? String(value) : "—";
                return [text, String(name)];
              }}
              labelFormatter={(_, payload) => {
                const p = payload?.[0]?.payload as Row | undefined;
                if (!p) return "";
                return `idx ${p.idx} · tc=${p.tc}${p.reason ? ` · ${p.reason}` : ""}`;
              }}
            />
            {refLine}
            <Line type="monotone" dataKey="r90" name="r90" stroke="#a78bfa" dot={false} strokeWidth={1.1} connectNulls isAnimationActive={false} />
            <Line type="monotone" dataKey="jump" name="jump" stroke="#fbbf24" dot={false} strokeWidth={1.1} connectNulls isAnimationActive={false} />
            <Line type="monotone" dataKey="maxResidual" name="max resid" stroke="#f87171" dot={false} strokeWidth={1.1} connectNulls isAnimationActive={false} />
            <Line
              type="monotone"
              dataKey="residualThresh"
              name="resid thresh"
              stroke="#fb923c"
              strokeDasharray="4 2"
              dot={false}
              strokeWidth={1}
              connectNulls
              isAnimationActive={false}
            />
          </LineChart>
        </ResponsiveContainer>
      </ChartBlock>

      <ChartBlock title="oh" subtitle="Observed heading (IMU °)">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart
            data={data}
            margin={{ top: 4, right: 8, left: 0, bottom: 0 }}
            syncId={SYNC_ID}
            onClick={onChartClick}
          >
            <CartesianGrid strokeDasharray="3 3" stroke={gridStroke} />
            {commonXAxis}
            <YAxis domain={[0, 360]} width={36} tick={{ ...axisStyle }} tickLine={false} axisLine={{ stroke: "#52525b" }} />
            <Tooltip
              contentStyle={{ background: "#18181b", border: "1px solid #3f3f46", fontSize: 11 }}
              formatter={(value) => {
                const n = Number(value);
                return [Number.isFinite(n) ? `${n.toFixed(1)}°` : "—", "oh"];
              }}
              labelFormatter={(_, payload) => {
                const p = payload?.[0]?.payload as Row | undefined;
                return p ? `idx ${p.idx} · tc=${p.tc}` : "";
              }}
            />
            {refLine}
            <Line type="monotone" dataKey="oh" stroke="#22d3ee" dot={false} strokeWidth={1.2} isAnimationActive={false} />
          </LineChart>
        </ResponsiveContainer>
      </ChartBlock>

      <ChartBlock title="vs" subtitle="Valid sensors (0–4)">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart
            data={data}
            margin={{ top: 4, right: 8, left: 0, bottom: 0 }}
            syncId={SYNC_ID}
            onClick={onChartClick}
          >
            <CartesianGrid strokeDasharray="3 3" stroke={gridStroke} />
            {commonXAxis}
            <YAxis domain={[0, 4]} width={28} tick={{ ...axisStyle }} tickLine={false} axisLine={{ stroke: "#52525b" }} />
            <Tooltip
              contentStyle={{ background: "#18181b", border: "1px solid #3f3f46", fontSize: 11 }}
              formatter={(value) => [value != null ? String(value) : "—", "vs"]}
              labelFormatter={(_, payload) => {
                const p = payload?.[0]?.payload as Row | undefined;
                return p ? `idx ${p.idx} · tc=${p.tc}` : "";
              }}
            />
            {refLine}
            <Line
              type="stepAfter"
              dataKey="vs"
              stroke="#fbbf24"
              dot={false}
              strokeWidth={1.2}
              isAnimationActive={false}
            />
          </LineChart>
        </ResponsiveContainer>
      </ChartBlock>

      <ChartBlock title="or" subtitle="Observed readings [L,R,F,B] (in) · gap = invalid">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart
            data={data}
            margin={{ top: 4, right: 8, left: 0, bottom: 0 }}
            syncId={SYNC_ID}
            onClick={onChartClick}
          >
            <CartesianGrid strokeDasharray="3 3" stroke={gridStroke} />
            {commonXAxis}
            <YAxis width={36} tick={{ ...axisStyle }} tickLine={false} axisLine={{ stroke: "#52525b" }} />
            <Legend wrapperStyle={{ fontSize: 10 }} />
            <Tooltip
              contentStyle={{ background: "#18181b", border: "1px solid #3f3f46", fontSize: 11 }}
              labelFormatter={(_, payload) => {
                const p = payload?.[0]?.payload as Row | undefined;
                return p ? `idx ${p.idx} · tc=${p.tc}` : "";
              }}
            />
            {refLine}
            <Line type="monotone" dataKey="orL" name="L" stroke="#34d399" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
            <Line type="monotone" dataKey="orR" name="R" stroke="#60a5fa" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
            <Line type="monotone" dataKey="orF" name="F" stroke="#f472b6" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
            <Line type="monotone" dataKey="orB" name="B" stroke="#c084fc" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
          </LineChart>
        </ResponsiveContainer>
      </ChartBlock>

      <ChartBlock title="op / re / cp · x" subtitle="Odom · MCL · Chassis (in)">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart
            data={data}
            margin={{ top: 4, right: 8, left: 0, bottom: 0 }}
            syncId={SYNC_ID}
            onClick={onChartClick}
          >
            <CartesianGrid strokeDasharray="3 3" stroke={gridStroke} />
            {commonXAxis}
            <YAxis width={36} tick={{ ...axisStyle }} tickLine={false} axisLine={{ stroke: "#52525b" }} />
            <Legend wrapperStyle={{ fontSize: 10 }} />
            <Tooltip
              contentStyle={{ background: "#18181b", border: "1px solid #3f3f46", fontSize: 11 }}
              labelFormatter={(_, payload) => {
                const p = payload?.[0]?.payload as Row | undefined;
                return p ? `idx ${p.idx} · tc=${p.tc}` : "";
              }}
            />
            {refLine}
            <Line type="monotone" dataKey="opX" name="op.x" stroke="#34d399" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
            <Line type="monotone" dataKey="reX" name="re.x" stroke="#60a5fa" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
            <Line type="monotone" dataKey="cpX" name="cp.x" stroke="#22d3ee" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
          </LineChart>
        </ResponsiveContainer>
      </ChartBlock>

      <ChartBlock title="op / re / cp · y" subtitle="Odom · MCL · Chassis (in)">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart
            data={data}
            margin={{ top: 4, right: 8, left: 0, bottom: 0 }}
            syncId={SYNC_ID}
            onClick={onChartClick}
          >
            <CartesianGrid strokeDasharray="3 3" stroke={gridStroke} />
            {commonXAxis}
            <YAxis width={36} tick={{ ...axisStyle }} tickLine={false} axisLine={{ stroke: "#52525b" }} />
            <Legend wrapperStyle={{ fontSize: 10 }} />
            <Tooltip
              contentStyle={{ background: "#18181b", border: "1px solid #3f3f46", fontSize: 11 }}
              labelFormatter={(_, payload) => {
                const p = payload?.[0]?.payload as Row | undefined;
                return p ? `idx ${p.idx} · tc=${p.tc}` : "";
              }}
            />
            {refLine}
            <Line type="monotone" dataKey="opY" name="op.y" stroke="#34d399" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
            <Line type="monotone" dataKey="reY" name="re.y" stroke="#60a5fa" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
            <Line type="monotone" dataKey="cpY" name="cp.y" stroke="#22d3ee" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
          </LineChart>
        </ResponsiveContainer>
      </ChartBlock>

      <ChartBlock title="op / re / cp · θ" subtitle="Odom · MCL · Chassis (°)">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart
            data={data}
            margin={{ top: 4, right: 8, left: 0, bottom: 0 }}
            syncId={SYNC_ID}
            onClick={onChartClick}
          >
            <CartesianGrid strokeDasharray="3 3" stroke={gridStroke} />
            {commonXAxis}
            <YAxis width={36} tick={{ ...axisStyle }} tickLine={false} axisLine={{ stroke: "#52525b" }} />
            <Legend wrapperStyle={{ fontSize: 10 }} />
            <Tooltip
              contentStyle={{ background: "#18181b", border: "1px solid #3f3f46", fontSize: 11 }}
              labelFormatter={(_, payload) => {
                const p = payload?.[0]?.payload as Row | undefined;
                return p ? `idx ${p.idx} · tc=${p.tc}` : "";
              }}
            />
            {refLine}
            <Line type="monotone" dataKey="opT" name="op.θ" stroke="#34d399" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
            <Line type="monotone" dataKey="reT" name="re.θ" stroke="#60a5fa" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
            <Line type="monotone" dataKey="cpT" name="cp.θ" stroke="#22d3ee" dot={false} strokeWidth={1} connectNulls isAnimationActive={false} />
          </LineChart>
        </ResponsiveContainer>
      </ChartBlock>

      <p className="px-2 pt-1 text-[10px] text-zinc-500">
        Cursor: idx {cursor} · tc {cursorTc}. Click a chart to seek. Pink line = current tick.
      </p>
    </div>
  );
}
