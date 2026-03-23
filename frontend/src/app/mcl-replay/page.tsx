"use client";

import Link from "next/link";
import { useEffect, useState } from "react";

import { FailureOverlay } from "@/components/FailureOverlay";
import { FailureTimeline } from "@/components/FailureTimeline";
import { FieldCanvas } from "@/components/FieldCanvas";
import { MetricsPanel } from "@/components/MetricsPanel";
import { CanvasLegend } from "@/components/CanvasLegend";
import { useMCLReplay } from "@/hooks/useMCLReplay";

const TICK_MS = 50;
const SECOND_MS = 1000;
const TICKS_PER_SECOND = SECOND_MS / TICK_MS;

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function tickCountToCursor(tickCount: number, rangeMax: number): number {
  return clamp(Math.max(1, Math.round(tickCount)) - 1, 0, rangeMax);
}

function msToCursor(ms: number, rangeMax: number): number {
  const snappedMs = Math.round(ms / TICK_MS) * TICK_MS;
  const tickCount = Math.max(1, Math.round(snappedMs / TICK_MS));
  return tickCountToCursor(tickCount, rangeMax);
}

function formatTimestamp(ms: number): string {
  return `${(ms / 1000).toFixed(2)}s (${ms}ms)`;
}

export default function MCLReplayPage() {
  const replay = useMCLReplay();
  const [isDragOver, setIsDragOver] = useState(false);

  useEffect(() => {
    if (replay.usingDropped) return;
    const requested = new URLSearchParams(window.location.search).get("file");
    if (!requested) return;
    if (!replay.files.includes(requested)) return;
    if (replay.selected === requested) return;
    replay.setSelected(requested);
  }, [replay]);

  const title = replay.usingDropped
    ? `Dropped: ${replay.droppedFileName ?? "local file"}`
    : replay.selected || "No replay selected";

  const hasData = replay.totalTicks > 0;

  const rangeMax = Math.max(0, replay.totalTicks - 1);
  const currentCursor = Math.min(replay.cursor, rangeMax);
  const currentTickCount = hasData ? (replay.currentTick?.tick_count ?? currentCursor + 1) : 0;
  const maxTickCount = hasData ? rangeMax + 1 : 0;
  const currentTimeMs = currentTickCount * TICK_MS;
  const maxTimeMs = Math.max(TICK_MS, maxTickCount * TICK_MS);
  const currentTimeSec = Math.floor(currentTimeMs / SECOND_MS);

  return (
    <main className="flex min-h-screen flex-col bg-zinc-950 p-4 text-zinc-100">
      <div className="mb-3 flex items-center justify-between">
        <h1 className="text-xl font-semibold">MCL Replay Viewer</h1>
        <Link className="rounded bg-zinc-800 px-3 py-1 text-sm" href="/">
          Back to Live
        </Link>
      </div>

      <div className="mb-3 flex flex-wrap items-center gap-2">
        <label className="text-sm">Replay:</label>
        <select
          className="rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
          value={replay.selected}
          onChange={(e) => replay.setSelected(e.target.value)}
          disabled={replay.usingDropped}
        >
          {replay.files.map((file) => (
            <option key={file} value={file}>
              {file}
            </option>
          ))}
        </select>
        <label
          className={`cursor-pointer rounded border px-2 py-1 text-xs ${
            isDragOver ? "border-cyan-400 bg-cyan-900/30" : "border-zinc-700 bg-zinc-900"
          }`}
          onDragOver={(event) => {
            setIsDragOver(true);
            replay.onDragOver(event);
          }}
          onDragLeave={() => setIsDragOver(false)}
          onDrop={(event) => {
            setIsDragOver(false);
            void replay.onDrop(event);
          }}
        >
          Drop JSON/TXT or click to upload
          <input
            className="hidden"
            type="file"
            accept=".json,.txt,application/json,text/plain"
            onChange={replay.onFileInput}
          />
        </label>
        {replay.usingDropped ? (
          <button
            type="button"
            className="rounded bg-zinc-700 px-2 py-1 text-xs"
            onClick={replay.clearDropped}
          >
            Clear dropped file
          </button>
        ) : null}
        <span className="text-xs text-zinc-400">{title}</span>
      </div>

      {replay.error ? (
        <div className="mb-3 rounded border border-red-700 bg-red-950/40 p-2 text-sm text-red-200">
          {replay.error}
        </div>
      ) : null}

      <div className="grid flex-1 grid-cols-1 gap-4 lg:grid-cols-[280px_1fr_320px]">
        <aside className="space-y-3">
          <CanvasLegend />
          <MetricsPanel tick={replay.currentTick} />
        </aside>
        <FieldCanvas
          tick={replay.currentTick}
          stage="post_resample"
          obstacles={replay.replayObstacles}
          fieldHalf={replay.fieldHalf}
          prevGroundTruth={null}
          className="h-[72vh] w-full rounded border border-zinc-700"
        />
        <aside className="space-y-3">
          <div className="rounded border border-zinc-700 p-3">
            <div className="mb-2 flex items-center gap-2">
              <button
                type="button"
                className="rounded bg-zinc-700 px-2 py-1"
                onClick={() => replay.setPlaying((v) => !v)}
                disabled={!hasData}
              >
                {replay.playing ? "Pause" : "Play"}
              </button>
              <button
                type="button"
                className="rounded bg-zinc-700 px-2 py-1"
                onClick={() => replay.setCursor(Math.max(0, currentCursor - 1))}
                disabled={!hasData}
              >
                -50ms
              </button>
              <button
                type="button"
                className="rounded bg-zinc-700 px-2 py-1"
                onClick={() => replay.setCursor(Math.min(rangeMax, currentCursor + 1))}
                disabled={!hasData}
              >
                +50ms
              </button>
              <button
                type="button"
                className="rounded bg-zinc-700 px-2 py-1"
                onClick={() => replay.setCursor(Math.max(0, currentCursor - TICKS_PER_SECOND))}
                disabled={!hasData}
              >
                -1s
              </button>
              <button
                type="button"
                className="rounded bg-zinc-700 px-2 py-1"
                onClick={() => replay.setCursor(Math.min(rangeMax, currentCursor + TICKS_PER_SECOND))}
                disabled={!hasData}
              >
                +1s
              </button>
              <select
                className="rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
                value={String(replay.speed)}
                onChange={(e) => replay.setSpeed(Number(e.target.value))}
              >
                {[0.5, 1, 2, 4].map((s) => (
                  <option key={s} value={s}>
                    {s}x
                  </option>
                ))}
              </select>
            </div>
            <input
              className="w-full"
              type="range"
              min={0}
              max={rangeMax}
              value={currentCursor}
              onChange={(e) => replay.setCursor(Number(e.target.value))}
              disabled={!hasData}
            />
            <div className="mt-1 text-xs text-zinc-400">
              {hasData ? `Tick ${currentTickCount} / ${maxTickCount} (${formatTimestamp(currentTimeMs)})` : "No data loaded"}
            </div>
            <div className="mt-2 grid grid-cols-2 gap-2 text-xs">
              <label className="flex flex-col gap-1">
                <span className="text-zinc-400">Time (ms, step 50)</span>
                <input
                  className="rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
                  type="number"
                  min={TICK_MS}
                  max={maxTimeMs}
                  step={TICK_MS}
                  value={currentTimeMs}
                  onChange={(e) => {
                    const requested = Number(e.target.value);
                    if (Number.isNaN(requested)) return;
                    replay.setCursor(msToCursor(requested, rangeMax));
                  }}
                  disabled={!hasData}
                />
              </label>
              <label className="flex flex-col gap-1">
                <span className="text-zinc-400">Time (sec, step 1)</span>
                <input
                  className="rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
                  type="number"
                  min={0}
                  max={Math.floor(maxTimeMs / SECOND_MS)}
                  step={1}
                  value={currentTimeSec}
                  onChange={(e) => {
                    const requestedSec = Number(e.target.value);
                    if (Number.isNaN(requestedSec)) return;
                    replay.setCursor(msToCursor(requestedSec * SECOND_MS, rangeMax));
                  }}
                  disabled={!hasData}
                />
              </label>
            </div>
            <div className="mt-2">
              <FailureTimeline ticks={[]} cursor={currentCursor} onJump={replay.setCursor} />
            </div>
          </div>
          <FailureOverlay tick={replay.currentTick} />
        </aside>
      </div>
    </main>
  );
}
