"use client";

import Link from "next/link";
import { useEffect, useState } from "react";

import { FailureOverlay } from "@/components/FailureOverlay";
import { FailureTimeline } from "@/components/FailureTimeline";
import { FieldCanvas } from "@/components/FieldCanvas";
import { MetricsPanel } from "@/components/MetricsPanel";
import { StageStepper, type Stage } from "@/components/StageStepper";
import { useMCLReplay } from "@/hooks/useMCLReplay";

export default function MCLReplayPage() {
  const replay = useMCLReplay();
  const [stage, setStage] = useState<Stage>("post_resample");
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

      <div className="grid flex-1 grid-cols-1 gap-4 lg:grid-cols-[1fr_320px]">
        <FieldCanvas
          tick={replay.currentTick}
          stage={stage}
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
                onClick={() => replay.setCursor(Math.max(0, replay.cursor - 1))}
                disabled={!hasData}
              >
                Prev
              </button>
              <button
                type="button"
                className="rounded bg-zinc-700 px-2 py-1"
                onClick={() => replay.setCursor(Math.min(rangeMax, replay.cursor + 1))}
                disabled={!hasData}
              >
                Next
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
              value={Math.min(replay.cursor, rangeMax)}
              onChange={(e) => replay.setCursor(Number(e.target.value))}
              disabled={!hasData}
            />
            <div className="mt-1 text-xs text-zinc-400">
              Tick {Math.min(replay.cursor, rangeMax)} / {rangeMax}
            </div>
            <div className="mt-2">
              <FailureTimeline ticks={[]} cursor={Math.min(replay.cursor, rangeMax)} onJump={replay.setCursor} />
            </div>
          </div>
          <StageStepper stage={stage} onStageChange={setStage} />
          <MetricsPanel tick={replay.currentTick} />
          <FailureOverlay tick={replay.currentTick} />
        </aside>
      </div>
    </main>
  );
}
