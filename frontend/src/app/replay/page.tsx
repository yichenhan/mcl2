"use client";

import Link from "next/link";
import { useEffect, useState } from "react";

import { FieldCanvas } from "@/components/FieldCanvas";
import { FailureOverlay } from "@/components/FailureOverlay";
import { FailureTimeline } from "@/components/FailureTimeline";
import { MetricsPanel } from "@/components/MetricsPanel";
import { StageStepper, type Stage } from "@/components/StageStepper";
import { useReplay } from "@/hooks/useReplay";

export default function ReplayPage() {
  const replay = useReplay();
  const [stage, setStage] = useState<Stage>("post_resample");

  useEffect(() => {
    const requested = new URLSearchParams(window.location.search).get("file");
    if (!requested) return;
    if (!replay.files.includes(requested)) return;
    if (replay.selected === requested) return;
    replay.setSelected(requested);
  }, [replay]);

  return (
    <main className="flex min-h-screen flex-col bg-zinc-950 p-4 text-zinc-100">
      <div className="mb-3 flex items-center justify-between">
        <h1 className="text-xl font-semibold">Replay Viewer</h1>
        <Link className="rounded bg-zinc-800 px-3 py-1 text-sm" href="/">
          Back to Live
        </Link>
      </div>

      <div className="mb-3 flex items-center gap-2">
        <label className="text-sm">Replay:</label>
        <select
          className="rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
          value={replay.selected}
          onChange={(e) => replay.setSelected(e.target.value)}
        >
          {replay.files.map((file) => (
            <option key={file} value={file}>
              {file}
            </option>
          ))}
        </select>
      </div>

      <div className="grid flex-1 grid-cols-1 gap-4 lg:grid-cols-[1fr_320px]">
        <FieldCanvas
          tick={replay.currentTick}
          stage={stage}
          obstacles={replay.replayObstacles}
          waypoints={replay.replayWaypoints}
          prevGroundTruth={
            replay.cursor > 0
              ? replay.allLoadedTicks[replay.cursor - 1]?.ground_truth ?? null
              : null
          }
          className="h-[72vh] w-full rounded border border-zinc-700"
        />
        <aside className="space-y-3">
          <div className="rounded border border-zinc-700 p-3">
            <div className="mb-2 flex items-center gap-2">
              <button
                type="button"
                className="rounded bg-zinc-700 px-2 py-1"
                onClick={() => replay.setPlaying((v) => !v)}
              >
                {replay.playing ? "Pause" : "Play"}
              </button>
              <button
                type="button"
                className="rounded bg-zinc-700 px-2 py-1"
                onClick={() => replay.setCursor(Math.max(0, replay.cursor - 1))}
              >
                Prev
              </button>
              <button
                type="button"
                className="rounded bg-zinc-700 px-2 py-1"
                onClick={() =>
                  replay.setCursor(Math.min(Math.max(0, replay.totalTicks - 1), replay.cursor + 1))
                }
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
              max={Math.max(0, replay.totalTicks - 1)}
              value={replay.cursor}
              onChange={(e) => replay.setCursor(Number(e.target.value))}
            />
            <div className="mt-1 text-xs text-zinc-400">
              Tick {replay.cursor} / {Math.max(0, replay.totalTicks - 1)}
            </div>
            <div className="mt-2">
              <FailureTimeline
                ticks={replay.allLoadedTicks}
                cursor={replay.cursor}
                onJump={replay.setCursor}
              />
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
