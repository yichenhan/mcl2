"use client";

import Link from "next/link";
import { useEffect, useMemo, useState } from "react";

import { FieldCanvas } from "@/components/FieldCanvas";
import { MetricsPanel } from "@/components/MetricsPanel";
import { ObstacleEditor } from "@/components/ObstacleEditor";
import { StageStepper, type Stage } from "@/components/StageStepper";
import { useKeyboard } from "@/hooks/useKeyboard";
import { useSimSession } from "@/hooks/useSimSession";
import type { AABB } from "@/lib/types";

export default function Home() {
  const [stage, setStage] = useState<Stage>("post_resample");
  const [numParticles, setNumParticles] = useState(300);
  const [obstacles, setObstacles] = useState<AABB[]>([]);
  const [stopLoop, setStopLoop] = useState<(() => void) | null>(null);
  const { currentAction } = useKeyboard();
  const sim = useSimSession();

  useEffect(() => {
    if (!sim.isRunning) return;
    if (currentAction === "none") {
      stopLoop?.();
      setStopLoop(null);
      return;
    }
    stopLoop?.();
    setStopLoop(() => sim.runWithAction(currentAction));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [currentAction, sim.isRunning]);

  const status = useMemo(() => {
    if (sim.error) return `Error: ${sim.error}`;
    if (!sim.isRunning) return "Session stopped";
    return `Session ${sim.sessionId ?? ""} running`;
  }, [sim.error, sim.isRunning, sim.sessionId]);

  return (
    <main className="flex min-h-screen flex-col bg-zinc-950 p-4 text-zinc-100">
      <div className="mb-3 flex items-center justify-between">
        <h1 className="text-xl font-semibold">MCL Simulator Live</h1>
        <Link className="rounded bg-zinc-800 px-3 py-1 text-sm" href="/replay">
          Open Replay
        </Link>
      </div>

      <div className="grid flex-1 grid-cols-1 gap-4 lg:grid-cols-[1fr_320px]">
        <FieldCanvas
          tick={sim.latest}
          stage={stage}
          obstacles={obstacles}
          className="h-[78vh] w-full rounded border border-zinc-700"
        />

        <aside className="space-y-3">
          <div className="rounded border border-zinc-700 p-3 text-xs text-zinc-300">
            {status}
          </div>
          <div className="rounded border border-zinc-700 p-3">
            <label className="mb-2 block text-xs uppercase text-zinc-400">
              Num particles
            </label>
            <input
              className="w-full rounded border border-zinc-600 bg-zinc-900 px-2 py-1"
              type="number"
              min={50}
              max={2000}
              value={numParticles}
              disabled={sim.isRunning}
              onChange={(e) => setNumParticles(Number(e.target.value))}
            />
            <div className="mt-3 flex gap-2">
              <button
                type="button"
                disabled={sim.isRunning}
                className="rounded bg-green-600 px-3 py-1 disabled:opacity-50"
                onClick={() => void sim.start(numParticles, obstacles)}
              >
                Start Session
              </button>
              <button
                type="button"
                disabled={!sim.isRunning}
                className="rounded bg-red-600 px-3 py-1 disabled:opacity-50"
                onClick={() => void sim.close()}
              >
                Stop Session
              </button>
            </div>
          </div>
          <StageStepper stage={stage} onStageChange={setStage} />
          <MetricsPanel tick={sim.latest} />
          <ObstacleEditor obstacles={obstacles} onChange={setObstacles} />
        </aside>
      </div>
    </main>
  );
}
