"use client";

import Link from "next/link";
import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";

import { RouteEditorCanvas } from "@/components/RouteEditorCanvas";
import { useRouteEditor } from "@/hooks/useRouteEditor";
import { api } from "@/lib/api";

export default function RouteEditorPage() {
  const router = useRouter();
  const editor = useRouteEditor();
  const [presets, setPresets] = useState<string[]>([]);

  useEffect(() => {
    void api.listRoutes().then(setPresets).catch(() => setPresets([]));
  }, []);

  return (
    <main className="flex min-h-screen flex-col bg-zinc-950 p-4 text-zinc-100">
      <div className="mb-3 flex items-center justify-between">
        <h1 className="text-xl font-semibold">Route Editor</h1>
        <div className="flex items-center gap-2">
          <Link className="rounded bg-zinc-800 px-3 py-1 text-sm" href="/routes">
            Browse Routes
          </Link>
          <Link className="rounded bg-zinc-800 px-3 py-1 text-sm" href="/replay">
            Replay
          </Link>
        </div>
      </div>

      <div className="grid flex-1 grid-cols-1 gap-4 lg:grid-cols-[1fr_340px]">
        <RouteEditorCanvas
          mode={editor.editMode}
          waypoints={editor.route.waypoints}
          obstacles={editor.route.obstacles}
          onWaypointsChange={editor.setWaypoints}
          onObstaclesChange={editor.setObstacles}
          onRobotSet={(x, y, heading) => editor.setRobotStart({ x, y, heading_deg: heading ?? 0 })}
        />

        <aside className="space-y-3">
          <div className="rounded border border-zinc-700 p-3">
            <div className="mb-2 text-xs uppercase text-zinc-400">Mode</div>
            <div className="flex gap-2">
              {(["waypoint", "obstacle", "robot"] as const).map((m) => (
                <button
                  key={m}
                  type="button"
                  className={`rounded px-2 py-1 text-xs ${
                    editor.editMode === m ? "bg-cyan-700" : "bg-zinc-800"
                  }`}
                  onClick={() => editor.setEditMode(m)}
                >
                  {m}
                </button>
              ))}
            </div>
          </div>

          <div className="rounded border border-zinc-700 p-3 text-sm">
            <label className="mb-1 block text-xs text-zinc-400">Route name</label>
            <input
              className="mb-2 w-full rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
              value={editor.route.name}
              onChange={(e) => editor.patchRoute({ name: e.target.value })}
            />
            <label className="mb-1 block text-xs text-zinc-400">Description</label>
            <input
              className="mb-2 w-full rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
              value={editor.route.description}
              onChange={(e) => editor.patchRoute({ description: e.target.value })}
            />
            <label className="mb-1 block text-xs text-zinc-400">Preset</label>
            <select
              className="mb-2 w-full rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
              onChange={(e) => {
                if (e.target.value) void editor.loadPreset(e.target.value);
              }}
              value=""
            >
              <option value="">Load preset...</option>
              {presets.map((p) => (
                <option key={p} value={p}>
                  {p}
                </option>
              ))}
            </select>

            <div className="grid grid-cols-2 gap-2">
              <div>
                <label className="mb-1 block text-xs text-zinc-400">Lookahead</label>
                <input
                  type="number"
                  className="w-full rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
                  value={editor.route.pure_pursuit.lookahead_distance}
                  onChange={(e) =>
                    editor.patchRoute({
                      pure_pursuit: {
                        ...editor.route.pure_pursuit,
                        lookahead_distance: Number(e.target.value),
                      },
                    })
                  }
                />
              </div>
              <div>
                <label className="mb-1 block text-xs text-zinc-400">Linear vel</label>
                <input
                  type="number"
                  className="w-full rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
                  value={editor.route.pure_pursuit.linear_velocity}
                  onChange={(e) =>
                    editor.patchRoute({
                      pure_pursuit: {
                        ...editor.route.pure_pursuit,
                        linear_velocity: Number(e.target.value),
                      },
                    })
                  }
                />
              </div>
            </div>
            <div className="mt-2 grid grid-cols-2 gap-2">
              <button
                type="button"
                className="rounded bg-emerald-700 px-3 py-2 text-xs disabled:opacity-50"
                disabled={editor.busy}
                onClick={() => void editor.saveRoute()}
              >
                Save Route
              </button>
              <button
                type="button"
                className="rounded bg-cyan-700 px-3 py-2 text-xs disabled:opacity-50"
                disabled={editor.busy}
                onClick={async () => {
                  const run = await editor.runRoute();
                  router.push(`/replay?file=${encodeURIComponent(run.replay_file)}`);
                }}
              >
                Run Route
              </button>
            </div>
            {editor.error ? <div className="mt-2 text-xs text-red-300">{editor.error}</div> : null}
          </div>
        </aside>
      </div>
    </main>
  );
}

