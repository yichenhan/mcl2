"use client";

import Link from "next/link";
import { useEffect, useMemo, useState } from "react";
import { useRouter } from "next/navigation";

import { FieldCanvas } from "@/components/FieldCanvas";
import { api } from "@/lib/api";
import type { RouteDefinition } from "@/lib/types";

export default function RoutesPage() {
  const router = useRouter();
  const [files, setFiles] = useState<string[]>([]);
  const [selected, setSelected] = useState<string>("");
  const [route, setRoute] = useState<RouteDefinition | null>(null);
  const [seed, setSeed] = useState<number>(42);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    void api
      .listRoutes()
      .then((list) => {
        setFiles(list);
        if (list.length > 0) setSelected(list[0]);
      })
      .catch((e) => setError(e instanceof Error ? e.message : "Failed to load routes"));
  }, []);

  useEffect(() => {
    if (!selected) return;
    void api
      .getRoute(selected)
      .then((r) => {
        setRoute(r);
        setSeed(r.failure_seed);
      })
      .catch((e) => setError(e instanceof Error ? e.message : "Failed to load route"));
  }, [selected]);

  const subtitle = useMemo(() => {
    if (!route) return "No route selected.";
    return `${route.waypoints.length} waypoints, ${route.obstacles.length} obstacles`;
  }, [route]);

  return (
    <main className="flex min-h-screen flex-col bg-zinc-950 p-4 text-zinc-100">
      <div className="mb-3 flex items-center justify-between">
        <h1 className="text-xl font-semibold">Routes</h1>
        <div className="flex gap-2">
          <Link className="rounded bg-zinc-800 px-3 py-1 text-sm" href="/editor">
            Open Editor
          </Link>
          <Link className="rounded bg-zinc-800 px-3 py-1 text-sm" href="/replay">
            Replay
          </Link>
        </div>
      </div>

      <div className="mb-3 flex flex-wrap items-center gap-2">
        <label className="text-sm">Route file</label>
        <select
          className="rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
          value={selected}
          onChange={(e) => setSelected(e.target.value)}
        >
          {files.map((f) => (
            <option key={f} value={f}>
              {f}
            </option>
          ))}
        </select>
        <label className="text-sm">Seed</label>
        <input
          type="number"
          className="w-28 rounded border border-zinc-700 bg-zinc-900 px-2 py-1"
          value={seed}
          onChange={(e) => setSeed(Number(e.target.value))}
        />
        <button
          type="button"
          className="rounded bg-cyan-700 px-3 py-1 text-sm disabled:opacity-50"
          disabled={!selected || busy}
          onClick={async () => {
            setBusy(true);
            setError(null);
            try {
              const run = await api.runRoute(selected, seed);
              router.push(`/replay?file=${encodeURIComponent(run.replay_file)}`);
            } catch (e) {
              setError(e instanceof Error ? e.message : "Run failed");
            } finally {
              setBusy(false);
            }
          }}
        >
          {busy ? "Running..." : "Run"}
        </button>
      </div>

      <div className="mb-3 text-sm text-zinc-400">{subtitle}</div>

      <div className="grid flex-1 grid-cols-1 gap-4 lg:grid-cols-[1fr_320px]">
        <FieldCanvas
          tick={null}
          stage="post_resample"
          obstacles={route?.obstacles ?? []}
          waypoints={route?.waypoints ?? []}
          className="h-[72vh] w-full rounded border border-zinc-700"
        />
        <aside className="space-y-2 rounded border border-zinc-700 p-3 text-sm text-zinc-300">
          <div className="text-xs uppercase text-zinc-500">Route detail</div>
          <div>Name: {route?.name ?? "-"}</div>
          <div>Description: {route?.description ?? "-"}</div>
          <div>Max ticks: {route?.max_ticks ?? "-"}</div>
          <div>Failure seed: {route?.failure_seed ?? "-"}</div>
          {error ? <div className="text-xs text-red-300">{error}</div> : null}
        </aside>
      </div>
    </main>
  );
}

