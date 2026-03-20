"use client";

import { useCallback, useEffect, useMemo, useState } from "react";

import { api } from "@/lib/api";
import type { AABB, TickState, Waypoint } from "@/lib/types";

const CHUNK_SIZE = 50;

export function useReplay() {
  const [files, setFiles] = useState<string[]>([]);
  const [selected, setSelected] = useState<string>("");
  const [meta, setMeta] = useState<Record<string, unknown> | null>(null);
  const [cache, setCache] = useState<Map<number, TickState[]>>(new Map());
  const [cursor, setCursor] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);

  const totalTicks = useMemo(() => {
    if (!meta) return 0;
    const v = meta.total_ticks;
    return typeof v === "number" ? v : 0;
  }, [meta]);

  const loadChunk = useCallback(
    async (chunkIdx: number) => {
      if (!selected || cache.has(chunkIdx)) return;
      const from = chunkIdx * CHUNK_SIZE;
      const to = from + CHUNK_SIZE;
      const ticks = await api.getReplayTicks(selected, from, to);
      setCache((prev) => {
        const next = new Map(prev);
        next.set(chunkIdx, ticks);
        return next;
      });
    },
    [cache, selected],
  );

  const refreshList = useCallback(async () => {
    const replayFiles = await api.listReplays();
    setFiles(replayFiles);
    if (replayFiles.length > 0 && !selected) {
      setSelected(replayFiles[0]);
    }
  }, [selected]);

  const selectReplay = useCallback(async (file: string) => {
    setCache(new Map());
    setCursor(0);
    const m = await api.getReplayMeta(file);
    setMeta(m);
    const firstChunk = await api.getReplayTicks(file, 0, CHUNK_SIZE);
    setCache(new Map([[0, firstChunk]]));
  }, []);

  useEffect(() => {
    queueMicrotask(() => {
      void refreshList();
    });
  }, [refreshList]);

  useEffect(() => {
    if (!selected) return;
    queueMicrotask(() => {
      void selectReplay(selected);
    });
  }, [selected, selectReplay]);

  useEffect(() => {
    if (!playing || totalTicks <= 0) return;
    const ms = Math.max(50, Math.floor(200 / speed));
    const id = window.setInterval(() => {
      setCursor((prev) => {
        const next = Math.min(totalTicks - 1, prev + 1);
        return next;
      });
    }, ms);
    return () => window.clearInterval(id);
  }, [playing, speed, totalTicks]);

  useEffect(() => {
    const chunkIdx = Math.floor(cursor / CHUNK_SIZE);
    queueMicrotask(() => {
      void loadChunk(chunkIdx);
      if (cursor >= (chunkIdx + 1) * CHUNK_SIZE - 5) {
        void loadChunk(chunkIdx + 1);
      }
    });
  }, [cursor, loadChunk]);

  const currentTick = useMemo(() => {
    const chunkIdx = Math.floor(cursor / CHUNK_SIZE);
    const inChunk = cursor % CHUNK_SIZE;
    return cache.get(chunkIdx)?.[inChunk] ?? null;
  }, [cache, cursor]);

  const allLoadedTicks = useMemo(() => {
    const keys = [...cache.keys()].sort((a, b) => a - b);
    const out: TickState[] = [];
    for (const k of keys) {
      const chunk = cache.get(k);
      if (chunk) out.push(...chunk);
    }
    return out;
  }, [cache]);

  const replayObstacles = useMemo(() => {
    if (!meta) return [] as AABB[];
    const v = meta.obstacles;
    return Array.isArray(v) ? (v as AABB[]) : [];
  }, [meta]);

  const replayWaypoints = useMemo(() => {
    if (!meta) return [] as Waypoint[];
    const cfg = meta.config as Record<string, unknown> | undefined;
    if (!cfg) return [] as Waypoint[];
    const w = cfg.waypoints;
    return Array.isArray(w) ? (w as Waypoint[]) : [];
  }, [meta]);

  return {
    files,
    selected,
    setSelected,
    meta,
    totalTicks,
    cursor,
    setCursor,
    currentTick,
    playing,
    setPlaying,
    speed,
    setSpeed,
    refreshList,
    allLoadedTicks,
    replayObstacles,
    replayWaypoints,
  };
}
