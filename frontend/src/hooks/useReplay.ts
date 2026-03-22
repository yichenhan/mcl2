"use client";

import { useCallback, useEffect, useMemo, useState } from "react";

import { api } from "@/lib/api";
import type { AABB, TickState, Waypoint } from "@/lib/types";

const CHUNK_SIZE = 50;
const LOCAL_REPLAY_PREFIX = "__local__:";

interface LocalReplay {
  id: string;
  displayName: string;
  ticks: TickState[];
  meta: Record<string, unknown>;
}

function parseUploadedReplay(
  raw: unknown,
  fileName: string,
): LocalReplay {
  const fallbackSessionId = fileName.replace(/\.json$/i, "") || "uploaded_replay";
  let ticks: TickState[] = [];
  let metaSource: Record<string, unknown> = {};

  if (Array.isArray(raw)) {
    ticks = raw as TickState[];
  } else if (raw && typeof raw === "object") {
    metaSource = raw as Record<string, unknown>;
    const maybeTicks = metaSource.ticks;
    if (!Array.isArray(maybeTicks)) {
      throw new Error("Uploaded JSON must contain a 'ticks' array (or be a raw tick array).");
    }
    ticks = maybeTicks as TickState[];
  } else {
    throw new Error("Uploaded JSON must be an object or array.");
  }

  if (ticks.length === 0) {
    throw new Error("Uploaded replay has no ticks.");
  }

  const first = ticks[0] as unknown as Record<string, unknown>;
  if (!first.post_predict || !first.post_update || !first.post_resample) {
    throw new Error("Uploaded ticks are not replay compatible (missing post_predict/post_update/post_resample).");
  }

  const sessionId =
    typeof metaSource.session_id === "string" && metaSource.session_id.trim().length > 0
      ? metaSource.session_id
      : fallbackSessionId;
  const id = `${LOCAL_REPLAY_PREFIX}${sessionId}`;
  const meta: Record<string, unknown> = {
    ...metaSource,
    session_id: sessionId,
    total_ticks:
      typeof metaSource.total_ticks === "number" ? metaSource.total_ticks : ticks.length,
    obstacles: Array.isArray(metaSource.obstacles) ? metaSource.obstacles : [],
    config:
      metaSource.config && typeof metaSource.config === "object"
        ? metaSource.config
        : {},
  };
  return { id, displayName: `${sessionId} (uploaded)`, ticks, meta };
}

export function useReplay() {
  const [files, setFiles] = useState<string[]>([]);
  const [selected, setSelected] = useState<string>("");
  const [meta, setMeta] = useState<Record<string, unknown> | null>(null);
  const [cache, setCache] = useState<Map<number, TickState[]>>(new Map());
  const [cursor, setCursor] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [localReplay, setLocalReplay] = useState<LocalReplay | null>(null);
  const [backendAvailable, setBackendAvailable] = useState(true);

  const isLocalReplaySelected = useMemo(
    () => selected.startsWith(LOCAL_REPLAY_PREFIX),
    [selected],
  );

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
      const ticks = isLocalReplaySelected
        ? (localReplay?.ticks.slice(from, to) ?? [])
        : await api.getReplayTicks(selected, from, to);
      setCache((prev) => {
        const next = new Map(prev);
        next.set(chunkIdx, ticks);
        return next;
      });
    },
    [cache, isLocalReplaySelected, localReplay?.ticks, selected],
  );

  const refreshList = useCallback(async () => {
    let replayFiles: string[] = [];
    try {
      replayFiles = await api.listReplays();
      setBackendAvailable(true);
    } catch {
      setBackendAvailable(false);
    }
    const withLocal = localReplay
      ? [localReplay.id, ...replayFiles]
      : replayFiles;
    setFiles(withLocal);
    if (withLocal.length > 0 && !selected) {
      setSelected(withLocal[0]);
    }
  }, [localReplay, selected]);

  const selectReplay = useCallback(async (file: string) => {
    setLoadError(null);
    setCache(new Map());
    setCursor(0);
    if (file.startsWith(LOCAL_REPLAY_PREFIX)) {
      if (!localReplay || localReplay.id !== file) {
        setLoadError("Selected uploaded replay is no longer available in memory.");
        setMeta(null);
        return;
      }
      setMeta(localReplay.meta);
      setCache(new Map([[0, localReplay.ticks.slice(0, CHUNK_SIZE)]]));
      return;
    }
    const m = await api.getReplayMeta(file);
    setMeta(m);
    const firstChunk = await api.getReplayTicks(file, 0, CHUNK_SIZE);
    setCache(new Map([[0, firstChunk]]));
  }, [localReplay]);

  const loadUploadedReplay = useCallback(
    async (file: File) => {
      setLoadError(null);
      let parsed: unknown = null;
      try {
        parsed = JSON.parse(await file.text());
      } catch {
        throw new Error("Failed to parse JSON file.");
      }
      const local = parseUploadedReplay(parsed, file.name);
      setLocalReplay(local);
      let replayFiles: string[] = [];
      try {
        replayFiles = await api.listReplays();
        setBackendAvailable(true);
      } catch {
        setBackendAvailable(false);
      }
      setFiles([local.id, ...replayFiles]);
      setSelected(local.id);
    },
    [],
  );

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
    backendAvailable,
    isLocalReplaySelected,
    localReplayName: localReplay?.displayName ?? null,
    selected,
    setSelected,
    meta,
    loadError,
    totalTicks,
    cursor,
    setCursor,
    currentTick,
    playing,
    setPlaying,
    speed,
    setSpeed,
    refreshList,
    loadUploadedReplay,
    allLoadedTicks,
    replayObstacles,
    replayWaypoints,
  };
}
