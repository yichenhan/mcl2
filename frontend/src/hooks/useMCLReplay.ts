"use client";

import { useCallback, useEffect, useMemo, useState } from "react";
import type { ChangeEvent, DragEvent } from "react";

import { api } from "@/lib/api";
import type { AABB, MCLReplayFile, MCLTickResult } from "@/lib/types";

const CHUNK_SIZE = 50;
const MCL_JSON_SEGMENT_RE = /\[MCL_JSON_START\]\s*([\s\S]*?)\s*\[MCL_JSON_FINISH\]/g;

function parseReplayPayload(input: unknown): {
  ticks: MCLTickResult[];
  obstacles: AABB[];
  fieldHalf?: number;
} {
  if (Array.isArray(input)) {
    return { ticks: input as MCLTickResult[], obstacles: [] };
  }
  if (!input || typeof input !== "object") {
    throw new Error("Expected JSON object or array");
  }
  const obj = input as Partial<MCLReplayFile>;
  if (!Array.isArray(obj.ticks)) {
    throw new Error("Missing ticks[] array");
  }
  return {
    ticks: obj.ticks as MCLTickResult[],
    obstacles: Array.isArray(obj.obstacles) ? (obj.obstacles as AABB[]) : [],
    fieldHalf: typeof obj.field_half === "number" ? obj.field_half : undefined,
  };
}

function parseReplayText(text: string): {
  ticks: MCLTickResult[];
  obstacles: AABB[];
  fieldHalf?: number;
} {
  // First try standard JSON file parsing.
  try {
    const parsed = JSON.parse(text) as unknown;
    return parseReplayPayload(parsed);
  } catch {
    // Fallback: parse a messy log file containing marked JSON tick payloads.
  }

  const ticks: MCLTickResult[] = [];
  const matches = text.matchAll(MCL_JSON_SEGMENT_RE);
  for (const match of matches) {
    const raw = (match[1] ?? "").trim();
    if (!raw) continue;
    try {
      ticks.push(JSON.parse(raw) as MCLTickResult);
    } catch {
      // Ignore malformed segments and keep valid ticks.
    }
  }

  if (ticks.length === 0) {
    throw new Error(
      "Could not parse file as JSON replay or extract any [MCL_JSON_START]...[MCL_JSON_FINISH] tick blocks.",
    );
  }
  return { ticks, obstacles: [] };
}

export function useMCLReplay() {
  const [files, setFiles] = useState<string[]>([]);
  const [selected, setSelected] = useState<string>("");
  const [meta, setMeta] = useState<Record<string, unknown> | null>(null);
  const [cache, setCache] = useState<Map<number, MCLTickResult[]>>(new Map());
  const [cursor, setCursor] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [error, setError] = useState<string | null>(null);

  const [droppedTicks, setDroppedTicks] = useState<MCLTickResult[] | null>(null);
  const [droppedObstacles, setDroppedObstacles] = useState<AABB[]>([]);
  const [droppedFieldHalf, setDroppedFieldHalf] = useState<number | undefined>(undefined);
  const [droppedFileName, setDroppedFileName] = useState<string | null>(null);

  const usingDropped = droppedTicks !== null;

  const totalTicks = useMemo(() => {
    if (usingDropped) return droppedTicks.length;
    if (!meta) return 0;
    const v = meta.total_ticks;
    return typeof v === "number" ? v : 0;
  }, [droppedTicks, meta, usingDropped]);

  const loadChunk = useCallback(
    async (chunkIdx: number) => {
      if (!selected || usingDropped || cache.has(chunkIdx)) return;
      const from = chunkIdx * CHUNK_SIZE;
      const to = from + CHUNK_SIZE;
      const ticks = await api.getMCLReplayTicks(selected, from, to);
      setCache((prev) => {
        const next = new Map(prev);
        next.set(chunkIdx, ticks);
        return next;
      });
    },
    [cache, selected, usingDropped],
  );

  const refreshList = useCallback(async () => {
    try {
      const replayFiles = await api.listMCLReplays();
      setFiles(replayFiles);
      if (replayFiles.length > 0 && !selected) {
        setSelected(replayFiles[0]);
      }
    } catch (e) {
      setError(e instanceof Error ? e.message : "Failed to load MCL replay list");
    }
  }, [selected]);

  const selectReplay = useCallback(async (file: string) => {
    setError(null);
    setCache(new Map());
    setCursor(0);
    const m = await api.getMCLReplayMeta(file);
    setMeta(m);
    const firstChunk = await api.getMCLReplayTicks(file, 0, CHUNK_SIZE);
    setCache(new Map([[0, firstChunk]]));
  }, []);

  useEffect(() => {
    queueMicrotask(() => {
      void refreshList();
    });
  }, [refreshList]);

  useEffect(() => {
    if (usingDropped || !selected) return;
    queueMicrotask(() => {
      void selectReplay(selected);
    });
  }, [selected, selectReplay, usingDropped]);

  useEffect(() => {
    if (!playing || totalTicks <= 0) return;
    const ms = Math.max(50, Math.floor(200 / speed));
    const id = window.setInterval(() => {
      setCursor((prev) => Math.min(totalTicks - 1, prev + 1));
    }, ms);
    return () => window.clearInterval(id);
  }, [playing, speed, totalTicks]);

  useEffect(() => {
    if (usingDropped) return;
    const chunkIdx = Math.floor(cursor / CHUNK_SIZE);
    queueMicrotask(() => {
      void loadChunk(chunkIdx);
      if (cursor >= (chunkIdx + 1) * CHUNK_SIZE - 5) {
        void loadChunk(chunkIdx + 1);
      }
    });
  }, [cursor, loadChunk, usingDropped]);

  const currentTick = useMemo(() => {
    if (usingDropped) return droppedTicks[cursor] ?? null;
    const chunkIdx = Math.floor(cursor / CHUNK_SIZE);
    const inChunk = cursor % CHUNK_SIZE;
    return cache.get(chunkIdx)?.[inChunk] ?? null;
  }, [cache, cursor, droppedTicks, usingDropped]);

  const replayObstacles = useMemo(() => {
    if (usingDropped) return droppedObstacles;
    if (!meta) return [] as AABB[];
    const v = meta.obstacles;
    return Array.isArray(v) ? (v as AABB[]) : [];
  }, [droppedObstacles, meta, usingDropped]);

  const fieldHalf = useMemo(() => {
    if (usingDropped) return droppedFieldHalf;
    if (!meta) return undefined;
    const v = meta.field_half;
    return typeof v === "number" ? v : undefined;
  }, [droppedFieldHalf, meta, usingDropped]);

  const clearDropped = useCallback(() => {
    setDroppedTicks(null);
    setDroppedObstacles([]);
    setDroppedFieldHalf(undefined);
    setDroppedFileName(null);
    setError(null);
    setCursor(0);
  }, []);

  const loadDroppedFile = useCallback(async (file: File) => {
    setError(null);
    const text = await file.text();
    const decoded = parseReplayText(text);
    setDroppedTicks(decoded.ticks);
    setDroppedObstacles(decoded.obstacles);
    setDroppedFieldHalf(decoded.fieldHalf);
    setDroppedFileName(file.name);
    setCursor(0);
    setPlaying(false);
  }, []);

  const onDrop = useCallback(
    async (event: DragEvent<HTMLElement>) => {
      event.preventDefault();
      const file = event.dataTransfer.files?.[0];
      if (!file) return;
      try {
        await loadDroppedFile(file);
      } catch (e) {
        setError(e instanceof Error ? e.message : "Failed to parse dropped replay file");
      }
    },
    [loadDroppedFile],
  );

  const onFileInput = useCallback(
    async (event: ChangeEvent<HTMLInputElement>) => {
      const file = event.target.files?.[0];
      if (!file) return;
      try {
        await loadDroppedFile(file);
      } catch (e) {
        setError(e instanceof Error ? e.message : "Failed to parse selected replay file");
      } finally {
        event.target.value = "";
      }
    },
    [loadDroppedFile],
  );

  const onDragOver = useCallback((event: DragEvent<HTMLElement>) => {
    event.preventDefault();
  }, []);

  return {
    files,
    selected,
    setSelected,
    cursor,
    setCursor,
    currentTick,
    totalTicks,
    playing,
    setPlaying,
    speed,
    setSpeed,
    replayObstacles,
    fieldHalf,
    usingDropped,
    droppedFileName,
    clearDropped,
    onDrop,
    onDragOver,
    onFileInput,
    error,
  };
}
