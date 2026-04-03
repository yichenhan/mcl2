"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { api } from "@/lib/api";
import { parseReplayText } from "@/lib/replayParser";
import { parseSessionAnalytics } from "@/lib/sessionAnalyticsDefaults";
import type {
  FailureConfig,
  Obstacle,
  SessionAnalytics,
  SessionMode,
  TickState,
  Waypoint,
} from "@/lib/types";

const DEFAULT_NUM_PARTICLES = 300;

interface StartOptions {
  mode: SessionMode;
  mapName?: string;
  algorithm?: "arc_nav" | "turn_and_go";
  /** Defaults to 300 when omitted. */
  numParticles?: number;
  obstacles: Obstacle[];
  failureConfig?: FailureConfig;
  sessionName?: string;
  maxTicks?: number;
  initialState?: { x: number; y: number; heading_deg: number };
  waypoints?: Waypoint[];
}

const DEFAULT_FAILURE_CONFIG: FailureConfig = {
  sensor_dead_prob: 0.02,
  sensor_stuck_prob: 0.01,
  odom_spike_prob: 0.005,
  heading_bias_prob: 0.005,
  spurious_reflection_prob: 0.01,
  kidnap_prob: 0,
  min_duration_ticks: 5,
  max_duration_ticks: 30,
  sensor_dead_max_duration_ticks: 30,
  sensor_stuck_max_duration_ticks: 5,
  odom_spike_max_duration_ticks: 15,
  heading_bias_max_duration_ticks: 40,
  spurious_reflection_max_duration_ticks: 1,
  odom_spike_range: [1.5, 3.0],
  heading_bias_range: [-10.0, 10.0],
  spurious_reflection_range: [2.0, 12.0],
};

export function useUnifiedSession() {
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [mode, setMode] = useState<SessionMode>("live");
  const [ticks, setTicks] = useState<TickState[]>([]);
  const [isRunning, setIsRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [cursor, setCursor] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [replayFiles, setReplayFiles] = useState<string[]>([]);
  const [replayAnalytics, setReplayAnalytics] = useState<SessionAnalytics | null>(null);
  const [pendingSeekTick, setPendingSeekTick] = useState<number | null>(null);
  const inFlightRef = useRef(false);

  const currentTick = useMemo(() => {
    if (ticks.length === 0) return null;
    if (mode !== "replay") return ticks[ticks.length - 1];
    return ticks[Math.max(0, Math.min(cursor, ticks.length - 1))] ?? null;
  }, [cursor, mode, ticks]);

  const start = useCallback(async (opts: StartOptions) => {
    setError(null);
    setMode(opts.mode);
    setTicks([]);
    setCursor(0);
    setReplayAnalytics(null);
    setPendingSeekTick(null);
    const started = await api.startSession({
      mode: opts.mode,
      map_name: opts.mapName,
      algorithm: opts.algorithm,
      num_particles: opts.numParticles ?? DEFAULT_NUM_PARTICLES,
      obstacles: opts.obstacles,
      failure_config: opts.failureConfig ?? DEFAULT_FAILURE_CONFIG,
      session_name: opts.sessionName,
      max_ticks: opts.maxTicks,
      initial_state: opts.initialState,
    });
    setSessionId(started.session_id);
    const initial = await api.tickContinuous(started.session_id, 0, 0);
    setTicks([initial]);
    setIsRunning(true);
  }, []);

  const tickContinuous = useCallback(async (linearVel: number, angularVelDeg: number) => {
    if (!sessionId || !isRunning || inFlightRef.current) return;
    inFlightRef.current = true;
    try {
      const next = await api.tickContinuous(sessionId, linearVel, angularVelDeg);
      setTicks((prev) => [...prev, next]);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Tick error");
    } finally {
      inFlightRef.current = false;
    }
  }, [isRunning, sessionId]);

  const tickAuto = useCallback(async () => {
    if (!sessionId || !isRunning || inFlightRef.current) return;
    inFlightRef.current = true;
    try {
      const next = await api.tickAuto(sessionId);
      setTicks((prev) => [...prev, next]);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Tick error");
    } finally {
      inFlightRef.current = false;
    }
  }, [isRunning, sessionId]);

  const stop = useCallback(async () => {
    if (!sessionId) return;
    const id = sessionId;
    setIsRunning(false);
    setSessionId(null);
    await api.closeSession(id);
  }, [sessionId]);

  const refreshReplayList = useCallback(async () => {
    const files = await api.listReplays();
    setReplayFiles(files);
  }, []);

  const loadReplay = useCallback(async (file: string) => {
    setMode("replay");
    setError(null);
    const meta = await api.getReplayMeta(file);
    setReplayAnalytics(parseSessionAnalytics(meta.analytics));
    const totalTicks = typeof meta.total_ticks === "number" ? meta.total_ticks : 0;
    const loaded = await api.getReplayTicks(file, 0, totalTicks);
    setTicks(loaded);
    setCursor(0);
  }, []);

  const loadReplayAndSeek = useCallback(
    async (file: string, tick: number) => {
      setPendingSeekTick(Math.max(0, Math.floor(tick)));
      await loadReplay(file);
    },
    [loadReplay],
  );

  const loadReplayUpload = useCallback(async (file: File) => {
    setMode("replay");
    setError(null);
    setReplayAnalytics(null);
    setPendingSeekTick(null);
    const text = await file.text();
    const parsed = parseReplayText(text);
    setTicks(parsed);
    setCursor(0);
    setIsPlaying(false);
  }, []);

  /** Clear live session + replay data when returning to the setup step. */
  const resetForSetup = useCallback(() => {
    setSessionId(null);
    setIsRunning(false);
    setTicks([]);
    setCursor(0);
    setIsPlaying(false);
    setError(null);
    setReplayAnalytics(null);
    setPendingSeekTick(null);
    inFlightRef.current = false;
  }, []);

  useEffect(() => {
    if (pendingSeekTick === null || ticks.length === 0) return;
    const t = Math.min(pendingSeekTick, ticks.length - 1);
    setCursor(t);
    setPendingSeekTick(null);
  }, [pendingSeekTick, ticks]);

  useEffect(() => {
    if (mode !== "replay" || !isPlaying || ticks.length === 0) return;
    const ms = Math.max(50, Math.floor(200 / speed));
    const id = window.setInterval(() => {
      setCursor((prev) => Math.min(ticks.length - 1, prev + 1));
    }, ms);
    return () => window.clearInterval(id);
  }, [isPlaying, mode, speed, ticks.length]);

  return {
    mode,
    setMode,
    sessionId,
    ticks,
    currentTick,
    isRunning,
    error,
    start,
    stop,
    tickContinuous,
    tickAuto,
    replayFiles,
    refreshReplayList,
    loadReplay,
    loadReplayAndSeek,
    loadReplayUpload,
    replayAnalytics,
    resetForSetup,
    cursor,
    setCursor,
    isPlaying,
    setIsPlaying,
    speed,
    setSpeed,
  };
}
