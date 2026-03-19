"use client";

import { useCallback, useEffect, useRef, useState } from "react";

import { api } from "@/lib/api";
import type {
  AABB,
  Action,
  SessionConfigResponse,
  TickState,
} from "@/lib/types";

const TICK_MS = 50;

export function useSimSession() {
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [ticks, setTicks] = useState<TickState[]>([]);
  const [config, setConfig] = useState<SessionConfigResponse | null>(null);
  const [isRunning, setIsRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const inFlightRef = useRef(false);

  const start = useCallback(
    async (numParticles: number, obstacles: AABB[]) => {
      setError(null);
      const started = await api.startSession({
        num_particles: numParticles,
        obstacles,
      });
      setSessionId(started.session_id);
      setTicks([]);
      setIsRunning(true);
      const cfg = await api.getSessionConfig(started.session_id);
      setConfig(cfg);
    },
    [],
  );

  const tickOnce = useCallback(
    async (action: Action) => {
      if (!sessionId || inFlightRef.current) return;
      inFlightRef.current = true;
      try {
        const next = await api.tick(sessionId, action);
        setTicks((prev) => [...prev, next]);
      } catch (err) {
        setError(err instanceof Error ? err.message : "Unknown tick error");
      } finally {
        inFlightRef.current = false;
      }
    },
    [sessionId],
  );

  const close = useCallback(async () => {
    if (!sessionId) return;
    try {
      await api.closeSession(sessionId);
    } catch {
      // Keep close best-effort so UI can reset.
    }
    setIsRunning(false);
    setSessionId(null);
  }, [sessionId]);

  const runWithAction = useCallback(
    (action: Action) => {
      if (!isRunning || !sessionId) return () => undefined;
      let stopped = false;
      const run = async () => {
        if (stopped) return;
        await tickOnce(action);
        if (!stopped) setTimeout(run, TICK_MS);
      };
      void run();
      return () => {
        stopped = true;
      };
    },
    [isRunning, sessionId, tickOnce],
  );

  useEffect(() => {
    return () => {
      if (sessionId) {
        void api.closeSession(sessionId);
      }
    };
  }, [sessionId]);

  return {
    sessionId,
    ticks,
    latest: ticks.length > 0 ? ticks[ticks.length - 1] : null,
    config,
    isRunning,
    error,
    start,
    tickOnce,
    runWithAction,
    close,
  };
}
