"use client";

import { useCallback, useEffect, useMemo, useState } from "react";

import { AnalyticsPanel } from "@/components/AnalyticsPanel";
import { FieldCanvas } from "@/components/FieldCanvas";
import { GateStatus } from "@/components/GateStatus";
import { Legend } from "@/components/Legend";
import { MetricsPanel } from "@/components/MetricsPanel";
import { SessionSetup } from "@/components/SessionSetup";
import { TimeSeriesPanel } from "@/components/TimeSeriesPanel";
import { TransportControls } from "@/components/TransportControls";
import { useUnifiedSession } from "@/hooks/useUnifiedSession";
import { useWaypointEditor } from "@/hooks/useWaypointEditor";
import { api } from "@/lib/api";
import { buildScrubberAnnotations } from "@/lib/replayAnnotations";
import type { FailureConfig as FailureConfigType, MapPreset, OverlayFlags } from "@/lib/types";

export default function Home() {
  const session = useUnifiedSession();
  const waypointEditor = useWaypointEditor();
  const [showSetup, setShowSetup] = useState(true);
  const [replayFocusEpoch, setReplayFocusEpoch] = useState(0);

  const [maps, setMaps] = useState<MapPreset[]>([]);
  const [mapName, setMapName] = useState("Empty Field");
  const [algorithm, setAlgorithm] = useState<"arc_nav" | "turn_and_go">("arc_nav");
  const [initialPose, setInitialPose] = useState({ x: 0, y: 0, heading_deg: 0 });
  const [failureConfig, setFailureConfig] = useState<FailureConfigType>({
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
  });
  const [overlayFlags, setOverlayFlags] = useState<OverlayFlags>({
    robotTruth: true,
    rawOdom: false,
    chassisPose: true,
    mclEstimate: true,
    acceptedEstimate: true,
    r90Circle: true,
    particles: true,
    heatmap: true,
    sensorReadings: true,
    sensorResiduals: true,
    diffMclPose: true,
    diffMclTruth: true,
    diffPoseTruth: true,
  });

  useEffect(() => {
    void api.listMaps().then((m) => {
      setMaps(m);
      if (m.length > 0) setMapName(m[0].name);
    });
    void session.refreshReplayList();
  }, []);

  useEffect(() => {
    setInitialPose({ x: 0, y: 0, heading_deg: 0 });
  }, [mapName]);

  // Backend-driven tick loop: just send empty POST /tick every 50ms.
  // The backend WaypointFollower computes velocity when navigating.
  useEffect(() => {
    if (!session.isRunning || session.mode === "replay") return;
    const id = window.setInterval(() => {
      void session.tickAuto();
    }, 50);
    return () => window.clearInterval(id);
  }, [session.isRunning, session.mode, session.tickAuto]);

  const status = useMemo(() => {
    if (session.error) return `Error: ${session.error}`;
    if (!session.isRunning) return "Session stopped";
    return `Session ${session.sessionId ?? ""} running`;
  }, [session.error, session.isRunning, session.sessionId]);

  const selectedMap = maps.find((m) => m.name === mapName);

  const scrubberAnnotations = useMemo(
    () => buildScrubberAnnotations(session.ticks, session.replayAnalytics),
    [session.ticks, session.replayAnalytics],
  );
  const availability = {
    robotTruth: session.mode !== "replay",
    diffMclTruth: session.mode !== "replay",
    diffPoseTruth: session.mode !== "replay",
  } as const;

  const openSetup = useCallback(() => {
    void Promise.resolve(session.stop()).then(() => {
      session.resetForSetup();
      waypointEditor.clearWaypoints();
      setInitialPose({ x: 0, y: 0, heading_deg: 0 });
      setShowSetup(true);
    });
  }, [session, waypointEditor]);

  return (
    <main
      className={`relative flex min-h-screen flex-col bg-zinc-950 p-4 text-zinc-100 ${
        !showSetup && session.mode === "replay" ? "pb-[min(420px,50vh)]" : ""
      }`}
    >
      {showSetup ? (
        <div
          className="fixed inset-0 z-50 flex flex-col overflow-hidden bg-zinc-950/98 backdrop-blur-sm"
          role="dialog"
          aria-modal="true"
          aria-labelledby="setup-title"
        >
          <div className="shrink-0 border-b border-zinc-800 px-4 py-4 sm:px-6">
            <h1 id="setup-title" className="text-xl font-semibold">
              MCL Unified Home
            </h1>
            <p className="mt-1 text-sm text-zinc-400">
              Configure session, then continue to the field and metrics.
            </p>
          </div>
          <div className="min-h-0 flex-1 overflow-y-auto px-4 pb-8 pt-4 sm:px-6">
            <div className="mx-auto max-w-5xl space-y-4">
              <SessionSetup
                mode={session.mode}
                setMode={session.setMode}
                mapName={mapName}
                setMapName={setMapName}
                maps={maps}
                algorithm={algorithm}
                setAlgorithm={setAlgorithm}
                failureConfig={failureConfig}
                onFailureConfigChange={setFailureConfig}
                replayFiles={session.replayFiles}
                replayFocusEpoch={replayFocusEpoch}
                onRefreshReplayList={() => void session.refreshReplayList()}
                onLoadReplayFile={(file, seekTick) => {
                  const p =
                    seekTick !== undefined
                      ? session.loadReplayAndSeek(file, seekTick)
                      : session.loadReplay(file);
                  void p.then(() => setShowSetup(false));
                }}
                onUploadReplay={(file) => {
                  void session.loadReplayUpload(file).then(() => setShowSetup(false));
                }}
                isRunning={session.isRunning}
                initialPose={initialPose}
                onInitialPoseChange={setInitialPose}
                onStart={() =>
                  void session
                    .start({
                      mode: session.mode,
                      mapName,
                      algorithm,
                      obstacles: selectedMap?.obstacles ?? [],
                      failureConfig,
                      initialState: initialPose,
                      waypoints: waypointEditor.waypoints,
                    })
                    .then(async () => {
                      setShowSetup(false);
                      // For route mode: submit waypoints to backend navigator
                      if (session.mode === "route" && waypointEditor.waypoints.length > 0 && session.sessionId) {
                        await api.navigate(session.sessionId, waypointEditor.waypoints);
                      }
                    })
                }
                onStop={() =>
                  void session.stop().then(() => {
                    setShowSetup(true);
                  })
                }
              />
            </div>
          </div>
        </div>
      ) : null}

      {!showSetup ? (
        <>
          <div className="mb-3 flex flex-col gap-3 lg:flex-row lg:items-start lg:justify-between">
            <h1 className="text-xl font-semibold">MCL Unified Home</h1>
            <div className="flex w-full min-w-0 flex-col items-stretch gap-2 lg:max-w-[min(100%,52rem)] lg:items-end">
              <div className="flex flex-wrap items-center justify-end gap-2">
                <span className="rounded border border-zinc-700 px-2 py-1.5 text-xs text-zinc-400">
                  {status}
                </span>
                <button
                  type="button"
                  className="rounded bg-zinc-800 px-3 py-1 text-sm"
                  onClick={openSetup}
                >
                  Reconfigure Session
                </button>
                {session.mode === "replay" ? (
                  <button
                    type="button"
                    className="rounded bg-sky-800 px-3 py-1 text-sm hover:bg-sky-700"
                    onClick={() => {
                      setReplayFocusEpoch((e) => e + 1);
                      setShowSetup(true);
                    }}
                  >
                    Browse replays
                  </button>
                ) : null}
                {session.isRunning && session.mode !== "replay" ? (
                  <button
                    type="button"
                    className="rounded bg-red-700 px-3 py-1 text-sm"
                    onClick={openSetup}
                  >
                    End Session
                  </button>
                ) : null}
              </div>
            </div>
          </div>

          <div className="mt-1 grid min-h-0 flex-1 grid-cols-1 gap-4 lg:grid-cols-[minmax(260px,300px)_minmax(0,1fr)_minmax(260px,300px)] lg:items-start">
            <aside className="order-2 flex min-h-0 w-full flex-col gap-3 overflow-y-auto lg:order-1 lg:max-h-[78vh] lg:pr-1">
              <Legend
                flags={overlayFlags}
                availability={availability}
                onToggle={(key, value) => setOverlayFlags((prev) => ({ ...prev, [key]: value }))}
                tick={session.currentTick}
              />
            </aside>

            <div className="order-1 min-h-0 min-w-0 lg:order-2">
              <FieldCanvas
                tick={session.currentTick}
                stage="post_resample"
                obstacles={selectedMap?.obstacles ?? []}
                navTarget={session.currentTick?.nav_status?.status === "navigating"
                  ? (waypointEditor.waypoints[session.currentTick.nav_status.current_waypoint_index] ?? null)
                  : null}
                heatmapMode={overlayFlags.heatmap ? "density" : "off"}
                overlayFlags={overlayFlags}
                onFieldClick={(x, y) => {
                  if (!session.isRunning || session.mode === "replay") return;
                  const fieldHalf = 72;
                  const clampedX = Math.max(-fieldHalf, Math.min(fieldHalf, x));
                  const clampedY = Math.max(-fieldHalf, Math.min(fieldHalf, y));
                  if (session.mode === "route") {
                    waypointEditor.addWaypoint(clampedX, clampedY);
                  } else if (session.sessionId) {
                    void api.navigate(session.sessionId, [{ x: clampedX, y: clampedY }]);
                  }
                }}
                waypoints={waypointEditor.waypoints}
                currentWaypointIdx={session.currentTick?.nav_status?.current_waypoint_index ?? 0}
                className="h-[78vh] w-full rounded border border-zinc-700"
              />
            </div>

            <aside className="order-3 flex min-h-0 w-full flex-col gap-3 overflow-y-auto lg:max-h-[78vh] lg:pl-1">
              <GateStatus gate={session.currentTick?.gate_decision} />
              <MetricsPanel tick={session.currentTick} />
              {session.mode === "replay" && session.replayAnalytics ? (
                <AnalyticsPanel analytics={session.replayAnalytics} />
              ) : null}
            </aside>
          </div>

          {session.mode === "replay" ? (
            <div
              className="fixed bottom-0 left-0 right-0 z-30 border-t border-zinc-700 bg-zinc-950 shadow-[0_-8px_24px_rgba(0,0,0,0.45)]"
              role="region"
              aria-label="Replay playback and time series"
            >
              <div className="mx-auto max-w-[100vw] px-3 pt-2">
                <TransportControls
                  compact
                  variant="bottomBar"
                  cursor={session.cursor}
                  totalTicks={session.ticks.length}
                  onSeek={session.setCursor}
                  onStep={(d) =>
                    session.setCursor((prev) =>
                      Math.max(0, Math.min(session.ticks.length - 1, prev + d)),
                    )
                  }
                  annotations={scrubberAnnotations}
                />
              </div>
              <TimeSeriesPanel
                ticks={session.ticks}
                cursor={session.cursor}
                onSeek={session.setCursor}
              />
            </div>
          ) : null}
        </>
      ) : null}
    </main>
  );
}
