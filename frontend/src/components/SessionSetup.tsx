"use client";

import { useEffect, useRef, useState } from "react";

import { FailureConfig } from "@/components/FailureConfig";
import { FieldCanvas } from "@/components/FieldCanvas";
import { ReplayBrowser } from "@/components/ReplayBrowser";
import type {
  FailureConfig as FailureConfigType,
  MapPreset,
  OverlayFlags,
  SessionMode,
} from "@/lib/types";

const FIELD_HALF = 72;

const SETUP_PREVIEW_OVERLAYS: OverlayFlags = {
  robotTruth: false,
  rawOdom: false,
  mclEstimate: false,
  acceptedEstimate: false,
  r90Circle: false,
  particles: false,
  heatmap: false,
  sensorReadings: false,
  sensorResiduals: false,
  diffMclPose: false,
  diffMclTruth: false,
  diffPoseTruth: false,
};

function clampToField(x: number, y: number) {
  return {
    x: Math.max(-FIELD_HALF, Math.min(FIELD_HALF, x)),
    y: Math.max(-FIELD_HALF, Math.min(FIELD_HALF, y)),
  };
}

type SetupStep =
  | "mode"
  | "sim-field"
  | "sim-failures"
  | "replay-how"
  | "replay-server"
  | "replay-upload";

interface Props {
  mode: SessionMode;
  setMode: (mode: SessionMode) => void;
  mapName: string;
  setMapName: (value: string) => void;
  maps: MapPreset[];
  algorithm: "arc_nav" | "turn_and_go";
  setAlgorithm: (value: "arc_nav" | "turn_and_go") => void;
  failureConfig: FailureConfigType;
  onFailureConfigChange: (next: FailureConfigType) => void;
  replayFiles: string[];
  /** `seekTick` jumps to that index after load (server replays with analytics). */
  onLoadReplayFile: (file: string, seekTick?: number) => void;
  onUploadReplay: (file: File) => void;
  /** When incremented, switches to replay mode and opens the server replay step. */
  replayFocusEpoch?: number;
  onRefreshReplayList?: () => void;
  isRunning: boolean;
  onStart: () => void;
  onStop: () => void;
  initialPose: { x: number; y: number; heading_deg: number };
  onInitialPoseChange: (next: { x: number; y: number; heading_deg: number }) => void;
}

function stepLabel(step: SetupStep): string {
  switch (step) {
    case "mode":
      return "What do you want to do?";
    case "sim-field":
      return "Field & algorithm";
    case "sim-failures":
      return "Noise & failures";
    case "replay-how":
      return "Load replay";
    case "replay-server":
      return "Server replay";
    case "replay-upload":
      return "Upload replay";
    default:
      return "";
  }
}

export function SessionSetup({
  mode,
  setMode,
  mapName,
  setMapName,
  maps,
  algorithm,
  setAlgorithm,
  failureConfig,
  onFailureConfigChange,
  replayFiles,
  onLoadReplayFile,
  onUploadReplay,
  replayFocusEpoch = 0,
  onRefreshReplayList,
  isRunning,
  onStart,
  onStop,
  initialPose,
  onInitialPoseChange,
}: Props) {
  const [step, setStep] = useState<SetupStep>("mode");
  const lastReplayFocusRef = useRef(0);

  useEffect(() => {
    if (replayFocusEpoch > 0 && replayFocusEpoch !== lastReplayFocusRef.current) {
      lastReplayFocusRef.current = replayFocusEpoch;
      setMode("replay");
      setStep("replay-server");
    }
  }, [replayFocusEpoch, setMode]);

  const goBack = () => {
    if (step === "sim-field" || step === "replay-how") setStep("mode");
    else if (step === "sim-failures") setStep("sim-field");
    else if (step === "replay-server" || step === "replay-upload") setStep("replay-how");
  };

  const selectMode = (next: SessionMode) => {
    setMode(next);
    if (next === "replay") {
      setStep("replay-how");
    } else {
      setStep("sim-field");
    }
  };

  const simStepIndex = step === "sim-field" ? 1 : step === "sim-failures" ? 2 : 0;

  return (
    <div className="space-y-6">
      {/* Progress — simulation path */}
      {(step === "sim-field" || step === "sim-failures") && (
        <div className="flex items-center gap-2 text-xs text-zinc-500">
          <span className="text-zinc-400">Live / Route</span>
          <span aria-hidden className="text-zinc-600">
            →
          </span>
          <span className={simStepIndex >= 1 ? "text-emerald-400/90" : ""}>1. Field & algorithm</span>
          <span className="text-zinc-600">·</span>
          <span className={simStepIndex >= 2 ? "text-emerald-400/90" : ""}>2. Failures</span>
        </div>
      )}

      {/* Progress — replay path */}
      {(step === "replay-how" || step === "replay-server" || step === "replay-upload") && (
        <div className="flex items-center gap-2 text-xs text-zinc-500">
          <span className="text-zinc-400">Replay</span>
          <span aria-hidden className="text-zinc-600">
            →
          </span>
          <span className={step !== "replay-how" ? "text-emerald-400/90" : ""}>1. Source</span>
          <span className="text-zinc-600">·</span>
          <span className={step === "replay-server" || step === "replay-upload" ? "text-emerald-400/90" : ""}>
            2. Choose file
          </span>
        </div>
      )}

      <header>
        <h2 className="text-lg font-semibold text-zinc-100">{stepLabel(step)}</h2>
        <p className="mt-1 text-sm text-zinc-500">
          {step === "mode" && "Pick a path. You can go back and change this anytime before you start."}
          {step === "sim-field" &&
            "Choose the field layout and how the robot steers. Click the preview to set the robot start pose; adjust heading with the number field."}
          {step === "sim-failures" &&
            "Tune random sensor and motion faults. Then start the simulator when you are ready."}
          {step === "replay-how" && "Replays can come from this machine’s session folder or from a file you export."}
          {step === "replay-server" && "Select a replay recorded on the server, then load it into the viewer."}
          {step === "replay-upload" && "Use a .txt or .json export from a previous run."}
        </p>
      </header>

      {/* Step: mode */}
      {step === "mode" && (
        <div className="grid gap-4 sm:grid-cols-3">
          <button
            type="button"
            disabled={isRunning}
            onClick={() => selectMode("live")}
            className="group flex flex-col items-start rounded-xl border border-zinc-700 bg-zinc-900/60 p-5 text-left transition hover:border-emerald-600/50 hover:bg-zinc-900 disabled:opacity-50"
          >
            <span className="text-base font-semibold text-zinc-100">Live</span>
            <span className="mt-2 text-sm text-zinc-400">
              Run MCL in real time. Click the field to set targets after the session starts.
            </span>
          </button>
          <button
            type="button"
            disabled={isRunning}
            onClick={() => selectMode("route")}
            className="group flex flex-col items-start rounded-xl border border-zinc-700 bg-zinc-900/60 p-5 text-left transition hover:border-emerald-600/50 hover:bg-zinc-900 disabled:opacity-50"
          >
            <span className="text-base font-semibold text-zinc-100">Route</span>
            <span className="mt-2 text-sm text-zinc-400">
              Plan waypoints on the field. The robot follows your path after the session starts.
            </span>
          </button>
          <button
            type="button"
            disabled={isRunning}
            onClick={() => selectMode("replay")}
            className="group flex flex-col items-start rounded-xl border border-zinc-700 bg-zinc-900/60 p-5 text-left transition hover:border-emerald-600/50 hover:bg-zinc-900 disabled:opacity-50"
          >
            <span className="text-base font-semibold text-zinc-100">Replay</span>
            <span className="mt-2 text-sm text-zinc-400">
              Step through saved ticks from the server or a file—no live simulation.
            </span>
          </button>
        </div>
      )}

      {/* Step: map + algorithm */}
      {step === "sim-field" && (
        <div className="space-y-4 rounded-xl border border-zinc-800 bg-zinc-900/40 p-5">
          <div className="grid gap-4 sm:grid-cols-2">
            <label className="block text-xs font-medium text-zinc-300">
              Map
              <select
                className="mt-2 w-full rounded-lg border border-zinc-600 bg-zinc-950 px-3 py-2 text-sm"
                value={mapName}
                disabled={isRunning}
                onChange={(e) => setMapName(e.target.value)}
              >
                {maps.map((m) => (
                  <option key={m.name} value={m.name}>
                    {m.name}
                  </option>
                ))}
              </select>
            </label>
            <label className="block text-xs font-medium text-zinc-300">
              Algorithm
              <select
                className="mt-2 w-full rounded-lg border border-zinc-600 bg-zinc-950 px-3 py-2 text-sm"
                value={algorithm}
                disabled={isRunning}
                onChange={(e) => setAlgorithm(e.target.value as "arc_nav" | "turn_and_go")}
              >
                <option value="arc_nav">Arc Nav</option>
                <option value="turn_and_go">Turn and Go</option>
              </select>
            </label>
          </div>

          <div className="space-y-2">
            <p className="text-xs font-medium text-zinc-400">Start position (click to place)</p>
            <FieldCanvas
              tick={null}
              stage="post_resample"
              obstacles={maps.find((m) => m.name === mapName)?.obstacles ?? []}
              placementPose={initialPose}
              onFieldClick={(x, y) => {
                const c = clampToField(x, y);
                onInitialPoseChange({ ...initialPose, x: c.x, y: c.y });
              }}
              heatmapMode="off"
              overlayFlags={SETUP_PREVIEW_OVERLAYS}
              className="h-[min(42vh,380px)] w-full max-w-3xl cursor-crosshair rounded border border-zinc-700"
            />
            <div className="flex flex-wrap items-end gap-4">
              <label className="text-xs text-zinc-300">
                Heading (deg)
                <input
                  className="mt-1 block w-28 rounded-md border border-zinc-600 bg-zinc-950 px-2 py-1.5 text-sm tabular-nums"
                  type="number"
                  step={1}
                  value={initialPose.heading_deg}
                  disabled={isRunning}
                  onChange={(e) => {
                    const v = Number(e.target.value);
                    if (Number.isNaN(v)) return;
                    onInitialPoseChange({ ...initialPose, heading_deg: v });
                  }}
                />
              </label>
              <div className="text-xs text-zinc-500">
                Position: ({initialPose.x.toFixed(1)}, {initialPose.y.toFixed(1)}) in
              </div>
            </div>
          </div>

          <div className="flex flex-wrap gap-2 pt-2">
            <button
              type="button"
              className="rounded-lg border border-zinc-600 px-4 py-2 text-sm text-zinc-200 hover:bg-zinc-800"
              onClick={goBack}
            >
              Back
            </button>
            <button
              type="button"
              className="rounded-lg bg-emerald-600 px-4 py-2 text-sm font-medium text-white hover:bg-emerald-500"
              onClick={() => setStep("sim-failures")}
            >
              Continue to failures
            </button>
          </div>
        </div>
      )}

      {/* Step: failures + start */}
      {step === "sim-failures" && (
        <div className="space-y-4">
          <FailureConfig value={failureConfig} onChange={onFailureConfigChange} disabled={isRunning} />
          <div className="flex flex-wrap gap-2">
            <button
              type="button"
              className="rounded-lg border border-zinc-600 px-4 py-2 text-sm text-zinc-200 hover:bg-zinc-800"
              onClick={() => setStep("sim-field")}
            >
              Back
            </button>
            <button
              type="button"
              className="rounded-lg bg-emerald-600 px-4 py-2 text-sm font-medium text-white hover:bg-emerald-500 disabled:opacity-50"
              disabled={isRunning}
              onClick={onStart}
            >
              Start session
            </button>
            <button
              type="button"
              className="rounded-lg border border-red-900/80 bg-red-950/40 px-4 py-2 text-sm text-red-200 hover:bg-red-950/60 disabled:opacity-40"
              disabled={!isRunning}
              onClick={onStop}
            >
              Stop
            </button>
          </div>
        </div>
      )}

      {/* Step: replay source choice */}
      {step === "replay-how" && (
        <div className="grid gap-4 sm:grid-cols-2">
          <button
            type="button"
            className="flex flex-col items-start rounded-xl border border-zinc-700 bg-zinc-900/60 p-6 text-left transition hover:border-sky-600/50 hover:bg-zinc-900"
            onClick={() => setStep("replay-server")}
          >
            <span className="text-base font-semibold text-zinc-100">Server replay</span>
            <span className="mt-2 text-sm text-zinc-400">
              Pick a file already on the API host (session recordings folder).
            </span>
          </button>
          <button
            type="button"
            className="flex flex-col items-start rounded-xl border border-zinc-700 bg-zinc-900/60 p-6 text-left transition hover:border-sky-600/50 hover:bg-zinc-900"
            onClick={() => setStep("replay-upload")}
          >
            <span className="text-base font-semibold text-zinc-100">Upload your own</span>
            <span className="mt-2 text-sm text-zinc-400">
              Bring a .txt or .json replay from disk—exported from another run or tool.
            </span>
          </button>
          <div className="sm:col-span-2">
            <button
              type="button"
              className="rounded-lg border border-zinc-600 px-4 py-2 text-sm text-zinc-200 hover:bg-zinc-800"
              onClick={goBack}
            >
              Back
            </button>
          </div>
        </div>
      )}

      {/* Step: server list */}
      {step === "replay-server" && (
        <div className="space-y-4 rounded-xl border border-zinc-800 bg-zinc-900/40 p-5">
          {replayFiles.length === 0 ? (
            <p className="text-sm text-amber-200/80">No server replays found. Try uploading a file instead.</p>
          ) : (
            <ReplayBrowser
              files={replayFiles}
              onRefresh={onRefreshReplayList}
              onLoadReplay={(file, peakTick) => onLoadReplayFile(file, peakTick)}
            />
          )}
          <div className="flex flex-wrap gap-2">
            <button
              type="button"
              className="rounded-lg border border-zinc-600 px-4 py-2 text-sm text-zinc-200 hover:bg-zinc-800"
              onClick={goBack}
            >
              Back
            </button>
          </div>
        </div>
      )}

      {/* Step: upload */}
      {step === "replay-upload" && (
        <div className="space-y-4 rounded-xl border border-zinc-800 bg-zinc-900/40 p-5">
          <label className="block text-xs font-medium text-zinc-300">
            File
            <input
              className="mt-3 block w-full max-w-lg cursor-pointer rounded-lg border border-dashed border-zinc-600 bg-zinc-950 px-3 py-8 text-sm text-zinc-400 file:mr-4 file:rounded file:border-0 file:bg-zinc-700 file:px-3 file:py-1.5 file:text-sm file:text-zinc-200"
              type="file"
              accept=".txt,.json,application/json,text/plain"
              onChange={(e) => {
                const file = e.target.files?.[0];
                if (file) onUploadReplay(file);
                e.currentTarget.value = "";
              }}
            />
          </label>
          <p className="text-sm text-zinc-500">
            Choosing a file loads the replay and opens the viewer immediately.
          </p>
          <button
            type="button"
            className="rounded-lg border border-zinc-600 px-4 py-2 text-sm text-zinc-200 hover:bg-zinc-800"
            onClick={goBack}
          >
            Back
          </button>
        </div>
      )}
    </div>
  );
}
