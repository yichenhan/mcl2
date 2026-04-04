import type { OverlayFlags, SessionMode } from "@/lib/types";

export function getDefaultOverlayFlags(): OverlayFlags {
  return {
    robotTruth: false,
    odomPose: true,
    mclEstimate: true,
    acceptedEstimate: true,
    r90Circle: true,
    particles: true,
    heatmap: true,
    sensorReadings: true,
    sensorResiduals: true,
    diffMclPose: true,
    diffMclTruth: false,
    diffPoseTruth: false,
  };
}

export function getOverlayAvailability(mode: SessionMode): Partial<Record<keyof OverlayFlags, boolean>> {
  if (mode === "replay") {
    return {
      robotTruth: false,
      diffMclTruth: false,
      diffPoseTruth: false,
    };
  }
  return {};
}
