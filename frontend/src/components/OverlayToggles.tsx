"use client";

export interface OverlayFlags {
  odomPose: boolean;
  sensorReadings: boolean;
  mclSensorErrors: boolean;
}

interface Props {
  overlays: OverlayFlags;
  onChange: (next: OverlayFlags) => void;
}

export function OverlayToggles({ overlays, onChange }: Props) {
  return (
    <div className="rounded border border-zinc-700 p-3">
      <h3 className="mb-2 text-sm font-semibold">Overlays</h3>
      <div className="flex flex-col gap-2 text-sm">
        <label className="flex items-center gap-2">
          <input
            type="checkbox"
            checked={overlays.odomPose}
            onChange={(e) => onChange({ ...overlays, odomPose: e.target.checked })}
          />
          Odom Pose
        </label>
        <label className="flex items-center gap-2">
          <input
            type="checkbox"
            checked={overlays.sensorReadings}
            onChange={(e) => onChange({ ...overlays, sensorReadings: e.target.checked })}
          />
          Sensor Readings
        </label>
        <label className="flex items-center gap-2">
          <input
            type="checkbox"
            checked={overlays.mclSensorErrors}
            onChange={(e) => onChange({ ...overlays, mclSensorErrors: e.target.checked })}
          />
          MCL Sensor Errors
        </label>
      </div>
    </div>
  );
}
