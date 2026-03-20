import type { ParsedFailure } from "@/lib/types";

const SENSOR_NAMES = ["Left", "Right", "Front", "Back"];

function getTokenNumber(input: string, key: string): number | undefined {
  const m = input.match(new RegExp(`${key}=(-?\\d+(?:\\.\\d+)?)`));
  if (!m) return undefined;
  return Number(m[1]);
}

export function parseFailureString(raw: string): ParsedFailure {
  const s = raw.trim();
  if (!s) {
    return {
      type: "unknown",
      label: "Unknown",
      severity: "info",
      color: "#71717a",
    };
  }

  const sensorIdx = getTokenNumber(s, "sensor");
  const param = getTokenNumber(s, "param");
  const targetX = getTokenNumber(s, "target_x");
  const targetY = getTokenNumber(s, "target_y");
  const sensorName =
    sensorIdx !== undefined && sensorIdx >= 0 && sensorIdx < SENSOR_NAMES.length
      ? SENSOR_NAMES[sensorIdx]
      : undefined;

  if (s.includes("sensor_dead")) {
    return {
      type: "sensor_dead",
      sensor: sensorIdx,
      sensorName,
      param,
      label: `S${sensorIdx ?? "?"} DEAD`,
      severity: "warning",
      color: "#f97316",
    };
  }
  if (s.includes("sensor_stuck")) {
    return {
      type: "sensor_stuck",
      sensor: sensorIdx,
      sensorName,
      param,
      label: `S${sensorIdx ?? "?"} STUCK`,
      severity: "warning",
      color: "#fb923c",
    };
  }
  if (s.includes("odom_spike")) {
    return {
      type: "odom_spike",
      param,
      label: `ODOM SPIKE x${(param ?? 0).toFixed(2)}`,
      severity: "warning",
      color: "#eab308",
    };
  }
  if (s.includes("heading_bias")) {
    return {
      type: "heading_bias",
      param,
      label: `HEADING BIAS ${(param ?? 0) >= 0 ? "+" : ""}${(param ?? 0).toFixed(1)} deg`,
      severity: "warning",
      color: "#a855f7",
    };
  }
  if (s.includes("spurious_reflection")) {
    return {
      type: "spurious_reflection",
      sensor: sensorIdx,
      sensorName,
      param,
      label: `GHOST ECHO S${sensorIdx ?? "?"} @ ${(param ?? 0).toFixed(1)}in`,
      severity: "info",
      color: "#ec4899",
    };
  }
  if (s.includes("kidnap")) {
    const hasTarget = targetX !== undefined && targetY !== undefined;
    return {
      type: "kidnap",
      targetX,
      targetY,
      label: hasTarget
        ? `KIDNAPPED to (${targetX!.toFixed(1)}, ${targetY!.toFixed(1)})`
        : "KIDNAPPED",
      severity: "critical",
      color: "#ef4444",
    };
  }

  return {
    type: "unknown",
    sensor: sensorIdx,
    param,
    label: raw,
    severity: "info",
    color: "#71717a",
  };
}

export function parseFailures(rawFailures: string[]): ParsedFailure[] {
  const parsed = rawFailures.map(parseFailureString);
  const deadSensors = parsed.filter((p) => p.type === "sensor_dead").length;
  if (deadSensors >= 4) {
    return [
      {
        type: "sensor_dead",
        label: "ALL SENSORS BLIND",
        severity: "critical",
        color: "#dc2626",
      },
      ...parsed.filter((p) => p.type !== "sensor_dead"),
    ];
  }
  return parsed;
}

