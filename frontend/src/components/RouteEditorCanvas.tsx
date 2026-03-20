"use client";

import { useMemo, useRef, useState } from "react";

import { FieldCanvas } from "@/components/FieldCanvas";
import type { EditMode } from "@/hooks/useRouteEditor";
import type { AABB, Waypoint } from "@/lib/types";

interface Props {
  mode: EditMode;
  waypoints: Waypoint[];
  obstacles: AABB[];
  onWaypointsChange: (waypoints: Waypoint[]) => void;
  onObstaclesChange: (obstacles: AABB[]) => void;
  onRobotSet: (x: number, y: number, headingDeg?: number) => void;
}

const FIELD_HALF = 72;

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

export function RouteEditorCanvas({
  mode,
  waypoints,
  obstacles,
  onWaypointsChange,
  onObstaclesChange,
  onRobotSet,
}: Props) {
  const wrapRef = useRef<HTMLDivElement | null>(null);
  const [dragA, setDragA] = useState<Waypoint | null>(null);
  const [cursor, setCursor] = useState<Waypoint | null>(null);

  const handleCanvasClick = (e: React.MouseEvent<HTMLDivElement>) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const x = ((px / rect.width) * 2 - 1) * FIELD_HALF;
    const y = -(((py / rect.height) * 2 - 1) * FIELD_HALF);
    const p = { x: clamp(x, -FIELD_HALF, FIELD_HALF), y: clamp(y, -FIELD_HALF, FIELD_HALF) };

    if (mode === "waypoint") {
      onWaypointsChange([...waypoints, p]);
      return;
    }
    if (mode === "robot") {
      onRobotSet(p.x, p.y, 0);
      return;
    }
    // obstacle mode
    if (!dragA) {
      setDragA(p);
      return;
    }
    const min_x = Math.min(dragA.x, p.x);
    const max_x = Math.max(dragA.x, p.x);
    const min_y = Math.min(dragA.y, p.y);
    const max_y = Math.max(dragA.y, p.y);
    onObstaclesChange([...obstacles, { min_x, min_y, max_x, max_y }]);
    setDragA(null);
  };

  const hint = useMemo(() => {
    if (mode === "waypoint") return "Click to add waypoint";
    if (mode === "robot") return "Click to set robot start";
    if (dragA) return "Click second corner to finish obstacle";
    return "Click first corner to start obstacle";
  }, [dragA, mode]);

  return (
    <div
      ref={wrapRef}
      className="relative"
      onClick={handleCanvasClick}
      onMouseMove={(e) => {
        const rect = e.currentTarget.getBoundingClientRect();
        const px = e.clientX - rect.left;
        const py = e.clientY - rect.top;
        const x = ((px / rect.width) * 2 - 1) * FIELD_HALF;
        const y = -(((py / rect.height) * 2 - 1) * FIELD_HALF);
        setCursor({ x, y });
      }}
    >
      <FieldCanvas
        tick={null}
        stage="post_resample"
        obstacles={obstacles}
        waypoints={waypoints}
        className="h-[70vh] w-full cursor-crosshair rounded border border-zinc-700"
      />
      <div className="pointer-events-none absolute left-2 top-2 rounded bg-zinc-900/90 px-2 py-1 text-xs text-zinc-200">
        {hint}
      </div>
      {cursor ? (
        <div className="pointer-events-none absolute bottom-2 right-2 rounded bg-zinc-900/90 px-2 py-1 text-xs text-zinc-300">
          x={cursor.x.toFixed(1)} y={cursor.y.toFixed(1)}
        </div>
      ) : null}
      {dragA ? (
        <div className="pointer-events-none absolute bottom-9 right-2 rounded bg-amber-950/90 px-2 py-1 text-xs text-amber-200">
          anchor ({dragA.x.toFixed(1)}, {dragA.y.toFixed(1)})
        </div>
      ) : null}
    </div>
  );
}

