"use client";

import { useCallback, useState } from "react";

import type { Waypoint } from "@/lib/types";

export function useWaypointEditor(initial: Waypoint[] = []) {
  const [waypoints, setWaypoints] = useState<Waypoint[]>(initial);

  const addWaypoint = useCallback((x: number, y: number) => {
    setWaypoints((prev) => [...prev, { x, y }]);
  }, []);

  const removeWaypoint = useCallback((index: number) => {
    setWaypoints((prev) => prev.filter((_, i) => i !== index));
  }, []);

  const clearWaypoints = useCallback(() => {
    setWaypoints([]);
  }, []);

  const loadPreset = useCallback((next: Waypoint[]) => {
    setWaypoints(next);
  }, []);

  return {
    waypoints,
    setWaypoints,
    addWaypoint,
    removeWaypoint,
    clearWaypoints,
    loadPreset,
  };
}
