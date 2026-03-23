"use client";

import { useCallback, useMemo, useState } from "react";

import { api } from "@/lib/api";
import type { AABB, RouteDefinition, Waypoint } from "@/lib/types";

export type EditMode = "waypoint" | "obstacle" | "robot";

function defaultRoute(): RouteDefinition {
  return {
    name: "custom_route",
    description: "",
    waypoints: [{ x: 0, y: 0 }],
    obstacles: [],
    initial_heading_deg: 0,
    follower: {
      linear_velocity: 26,
      waypoint_tolerance: 3,
      max_angular_velocity_deg: 220,
      turn_in_place_threshold_deg: 70,
    },
    failure_seed: 42,
    failure_config: {
      sensor_dead_prob: 0.01,
      sensor_stuck_prob: 0.008,
      odom_spike_prob: 0.008,
      heading_bias_prob: 0.006,
      spurious_reflection_prob: 0.01,
      kidnap_prob: 0.001,
      min_duration_ticks: 6,
      max_duration_ticks: 18,
      odom_spike_range: [2, 4],
      heading_bias_range: [-12, 12],
      spurious_reflection_range: [2, 10],
    },
    kidnap_events: [],
    max_ticks: 600,
  };
}

export function useRouteEditor() {
  const [route, setRoute] = useState<RouteDefinition>(defaultRoute);
  const [editMode, setEditMode] = useState<EditMode>("waypoint");
  const [robotStart, setRobotStart] = useState<{ x: number; y: number; heading_deg: number }>({
    x: 0,
    y: 0,
    heading_deg: 0,
  });
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const setWaypoints = useCallback((waypoints: Waypoint[]) => {
    setRoute((prev) => ({ ...prev, waypoints }));
  }, []);

  const setObstacles = useCallback((obstacles: AABB[]) => {
    setRoute((prev) => ({ ...prev, obstacles }));
  }, []);

  const patchRoute = useCallback((patch: Partial<RouteDefinition>) => {
    setRoute((prev) => ({ ...prev, ...patch }));
  }, []);

  const loadPreset = useCallback(async (file: string) => {
    setBusy(true);
    setError(null);
    try {
      const loaded = await api.getRoute(file);
      setRoute(loaded);
      setRobotStart((prev) => ({
        ...prev,
        heading_deg: loaded.initial_heading_deg ?? prev.heading_deg,
      }));
    } catch (e) {
      setError(e instanceof Error ? e.message : "Failed to load preset");
    } finally {
      setBusy(false);
    }
  }, []);

  const toRouteJSON = useCallback((): RouteDefinition => {
    return {
      ...route,
      initial_heading_deg: robotStart.heading_deg,
      waypoints:
        route.waypoints.length > 0 ? route.waypoints : [{ x: robotStart.x, y: robotStart.y }],
    };
  }, [robotStart.heading_deg, robotStart.x, robotStart.y, route]);

  const saveRoute = useCallback(async () => {
    setBusy(true);
    setError(null);
    try {
      await api.saveRoute(toRouteJSON());
    } catch (e) {
      setError(e instanceof Error ? e.message : "Failed to save route");
      throw e;
    } finally {
      setBusy(false);
    }
  }, [toRouteJSON]);

  const runRoute = useCallback(async () => {
    setBusy(true);
    setError(null);
    try {
      const payload = toRouteJSON();
      const saved = await api.saveRoute(payload);
      return await api.runRoute(saved.file, payload.failure_seed);
    } catch (e) {
      setError(e instanceof Error ? e.message : "Failed to run route");
      throw e;
    } finally {
      setBusy(false);
    }
  }, [toRouteJSON]);

  return useMemo(
    () => ({
      route,
      editMode,
      setEditMode,
      robotStart,
      setRobotStart,
      setWaypoints,
      setObstacles,
      patchRoute,
      loadPreset,
      saveRoute,
      runRoute,
      toRouteJSON,
      error,
      busy,
    }),
    [
      busy,
      editMode,
      error,
      loadPreset,
      patchRoute,
      robotStart,
      route,
      runRoute,
      saveRoute,
      setObstacles,
      setWaypoints,
      toRouteJSON,
    ],
  );
}

