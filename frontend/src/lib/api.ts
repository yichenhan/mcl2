import type {
  Action,
  CancelNavResponse,
  MapPreset,
  NavResponse,
  NavStatus,
  SessionConfigResponse,
  SessionStartRequest,
  SessionStartResponse,
  TickState,
  Waypoint,
} from "@/lib/types";

const API_BASE =
  process.env.NEXT_PUBLIC_API_BASE_URL ??
  (typeof window === "undefined" ? "http://127.0.0.1:8080" : "");

async function request<T>(
  path: string,
  init?: RequestInit,
): Promise<T> {
  const res = await fetch(`${API_BASE}${path}`, {
    ...init,
    headers: {
      "Content-Type": "application/json",
      ...(init?.headers ?? {}),
    },
    cache: "no-store",
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(`${res.status} ${res.statusText}: ${text}`);
  }
  return res.json() as Promise<T>;
}

export const api = {
  startSession(body: SessionStartRequest): Promise<SessionStartResponse> {
    return request<SessionStartResponse>("/api/session/start", {
      method: "POST",
      body: JSON.stringify(body),
    });
  },

  tick(sessionId: string, action: Action): Promise<TickState> {
    return request<TickState>(`/api/session/${sessionId}/tick`, {
      method: "POST",
      body: JSON.stringify({ action }),
    });
  },

  tickContinuous(
    sessionId: string,
    linear_vel: number,
    angular_vel_deg: number,
  ): Promise<TickState> {
    return request<TickState>(`/api/session/${sessionId}/tick`, {
      method: "POST",
      body: JSON.stringify({ linear_vel, angular_vel_deg }),
    });
  },

  getSessionState(sessionId: string): Promise<TickState[]> {
    return request<TickState[]>(`/api/session/${sessionId}/state`);
  },

  getSessionConfig(sessionId: string): Promise<SessionConfigResponse> {
    return request<SessionConfigResponse>(`/api/session/${sessionId}/config`);
  },

  closeSession(
    sessionId: string,
  ): Promise<{ saved: boolean; saved_log?: boolean; ticks: number; path: string; log_path?: string }> {
    return request<{ saved: boolean; saved_log?: boolean; ticks: number; path: string; log_path?: string }>(
      `/api/session/${sessionId}`,
      { method: "DELETE" },
    );
  },

  listMaps(): Promise<MapPreset[]> {
    return request<MapPreset[]>("/api/maps");
  },

  listReplays(): Promise<string[]> {
    return request<string[]>("/api/replays");
  },

  getReplayMeta(file: string): Promise<Record<string, unknown>> {
    return request<Record<string, unknown>>(
      `/api/replays/${encodeURIComponent(file)}/meta`,
    );
  },

  getReplayTicks(file: string, from: number, to: number): Promise<TickState[]> {
    return request<TickState[]>(
      `/api/replays/${encodeURIComponent(file)}/ticks?from=${from}&to=${to}`,
    );
  },

  health(): Promise<{ status: string }> {
    return request<{ status: string }>("/api/health");
  },

  // Navigation API (Phase 5+)
  navigate(
    sessionId: string,
    waypoints: Waypoint[],
    followerConfig?: Partial<{ linear_velocity: number; waypoint_tolerance: number; max_angular_vel_deg: number; heading_gain: number; slowdown_radius: number }>,
  ): Promise<NavResponse> {
    return request<NavResponse>(`/api/session/${sessionId}/navigate`, {
      method: "POST",
      body: JSON.stringify({ waypoints, follower_config: followerConfig }),
    });
  },

  getNavStatus(sessionId: string): Promise<NavStatus> {
    return request<NavStatus>(`/api/session/${sessionId}/navigate/status`);
  },

  cancelNav(sessionId: string): Promise<CancelNavResponse> {
    return request<CancelNavResponse>(`/api/session/${sessionId}/navigate`, {
      method: "DELETE",
    });
  },

  tickAuto(sessionId: string): Promise<TickState> {
    return request<TickState>(`/api/session/${sessionId}/tick`, {
      method: "POST",
      body: JSON.stringify({}),
    });
  },
};
