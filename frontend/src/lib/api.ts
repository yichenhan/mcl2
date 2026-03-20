import type {
  Action,
  RouteDefinition,
  RouteRunResult,
  SessionConfigResponse,
  SessionStartRequest,
  SessionStartResponse,
  TickState,
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

  getSessionState(sessionId: string): Promise<TickState[]> {
    return request<TickState[]>(`/api/session/${sessionId}/state`);
  },

  getSessionConfig(sessionId: string): Promise<SessionConfigResponse> {
    return request<SessionConfigResponse>(`/api/session/${sessionId}/config`);
  },

  closeSession(
    sessionId: string,
  ): Promise<{ saved: boolean; ticks: number; path: string }> {
    return request<{ saved: boolean; ticks: number; path: string }>(
      `/api/session/${sessionId}`,
      { method: "DELETE" },
    );
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

  listRoutes(): Promise<string[]> {
    return request<string[]>("/api/routes");
  },

  getRoute(name: string): Promise<RouteDefinition> {
    return request<RouteDefinition>(`/api/routes/${encodeURIComponent(name)}`);
  },

  saveRoute(route: RouteDefinition): Promise<{ saved: boolean; file: string }> {
    return request<{ saved: boolean; file: string }>("/api/routes", {
      method: "POST",
      body: JSON.stringify(route),
    });
  },

  runRoute(name: string, seed?: number): Promise<{ replay_file: string; result: RouteRunResult }> {
    const query = seed === undefined ? "" : `?seed=${seed}`;
    return request<{ replay_file: string; result: RouteRunResult }>(
      `/api/routes/${encodeURIComponent(name)}/run${query}`,
      { method: "POST" },
    );
  },
};
