import { describe, expect, it, vi, beforeEach } from "vitest";

const mockFetch = vi.fn();
global.fetch = mockFetch;

beforeEach(() => {
  mockFetch.mockClear();
});

describe("api.navigate", () => {
  it("sends POST to /api/session/:id/navigate with correct body", async () => {
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ status: "navigating", total_waypoints: 2 }),
    });

    const { api } = await import("@/lib/api");
    const result = await api.navigate("s1", [{ x: 10, y: 20 }, { x: 30, y: 40 }]);

    expect(mockFetch).toHaveBeenCalledOnce();
    const [url, init] = mockFetch.mock.calls[0] as [string, RequestInit];
    expect(url).toContain("/api/session/s1/navigate");
    expect(init.method).toBe("POST");
    const body = JSON.parse(init.body as string) as Record<string, unknown>;
    expect(body.waypoints).toHaveLength(2);
    expect((body.waypoints as {x: number, y: number}[])[0].x).toBe(10);
    expect(result.status).toBe("navigating");
    expect(result.total_waypoints).toBe(2);
  });
});

describe("api.tickAuto", () => {
  it("sends POST /tick with empty body {}", async () => {
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ tick: 5 }),
    });

    const { api } = await import("@/lib/api");
    await api.tickAuto("s1");

    expect(mockFetch).toHaveBeenCalledOnce();
    const [url, init] = mockFetch.mock.calls[0] as [string, RequestInit];
    expect(url).toContain("/api/session/s1/tick");
    expect(init.method).toBe("POST");
    expect(init.body).toBe("{}");
  });
});

describe("api.cancelNav", () => {
  it("sends DELETE to /api/session/:id/navigate", async () => {
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ cancelled: true, waypoints_remaining: 3 }),
    });

    const { api } = await import("@/lib/api");
    const result = await api.cancelNav("s1");

    expect(mockFetch).toHaveBeenCalledOnce();
    const [url, init] = mockFetch.mock.calls[0] as [string, RequestInit];
    expect(url).toContain("/api/session/s1/navigate");
    expect(init.method).toBe("DELETE");
    expect(result.cancelled).toBe(true);
    expect(result.waypoints_remaining).toBe(3);
  });
});

describe("TickState chassis_pose parsing", () => {
  it("parses chassis_pose when present in JSON", () => {
    const raw = {
      tick: 1,
      ground_truth: { x: 0, y: 0, heading_deg: 0 },
      observed_readings: [100, 200, 300, 400],
      observed_heading: 45,
      active_failures: [],
      post_predict: { estimate: { x: 0, y: 0 }, n_eff: 0 },
      post_update: { estimate: { x: 0, y: 0 }, n_eff: 0 },
      post_resample: { estimate: { x: 0, y: 0 }, n_eff: 0 },
      mcl_error: 0,
      odom_error: 0,
      valid_sensor_count: 4,
      chassis_pose: { x: 1.5, y: 2.5, theta: 45.0 },
    };
    // chassis_pose is available
    expect(raw.chassis_pose.x).toBe(1.5);
    expect(raw.chassis_pose.y).toBe(2.5);
    expect(raw.chassis_pose.theta).toBe(45.0);
  });

  it("chassis_pose is undefined when not in JSON (old replay)", () => {
    const raw: Record<string, unknown> = {
      tick: 1,
      ground_truth: { x: 0, y: 0, heading_deg: 0 },
      accepted_estimate: { x: 7.0, y: 8.0 },
      observed_heading: 30.0,
    };
    expect(raw.chassis_pose).toBeUndefined();
  });
});
