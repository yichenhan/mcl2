import { describe, expect, it } from "vitest";

import { buildScrubberAnnotations } from "@/lib/replayAnnotations";
import { DEFAULT_SESSION_ANALYTICS } from "@/lib/sessionAnalyticsDefaults";
import { toTickState } from "@/lib/tick";
import type { SessionAnalytics } from "@/lib/types";

const gd = (accepted: boolean) =>
  ({
    accepted,
    failed_centroid_jump: false,
    failed_r90: false,
    failed_passability: false,
    failed_residual: false,
    jump_in: 0,
    radius_90_in: 0,
    spread_in: 0,
    reason: "",
  }) as const;

describe("buildScrubberAnnotations", () => {
  it("adds error_peak from analytics", () => {
    const ticks = [toTickState({ tick: 0, gate_decision: gd(true) })];
    const analytics: SessionAnalytics = {
      ...DEFAULT_SESSION_ANALYTICS,
      peak_accepted_error_tick: 0,
      peak_accepted_error: 9,
    };
    const a = buildScrubberAnnotations(ticks, analytics);
    expect(a.some((x) => x.type === "error_peak")).toBe(true);
  });

  it("flags false accepts when accepted_error exceeds threshold", () => {
    const ticks = [
      toTickState({
        tick: 0,
        gate_decision: gd(true),
        accepted_error: 10,
      }),
    ];
    const a = buildScrubberAnnotations(ticks, null, 3);
    expect(a.some((x) => x.type === "false_accept")).toBe(true);
  });

  it("adds rejection streak span for long rejects", () => {
    const ticks = Array.from({ length: 8 }, (_, i) =>
      toTickState({
        tick: i,
        gate_decision: gd(i < 6 ? false : true),
      }),
    );
    const a = buildScrubberAnnotations(ticks, null);
    expect(a.some((x) => x.type === "rejection_streak" && x.endTick !== undefined)).toBe(true);
  });
});
