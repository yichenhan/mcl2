import { describe, expect, it } from "vitest";

import { DEFAULT_SESSION_ANALYTICS, parseSessionAnalytics } from "@/lib/sessionAnalyticsDefaults";

describe("parseSessionAnalytics", () => {
  it("returns null for non-object", () => {
    expect(parseSessionAnalytics(null)).toBeNull();
    expect(parseSessionAnalytics(undefined)).toBeNull();
  });

  it("fills defaults for partial object", () => {
    const a = parseSessionAnalytics({ peak_accepted_error: 12.5, false_accept_count: 2 });
    expect(a).not.toBeNull();
    expect(a!.peak_accepted_error).toBe(12.5);
    expect(a!.false_accept_count).toBe(2);
    expect(a!.gate_precision).toBe(DEFAULT_SESSION_ANALYTICS.gate_precision);
  });
});
