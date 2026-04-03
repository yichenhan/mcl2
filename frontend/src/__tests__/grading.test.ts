import { describe, expect, it } from "vitest";

import { gradeField, gradeSession } from "@/lib/grading";
import { DEFAULT_SESSION_ANALYTICS } from "@/lib/sessionAnalyticsDefaults";
import type { SessionAnalytics } from "@/lib/types";

function analytics(partial: Partial<SessionAnalytics>): SessionAnalytics {
  return { ...DEFAULT_SESSION_ANALYTICS, ...partial };
}

describe("grading", () => {
  it("grades low peak error as great", () => {
    expect(gradeField("peak_accepted_error", 3)).toBe("great");
    expect(gradeField("peak_accepted_error", 20)).toBe("poor");
  });

  it("grades high gate precision as great", () => {
    expect(gradeField("gate_precision", 0.97)).toBe("great");
    expect(gradeField("gate_precision", 0.7)).toBe("bad");
  });

  it("gradeSession picks worst bucket", () => {
    const g = gradeSession(
      analytics({
        peak_accepted_error: 2,
        gate_precision: 0.5,
        false_accept_count: 0,
      }),
    );
    expect(g.overall).toBe("bad");
    expect(g.fields.some((f) => f.field === "gate_precision" && f.grade === "bad")).toBe(true);
  });

  it("gradeSession summary mentions false accepts", () => {
    const g = gradeSession(analytics({ false_accept_count: 2, false_reject_count: 0 }));
    expect(g.summary.toLowerCase()).toContain("false accept");
  });
});
