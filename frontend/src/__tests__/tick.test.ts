import { describe, expect, it } from "vitest";

import { toTickState } from "@/lib/tick";

describe("toTickState", () => {
  it("fills defaults for missing fields", () => {
    const tick = toTickState({ tick: 7 });
    expect(tick.tick).toBe(7);
    expect(tick.timestamp_iso).toBe("");
    expect(tick.ground_truth).toBeUndefined();
    expect(tick.observed_readings).toEqual([-1, -1, -1, -1]);
    expect(tick.post_resample.radius_90).toBe(0);
  });

  it("fills accepted_error default to undefined", () => {
    const tick = toTickState({ tick: 0 });
    expect(tick.accepted_error).toBeUndefined();
  });

  it("preserves accepted_error from raw data", () => {
    const tick = toTickState({ tick: 0, accepted_error: 2.5 });
    expect(tick.accepted_error).toBe(2.5);
  });
});
