import { describe, expect, it } from "vitest";

import { getOverlayAvailability } from "@/lib/overlays";

describe("overlay availability", () => {
  it("disables truth-based overlays in replay mode", () => {
    const replay = getOverlayAvailability("replay");
    expect(replay.robotTruth).toBe(false);
    expect(replay.diffMclTruth).toBe(false);
    expect(replay.diffPoseTruth).toBe(false);
  });

  it("keeps overlays available in live mode", () => {
    const live = getOverlayAvailability("live");
    expect(live.robotTruth).toBeUndefined();
  });
});
