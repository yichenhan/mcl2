import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";

import { Legend } from "@/components/Legend";
import { getDefaultOverlayFlags } from "@/lib/overlays";

describe("Legend", () => {
  it("toggles available flags", () => {
    const onToggle = vi.fn();
    const flags = getDefaultOverlayFlags();
    render(
      <Legend
        flags={flags}
        availability={{}}
        onToggle={onToggle}
        tick={null}
      />,
    );
    const checkbox = screen.getByLabelText("Particles");
    fireEvent.click(checkbox);
    expect(onToggle).toHaveBeenCalledWith("particles", false);
  });

  it("disables unavailable flags", () => {
    const onToggle = vi.fn();
    const flags = getDefaultOverlayFlags();
    render(
      <Legend
        flags={flags}
        availability={{ robotTruth: false }}
        onToggle={onToggle}
        tick={null}
      />,
    );
    const checkbox = screen.getByLabelText("Robot True Location") as HTMLInputElement;
    expect(checkbox.disabled).toBe(true);
  });
});
