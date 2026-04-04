import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";

import { TransportControls } from "@/components/TransportControls";

describe("TransportControls annotations", () => {
  it("seeks when an annotation marker is clicked", () => {
    const onSeek = vi.fn();
    render(
      <TransportControls
        cursor={0}
        totalTicks={20}
        onSeek={onSeek}
        onStep={() => {}}
        compact
        annotations={[{ tick: 5, type: "error_peak", label: "Peak" }]}
      />,
    );
    const marker = screen.getByRole("button", { name: "Peak" });
    fireEvent.click(marker);
    expect(onSeek).toHaveBeenCalledWith(5);
  });
});
