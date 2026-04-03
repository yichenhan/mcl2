import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import { beforeEach, describe, expect, it, vi } from "vitest";

import { ReplayBrowser } from "@/components/ReplayBrowser";
import { api } from "@/lib/api";

vi.mock("@/lib/api", () => ({
  api: {
    getReplayMeta: vi.fn(),
  },
}));

const getReplayMeta = vi.mocked(api.getReplayMeta);

describe("ReplayBrowser", () => {
  beforeEach(() => {
    getReplayMeta.mockReset();
  });

  it("orders by peak error descending and loads on row click", async () => {
    getReplayMeta.mockImplementation(async (file: string) => {
      if (file === "a.json") {
        return {
          total_ticks: 10,
          analytics: { peak_accepted_error: 5, peak_accepted_error_tick: 2, gate_precision: 0.9, gate_recall: 0.95 },
        };
      }
      return {
        total_ticks: 20,
        analytics: { peak_accepted_error: 15, peak_accepted_error_tick: 7, gate_precision: 0.8, gate_recall: 0.85 },
      };
    });

    const onLoad = vi.fn();
    render(<ReplayBrowser files={["a.json", "b.json"]} onLoadReplay={onLoad} />);

    await waitFor(() => expect(getReplayMeta).toHaveBeenCalledTimes(2));

    const bCell = await screen.findByText("b.json");
    const bRow = bCell.closest("tr");
    expect(bRow).toBeTruthy();
    const aCell = screen.getByText("a.json");
    const aRow = aCell.closest("tr");
    expect(bRow!.compareDocumentPosition(aRow!) & Node.DOCUMENT_POSITION_FOLLOWING).toBeTruthy();

    fireEvent.click(bRow!);
    expect(onLoad).toHaveBeenCalledWith("b.json", 7);
  });
});
