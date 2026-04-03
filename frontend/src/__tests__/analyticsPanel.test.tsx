import { render, screen } from "@testing-library/react";
import { describe, expect, it } from "vitest";

import { AnalyticsPanel } from "@/components/AnalyticsPanel";
import { DEFAULT_SESSION_ANALYTICS } from "@/lib/sessionAnalyticsDefaults";

describe("AnalyticsPanel", () => {
  it("renders overall grade and summary", () => {
    render(
      <AnalyticsPanel
        analytics={{
          ...DEFAULT_SESSION_ANALYTICS,
          false_accept_count: 2,
          peak_accepted_error: 20,
        }}
      />,
    );
    expect(screen.getByText("Session analytics")).toBeInTheDocument();
    expect(screen.getByText(/Overall/i)).toBeInTheDocument();
    expect(screen.getByText(/2 false accept\(s\)/)).toBeInTheDocument();
  });
});
