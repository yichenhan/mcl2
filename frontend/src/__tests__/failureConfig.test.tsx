import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";

import { FailureConfig } from "@/components/FailureConfig";
import type { FailureConfig as FailureConfigType } from "@/lib/types";

describe("FailureConfig", () => {
  it("emits updated backend-shaped config with probability and per-failure max ticks", () => {
    const onChange = vi.fn();
    const value: FailureConfigType = {
      sensor_dead_prob: 0.02,
      sensor_stuck_prob: 0.01,
      odom_spike_prob: 0.005,
      heading_bias_prob: 0.005,
      spurious_reflection_prob: 0.01,
      kidnap_prob: 0,
      min_duration_ticks: 5,
      max_duration_ticks: 30,
      sensor_dead_max_duration_ticks: 30,
      sensor_stuck_max_duration_ticks: 5,
      odom_spike_max_duration_ticks: 15,
      heading_bias_max_duration_ticks: 40,
      spurious_reflection_max_duration_ticks: 1,
      odom_spike_range: [1.5, 3.0],
      heading_bias_range: [-10.0, 10.0],
      spurious_reflection_range: [2.0, 12.0],
    };

    render(<FailureConfig value={value} onChange={onChange} />);
    const freq = screen.getByLabelText(/Sensor dead frequency percent/i);
    fireEvent.change(freq, { target: { value: "33.3" } });
    expect(onChange).toHaveBeenCalled();
    const next = onChange.mock.calls[0][0] as FailureConfigType;
    expect(next.sensor_dead_prob).toBeCloseTo(0.333);
    expect(next.sensor_dead_max_duration_ticks).toBe(30);
    expect(next.min_duration_ticks).toBe(5);
  });
});
