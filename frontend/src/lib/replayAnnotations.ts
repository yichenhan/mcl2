import type { ScrubberAnnotation, SessionAnalytics, TickState } from "./types";

/**
 * Build scrubber markers from tick stream and optional session analytics.
 * `falseAcceptThreshold` defaults to analytics.false_accept_threshold when > 0, else 3 in.
 */
export function buildScrubberAnnotations(
  ticks: TickState[],
  analytics: SessionAnalytics | null,
  falseAcceptThresholdIn?: number
): ScrubberAnnotation[] {
  const out: ScrubberAnnotation[] = [];
  if (!ticks.length) return out;

  const faThresh =
    falseAcceptThresholdIn ??
    (analytics && analytics.false_accept_threshold > 0 ? analytics.false_accept_threshold : 3);

  if (analytics && analytics.cold_start_convergence_ticks > 0) {
    out.push({
      tick: analytics.cold_start_convergence_ticks,
      type: "convergence",
      label: "Cold-start convergence",
    });
  }

  if (analytics && analytics.peak_accepted_error_tick >= 0) {
    const t = Math.min(
      Math.max(0, Math.floor(analytics.peak_accepted_error_tick)),
      ticks.length - 1
    );
    out.push({
      tick: t,
      type: "error_peak",
      label: `Peak error ${analytics.peak_accepted_error.toFixed(1)} in`,
    });
  }

  for (let i = 0; i < ticks.length; i++) {
    const tk = ticks[i];
    const gd = tk.gate_decision;
    if (gd?.accepted && (tk.accepted_error ?? 0) > faThresh) {
      out.push({
        tick: i,
        type: "false_accept",
        label: `FA ${(tk.accepted_error ?? 0).toFixed(1)} in`,
      });
    }
  }

  let streakStart: number | null = null;
  for (let i = 0; i < ticks.length; i++) {
    const accepted = ticks[i].gate_decision?.accepted ?? false;
    if (!accepted) {
      if (streakStart === null) streakStart = i;
    } else if (streakStart !== null) {
      const len = i - streakStart;
      if (len >= 5) {
        out.push({
          tick: streakStart,
          endTick: i - 1,
          type: "rejection_streak",
          label: `Reject ${len} ticks`,
        });
      }
      streakStart = null;
    }
  }
  if (streakStart !== null) {
    const len = ticks.length - streakStart;
    if (len >= 5) {
      out.push({
        tick: streakStart,
        endTick: ticks.length - 1,
        type: "rejection_streak",
        label: `Reject ${len} ticks`,
      });
    }
  }

  out.sort((a, b) => a.tick - b.tick);
  return out;
}
