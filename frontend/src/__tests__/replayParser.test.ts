import { describe, expect, it } from "vitest";

import { parseReplayText } from "@/lib/replayParser";

const COMPACT_SAMPLE = `
[T=1||tc]=1
[T=1||oh]=90.0
[T=1||pg]=1
[T=1||ts]=2026-04-04T00:15:06.636Z
[T=1||vs]=2
[T=1||us]=0
[T=1||ne]=20.0
[T=1||or.0]=10.00
[T=1||or.1]=10.00
[T=1||or.2]=-1.00
[T=1||or.3]=-1.00
[T=1||op.x]=5.50
[T=1||op.y]=3.20
[T=1||op.t]=90.0
[T=1||re.x]=-6.09
[T=1||re.y]=-7.32
[T=1||re.t]=90.0
[T=1||cs.x]=-6.09
[T=1||cs.y]=-7.32
[T=1||cs.r]=87.71
[T=1||cs.s]=62.71
[T=1||g.a]=0
[T=1||g.rsn]=sensor residual gate
[T=1||g.r]=87.71
[T=1||g.j]=9.52
[T=1||g.s]=62.71
[T=1||g.fcj]=0
[T=1||g.fr]=0
[T=1||g.fp]=0
[T=1||g.fres]=1
[T=1||g.wcj]=0
[T=1||g.wr]=0
[T=1||g.wp]=0
[T=1||g.wres]=1
[T=1||g.msr]=63.28
[T=1||g.rt]=1.000
[T=1||g.cj]=15.87
[T=1||g.ne]=20.0
[T=1||pr.0]=73.28
[T=1||pr.1]=58.67
[T=1||pr.2]=-1.00
[T=1||pr.3]=-1.00
[T=1||sr.0]=63.28
[T=1||sr.1]=48.67
[T=1||sr.2]=0.00
[T=1||sr.3]=0.00
[T=1||pp.x]=-6.09
[T=1||pp.y]=-7.32
[T=1||pp.n]=20.0
[T=1||pp.s]=62.71
[T=1||pp.r]=84.62
[T=1||pu.x]=-6.09
[T=1||pu.y]=-7.32
[T=1||pu.n]=20.0
[T=1||pu.s]=62.71
[T=1||pu.r]=87.71
[T=1||ps.x]=-6.09
[T=1||ps.y]=-7.32
[T=1||ps.n]=20.0
[T=1||ps.s]=62.71
[T=1||ps.r]=87.71
`.trim();

describe("parseReplayText – compact [T=N||k]=v format", () => {
  it("parses compact lines into TickState", () => {
    const ticks = parseReplayText(COMPACT_SAMPLE);
    expect(ticks).toHaveLength(1);
    const t = ticks[0];

    expect(t.tick).toBe(1);
    expect(t.observed_heading).toBe(90.0);
    expect(t.pose_gated).toBe(true);
    expect(t.timestamp_iso).toBe("2026-04-04T00:15:06.636Z");
    expect(t.valid_sensor_count).toBe(2);
    expect(t.update_skipped).toBe(false);
    expect(t.observed_readings).toEqual([10.0, 10.0, -1.0, -1.0]);
  });

  it("expands raw_odom correctly", () => {
    const t = parseReplayText(COMPACT_SAMPLE)[0];
    expect(t.raw_odom).toBeDefined();
    expect(t.raw_odom!.x).toBeCloseTo(5.5);
    expect(t.raw_odom!.y).toBeCloseTo(3.2);
  });

  it("expands cluster_stats correctly", () => {
    const t = parseReplayText(COMPACT_SAMPLE)[0];
    expect(t.cluster_stats).toBeDefined();
    expect(t.cluster_stats!.centroid.x).toBeCloseTo(-6.09);
    expect(t.cluster_stats!.radius_90).toBeCloseTo(87.71);
    expect(t.cluster_stats!.spread).toBeCloseTo(62.71);
  });

  it("expands post_resample correctly", () => {
    const t = parseReplayText(COMPACT_SAMPLE)[0];
    expect(t.post_resample.radius_90).toBeCloseTo(87.71);
    expect(t.post_resample.spread).toBeCloseTo(62.71);
    expect(t.post_resample.estimate.x).toBeCloseTo(-6.09);
  });

  it("expands gate_decision boolean fields", () => {
    const t = parseReplayText(COMPACT_SAMPLE)[0];
    expect(t.gate_decision).toBeDefined();
    expect(t.gate_decision!.accepted).toBe(false);
    expect(t.gate_decision!.failed_residual).toBe(true);
    expect(t.gate_decision!.failed_r90).toBe(false);
    expect(t.gate_decision!.reason).toBe("sensor residual gate");
  });

  it("expands raw_estimate", () => {
    const t = parseReplayText(COMPACT_SAMPLE)[0];
    expect(t.raw_estimate).toBeDefined();
    expect(t.raw_estimate!.x).toBeCloseTo(-6.09);
    expect(t.raw_estimate!.y).toBeCloseTo(-7.32);
    expect(t.raw_estimate!.theta).toBeCloseTo(90.0);
  });

  it("expands predicted readings and sensor residuals", () => {
    const t = parseReplayText(COMPACT_SAMPLE)[0];
    expect(t.mcl_predicted_readings).toEqual([73.28, 58.67, -1.0, -1.0]);
    expect(t.sensor_residuals).toEqual([63.28, 48.67, 0.0, 0.0]);
  });

  it("handles mixed noise lines", () => {
    const noisy = `
some garbage
[T=5||tc]=5
[T=5||oh]=45.0
[T=5||vs]=4
more garbage
[T=5||cs.r]=12.50
    `.trim();
    const ticks = parseReplayText(noisy);
    expect(ticks).toHaveLength(1);
    expect(ticks[0].observed_heading).toBe(45.0);
    expect(ticks[0].cluster_stats?.radius_90).toBe(12.5);
  });

  it("uses the compact tick id even when tc is missing", () => {
    const partial = `
[T=7||cp.x]=12.34
[T=7||cp.y]=56.78
[T=7||cp.t]=90.0
    `.trim();
    const ticks = parseReplayText(partial);
    expect(ticks).toHaveLength(1);
    expect(ticks[0].tick).toBe(7);
    expect(ticks[0].chassis_pose).toBeDefined();
    expect(ticks[0].chassis_pose!.x).toBeCloseTo(12.34);
    expect(ticks[0].chassis_pose!.y).toBeCloseTo(56.78);
    expect(ticks[0].chassis_pose!.theta).toBeCloseTo(90.0);
  });
});
