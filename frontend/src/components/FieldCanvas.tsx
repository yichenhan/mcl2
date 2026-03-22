"use client";

import { useEffect, useMemo, useRef } from "react";

import { parseFailures } from "@/lib/failures";
import type { AABB, MCLSnapshot, TickState } from "@/lib/types";
import type { Stage } from "@/components/StageStepper";

export interface RenderOptions {
  particles: boolean;
  estimate: boolean;
  groundTruth: boolean;
  sensorRays: boolean;
  radius90: boolean;
  waypoints: boolean;
}

export const DEFAULT_RENDER_OPTIONS: RenderOptions = {
  particles: true,
  estimate: true,
  groundTruth: true,
  sensorRays: true,
  radius90: true,
  waypoints: true,
};

interface Props {
  tick: TickState | null;
  stage: Stage;
  obstacles: AABB[];
  waypoints?: { x: number; y: number }[];
  currentWaypointIdx?: number;
  prevTick?: TickState | null;
  renderOptions?: RenderOptions;
  className?: string;
}

const FIELD_HALF = 72;

function headingToDir(headingDeg: number) {
  const h = (headingDeg * Math.PI) / 180;
  return { x: Math.sin(h), y: Math.cos(h) };
}

function pickSnapshot(tick: TickState, stage: Stage): MCLSnapshot {
  if (stage === "post_predict") return tick.post_predict;
  if (stage === "post_update") return tick.post_update;
  return tick.post_resample;
}

export function FieldCanvas({
  tick,
  stage,
  obstacles,
  waypoints,
  currentWaypointIdx,
  prevTick,
  renderOptions = DEFAULT_RENDER_OPTIONS,
  className,
}: Props) {
  const opts = renderOptions;
  const prevSelected = useMemo(
    () => (prevTick ? pickSnapshot(prevTick, stage) : null),
    [prevTick, stage],
  );
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const selected = useMemo(() => (tick ? pickSnapshot(tick, stage) : null), [tick, stage]);
  const parsedFailures = useMemo(
    () => parseFailures(tick?.active_failures ?? []),
    [tick?.active_failures],
  );

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const width = canvas.clientWidth || 800;
    const height = canvas.clientHeight || 800;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.floor(width * dpr);
    canvas.height = Math.floor(height * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const pad = 16;
    const span = FIELD_HALF * 2;
    const scale = Math.min((width - 2 * pad) / span, (height - 2 * pad) / span);
    const toCanvas = (x: number, y: number) => ({
      x: width / 2 + x * scale,
      y: height / 2 - y * scale,
    });

    // --- background ---
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#0b1020";
    ctx.fillRect(0, 0, width, height);

    // --- grid ---
    ctx.strokeStyle = "#1f2a44";
    ctx.lineWidth = 1;
    for (let v = -72; v <= 72; v += 12) {
      const p0 = toCanvas(v, -72);
      const p1 = toCanvas(v, 72);
      const p2 = toCanvas(-72, v);
      const p3 = toCanvas(72, v);
      ctx.beginPath();
      ctx.moveTo(p0.x, p0.y);
      ctx.lineTo(p1.x, p1.y);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(p2.x, p2.y);
      ctx.lineTo(p3.x, p3.y);
      ctx.stroke();
    }

    // --- field border ---
    ctx.strokeStyle = "#d4d4d8";
    ctx.lineWidth = 2;
    const tl = toCanvas(-72, 72);
    const br = toCanvas(72, -72);
    ctx.strokeRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);

    // --- obstacles ---
    ctx.fillStyle = "rgba(251, 146, 60, 0.45)";
    for (const o of obstacles) {
      const pMin = toCanvas(o.min_x, o.max_y);
      const pMax = toCanvas(o.max_x, o.min_y);
      ctx.fillRect(pMin.x, pMin.y, pMax.x - pMin.x, pMax.y - pMin.y);
    }

    // --- waypoints ---
    if (opts.waypoints && waypoints && waypoints.length > 0) {
      ctx.strokeStyle = "#06b6d4";
      ctx.lineWidth = 1.5;
      ctx.setLineDash([6, 4]);
      for (let i = 1; i < waypoints.length; i++) {
        const p0 = toCanvas(waypoints[i - 1].x, waypoints[i - 1].y);
        const p1 = toCanvas(waypoints[i].x, waypoints[i].y);
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        ctx.lineTo(p1.x, p1.y);
        ctx.stroke();
      }
      ctx.setLineDash([]);
      for (let i = 0; i < waypoints.length; i++) {
        const p = toCanvas(waypoints[i].x, waypoints[i].y);
        const active = currentWaypointIdx === i;
        ctx.fillStyle = active ? "#06b6d4" : "#3f3f46";
        ctx.strokeStyle = "#e4e4e7";
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 9, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = "#f4f4f5";
        ctx.font = "10px sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(i + 1), p.x, p.y);
      }
    }

    if (!tick || !selected) return;

    // --- particles ---
    if (opts.particles) {
      for (const p of selected.particles) {
        const pt = toCanvas(p.x, p.y);
        const weightAlpha = Math.min(0.85, Math.max(0.15, p.weight * 5));
        ctx.fillStyle = `rgba(96, 165, 250, ${weightAlpha})`;
        ctx.fillRect(pt.x, pt.y, 2, 2);
      }
    }

    // --- estimate crosshair (blue) ---
    if (opts.estimate) {
      const est = toCanvas(selected.estimate.x, selected.estimate.y);

      const gateDecision =
        tick.gate_decision && typeof tick.gate_decision === "object"
          ? (tick.gate_decision as Record<string, unknown>)
          : null;
      const radius90In =
        gateDecision && typeof gateDecision.radius_90_in === "number"
          ? gateDecision.radius_90_in
          : null;

      if (opts.radius90 && radius90In && Number.isFinite(radius90In) && radius90In > 0) {
        ctx.strokeStyle = "rgba(147, 197, 253, 0.8)";
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        ctx.arc(est.x, est.y, radius90In * scale, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      ctx.strokeStyle = "#3b82f6";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(est.x, est.y, 6, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(est.x - 8, est.y);
      ctx.lineTo(est.x + 8, est.y);
      ctx.moveTo(est.x, est.y - 8);
      ctx.lineTo(est.x, est.y + 8);
      ctx.stroke();
    }

    // --- ground truth green dot (when available from full sim) ---
    const gt = tick.ground_truth;
    if (opts.groundTruth && gt) {
      const gtPt = toCanvas(gt.x, gt.y);
      ctx.fillStyle = "rgba(74, 222, 128, 0.7)";
      ctx.beginPath();
      ctx.arc(gtPt.x, gtPt.y, 4, 0, Math.PI * 2);
      ctx.fill();
    }

    // --- accepted / control pose (orange dot + IMU heading) ---
    const ap = tick.accepted_pose;
    if (opts.groundTruth && ap) {
      const apPt = toCanvas(ap.x, ap.y);
      ctx.fillStyle = "#fb923c";
      ctx.beginPath();
      ctx.arc(apPt.x, apPt.y, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#fdba74";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(apPt.x, apPt.y, 5, 0, Math.PI * 2);
      ctx.stroke();

      const headDir = headingToDir(tick.observed_heading);
      const headEnd = toCanvas(ap.x + headDir.x * 6, ap.y + headDir.y * 6);
      ctx.strokeStyle = "#fdba74";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(apPt.x, apPt.y);
      ctx.lineTo(headEnd.x, headEnd.y);
      ctx.stroke();
    }

    // --- robot pose triangle (green/red based on kidnap) ---
    const pose = gt
      ? gt
      : ap
        ? { x: ap.x, y: ap.y, heading_deg: tick.observed_heading }
        : {
            x: selected.estimate.x,
            y: selected.estimate.y,
            heading_deg: tick.observed_heading,
          };

    const robot = toCanvas(pose.x, pose.y);
    const dir = headingToDir(pose.heading_deg);
    const perp = { x: dir.y, y: -dir.x };
    const front = toCanvas(pose.x + dir.x * 3, pose.y + dir.y * 3);
    const left = toCanvas(
      pose.x - dir.x * 2 + perp.x * 2,
      pose.y - dir.y * 2 + perp.y * 2,
    );
    const right = toCanvas(
      pose.x - dir.x * 2 - perp.x * 2,
      pose.y - dir.y * 2 - perp.y * 2,
    );
    const hasKidnap = parsedFailures.some((f) => f.type === "kidnap");
    const hasOdomSpike = parsedFailures.some((f) => f.type === "odom_spike");
    const hasHeadingBias = parsedFailures.some((f) => f.type === "heading_bias");
    ctx.fillStyle = hasKidnap ? "#ef4444" : "#22c55e";
    ctx.beginPath();
    ctx.moveTo(front.x, front.y);
    ctx.lineTo(left.x, left.y);
    ctx.lineTo(right.x, right.y);
    ctx.closePath();
    ctx.fill();

    if (hasOdomSpike) {
      ctx.fillStyle = "rgba(234, 179, 8, 0.25)";
      for (let i = 0; i < 3; i++) {
        const jitter = i - 1;
        ctx.beginPath();
        ctx.moveTo(front.x + jitter, front.y + jitter);
        ctx.lineTo(left.x + jitter, left.y + jitter);
        ctx.lineTo(right.x + jitter, right.y + jitter);
        ctx.closePath();
        ctx.fill();
      }
    }

    if (hasHeadingBias) {
      const f = parsedFailures.find((x) => x.type === "heading_bias");
      const bias = f?.param ?? 0;
      const start = (pose.heading_deg * Math.PI) / 180;
      const end = ((pose.heading_deg + bias) * Math.PI) / 180;
      ctx.strokeStyle = "#a855f7";
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      ctx.arc(robot.x, robot.y, 18, -start + Math.PI / 2, -end + Math.PI / 2, bias > 0);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    if (hasKidnap && prevTick && prevSelected) {
      const prevGt = prevTick.ground_truth;
      const prevPose = prevGt
        ? prevGt
        : {
            x: prevSelected.estimate.x,
            y: prevSelected.estimate.y,
            heading_deg: prevTick.observed_heading,
          };
      const prev = toCanvas(prevPose.x, prevPose.y);
      ctx.strokeStyle = "#ef4444";
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(prev.x, prev.y);
      ctx.lineTo(robot.x, robot.y);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // --- sensor rays ---
    if (opts.sensorRays) {
      const angles = [-90, 90, 0, 180];
      ctx.setLineDash([4, 4]);
      tick.observed_readings.forEach((dist, i) => {
        const dead = parsedFailures.some((f) => f.type === "sensor_dead" && f.sensor === i);
        const stuck = parsedFailures.some((f) => f.type === "sensor_stuck" && f.sensor === i);
        const spurious = parsedFailures.some(
          (f) => f.type === "spurious_reflection" && f.sensor === i,
        );
        const total = pose.heading_deg + angles[i];
        const d = headingToDir(total);
        if (dead || dist < 0) {
          const px = toCanvas(pose.x + d.x * 6, pose.y + d.y * 6);
          ctx.strokeStyle = "#f97316";
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.moveTo(px.x - 4, px.y - 4);
          ctx.lineTo(px.x + 4, px.y + 4);
          ctx.moveTo(px.x + 4, px.y - 4);
          ctx.lineTo(px.x - 4, px.y + 4);
          ctx.stroke();
          return;
        }

        const endPt = toCanvas(pose.x + d.x * dist, pose.y + d.y * dist);
        ctx.strokeStyle = spurious ? "#ef4444" : stuck ? "#71717a" : "#f59e0b";
        ctx.setLineDash(spurious || stuck ? [] : [4, 4]);
        ctx.beginPath();
        ctx.moveTo(robot.x, robot.y);
        ctx.lineTo(endPt.x, endPt.y);
        ctx.stroke();
        if (spurious) {
          ctx.strokeStyle = "#ef4444";
          ctx.beginPath();
          ctx.moveTo(endPt.x, endPt.y - 4);
          ctx.lineTo(endPt.x + 4, endPt.y);
          ctx.lineTo(endPt.x, endPt.y + 4);
          ctx.lineTo(endPt.x - 4, endPt.y);
          ctx.closePath();
          ctx.stroke();
        }
      });
      ctx.setLineDash([]);
    }
  }, [currentWaypointIdx, obstacles, opts, parsedFailures, prevSelected, prevTick, selected, stage, tick, waypoints]);

  const kidnapActive = parsedFailures.some((f) => f.type === "kidnap");

  return (
    <canvas
      ref={canvasRef}
      className={`${className ?? "h-[640px] w-full rounded border border-zinc-700"} ${
        kidnapActive ? "border-red-500 shadow-[0_0_20px_rgba(239,68,68,0.45)]" : ""
      }`}
    />
  );
}
