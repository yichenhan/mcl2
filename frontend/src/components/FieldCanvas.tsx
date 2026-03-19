"use client";

import { useEffect, useMemo, useRef } from "react";

import type { AABB, MCLSnapshot, TickState } from "@/lib/types";
import type { Stage } from "@/components/StageStepper";

interface Props {
  tick: TickState | null;
  stage: Stage;
  obstacles: AABB[];
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

export function FieldCanvas({ tick, stage, obstacles, className }: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const selected = useMemo(() => (tick ? pickSnapshot(tick, stage) : null), [tick, stage]);

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

    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#0b1020";
    ctx.fillRect(0, 0, width, height);

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

    ctx.strokeStyle = "#d4d4d8";
    ctx.lineWidth = 2;
    const tl = toCanvas(-72, 72);
    const br = toCanvas(72, -72);
    ctx.strokeRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);

    ctx.fillStyle = "rgba(251, 146, 60, 0.45)";
    for (const o of obstacles) {
      const pMin = toCanvas(o.min_x, o.max_y);
      const pMax = toCanvas(o.max_x, o.min_y);
      ctx.fillRect(pMin.x, pMin.y, pMax.x - pMin.x, pMax.y - pMin.y);
    }

    if (!tick || !selected) return;

    for (const p of selected.particles) {
      const pt = toCanvas(p.x, p.y);
      const weightAlpha = Math.min(0.85, Math.max(0.15, p.weight * 5));
      ctx.fillStyle = `rgba(96, 165, 250, ${weightAlpha})`;
      ctx.fillRect(pt.x, pt.y, 2, 2);
    }

    const est = toCanvas(selected.estimate.x, selected.estimate.y);
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

    const gt = tick.ground_truth;
    const robot = toCanvas(gt.x, gt.y);
    const dir = headingToDir(gt.heading_deg);
    const perp = { x: dir.y, y: -dir.x };
    const front = toCanvas(gt.x + dir.x * 3, gt.y + dir.y * 3);
    const left = toCanvas(gt.x - dir.x * 2 + perp.x * 2, gt.y - dir.y * 2 + perp.y * 2);
    const right = toCanvas(gt.x - dir.x * 2 - perp.x * 2, gt.y - dir.y * 2 - perp.y * 2);
    ctx.fillStyle = "#22c55e";
    ctx.beginPath();
    ctx.moveTo(front.x, front.y);
    ctx.lineTo(left.x, left.y);
    ctx.lineTo(right.x, right.y);
    ctx.closePath();
    ctx.fill();

    const angles = [-90, 90, 0, 180];
    ctx.setLineDash([4, 4]);
    tick.observed_readings.forEach((dist, i) => {
      if (dist < 0) return;
      const total = gt.heading_deg + angles[i];
      const d = headingToDir(total);
      const end = toCanvas(gt.x + d.x * dist, gt.y + d.y * dist);
      ctx.strokeStyle = "#f59e0b";
      ctx.beginPath();
      ctx.moveTo(robot.x, robot.y);
      ctx.lineTo(end.x, end.y);
      ctx.stroke();
    });
    ctx.setLineDash([]);
  }, [obstacles, selected, stage, tick]);

  return (
    <canvas
      ref={canvasRef}
      className={className ?? "h-[640px] w-full rounded border border-zinc-700"}
    />
  );
}
