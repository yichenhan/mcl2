"use client";

import { useEffect, useMemo, useRef } from "react";

import { parseFailures } from "@/lib/failures";
import { isTickState } from "@/lib/types";
import type { AABB, AnyTick, MCLSnapshot } from "@/lib/types";
import type { Stage } from "@/components/StageStepper";

interface Props {
  tick: AnyTick | null;
  stage: Stage;
  obstacles: AABB[];
  fieldHalf?: number;
  waypoints?: { x: number; y: number }[];
  currentWaypointIdx?: number;
  prevGroundTruth?: { x: number; y: number } | null;
  className?: string;
}

const FIELD_HALF = 72;

function headingToDir(headingDeg: number) {
  const h = (headingDeg * Math.PI) / 180;
  return { x: Math.sin(h), y: Math.cos(h) };
}

function pickSnapshot(tick: AnyTick, stage: Stage): MCLSnapshot {
  if (stage === "post_predict") return tick.post_predict;
  if (stage === "post_update") return tick.post_update;
  return tick.post_resample;
}

export function FieldCanvas({
  tick,
  stage,
  obstacles,
  fieldHalf,
  waypoints,
  currentWaypointIdx,
  prevGroundTruth,
  className,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const selected = useMemo(() => (tick ? pickSnapshot(tick, stage) : null), [tick, stage]);
  const activeFieldHalf = fieldHalf ?? FIELD_HALF;
  const parsedFailures = useMemo(
    () => (tick && isTickState(tick) ? parseFailures(tick.active_failures) : []),
    [tick],
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
    const span = activeFieldHalf * 2;
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
    for (let v = -activeFieldHalf; v <= activeFieldHalf; v += 12) {
      const p0 = toCanvas(v, -activeFieldHalf);
      const p1 = toCanvas(v, activeFieldHalf);
      const p2 = toCanvas(-activeFieldHalf, v);
      const p3 = toCanvas(activeFieldHalf, v);
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
    const tl = toCanvas(-activeFieldHalf, activeFieldHalf);
    const br = toCanvas(activeFieldHalf, -activeFieldHalf);
    ctx.strokeRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);

    ctx.fillStyle = "rgba(251, 146, 60, 0.45)";
    for (const o of obstacles) {
      const pMin = toCanvas(o.min_x, o.max_y);
      const pMax = toCanvas(o.max_x, o.min_y);
      ctx.fillRect(pMin.x, pMin.y, pMax.x - pMin.x, pMax.y - pMin.y);
    }

    if (waypoints && waypoints.length > 0) {
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

    if (isTickState(tick)) {
      const gt = tick.ground_truth;
      const robot = toCanvas(gt.x, gt.y);
      const dir = headingToDir(gt.heading_deg);
      const perp = { x: dir.y, y: -dir.x };
      const front = toCanvas(gt.x + dir.x * 3, gt.y + dir.y * 3);
      const left = toCanvas(gt.x - dir.x * 2 + perp.x * 2, gt.y - dir.y * 2 + perp.y * 2);
      const right = toCanvas(gt.x - dir.x * 2 - perp.x * 2, gt.y - dir.y * 2 - perp.y * 2);
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
        const start = (gt.heading_deg * Math.PI) / 180;
        const end = ((gt.heading_deg + bias) * Math.PI) / 180;
        ctx.strokeStyle = "#a855f7";
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.arc(robot.x, robot.y, 18, -start + Math.PI / 2, -end + Math.PI / 2, bias > 0);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      if (hasKidnap && prevGroundTruth) {
        const prev = toCanvas(prevGroundTruth.x, prevGroundTruth.y);
        ctx.strokeStyle = "#ef4444";
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(prev.x, prev.y);
        ctx.lineTo(robot.x, robot.y);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      const angles = [-90, 90, 0, 180];
      ctx.setLineDash([4, 4]);
      tick.observed_readings.forEach((dist, i) => {
        const dead = parsedFailures.some((f) => f.type === "sensor_dead" && f.sensor === i);
        const stuck = parsedFailures.some((f) => f.type === "sensor_stuck" && f.sensor === i);
        const spurious = parsedFailures.some((f) => f.type === "spurious_reflection" && f.sensor === i);
        const total = gt.heading_deg + angles[i];
        const d = headingToDir(total);
        if (dead || dist < 0) {
          const px = toCanvas(gt.x + d.x * 6, gt.y + d.y * 6);
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

        const end = toCanvas(gt.x + d.x * dist, gt.y + d.y * dist);
        ctx.strokeStyle = spurious ? "#ef4444" : stuck ? "#71717a" : "#f59e0b";
        ctx.setLineDash(spurious || stuck ? [] : [4, 4]);
        ctx.beginPath();
        ctx.moveTo(robot.x, robot.y);
        ctx.lineTo(end.x, end.y);
        ctx.stroke();
        if (spurious) {
          ctx.strokeStyle = "#ef4444";
          ctx.beginPath();
          ctx.moveTo(end.x, end.y - 4);
          ctx.lineTo(end.x + 4, end.y);
          ctx.lineTo(end.x, end.y + 4);
          ctx.lineTo(end.x - 4, end.y);
          ctx.closePath();
          ctx.stroke();
        }
      });
      ctx.setLineDash([]);
    } else {
      // MCL replay mode:
      // 1) Odom pose
      // 2) MCL prediction with heading (raw estimate)
      // 3) Sensor readings from odom pose
      // 4) MCL diff vector and magnitude between odom and MCL prediction
      if (tick.odom_pose) {
        const odom = tick.odom_pose;
        const odomDir = headingToDir(odom.theta);
        const odomPerp = { x: odomDir.y, y: -odomDir.x };
        const front = toCanvas(odom.x + odomDir.x * 3, odom.y + odomDir.y * 3);
        const left = toCanvas(
          odom.x - odomDir.x * 2 + odomPerp.x * 2,
          odom.y - odomDir.y * 2 + odomPerp.y * 2,
        );
        const right = toCanvas(
          odom.x - odomDir.x * 2 - odomPerp.x * 2,
          odom.y - odomDir.y * 2 - odomPerp.y * 2,
        );
        ctx.fillStyle = "#22c55e";
        ctx.beginPath();
        ctx.moveTo(front.x, front.y);
        ctx.lineTo(left.x, left.y);
        ctx.lineTo(right.x, right.y);
        ctx.closePath();
        ctx.fill();
      }

      if (tick.odom_pose && tick.observed_readings) {
        const odom = tick.odom_pose;
        const odomPt = toCanvas(odom.x, odom.y);
        const angles = [-90, 90, 0, 180];
        ctx.setLineDash([4, 4]);
        tick.observed_readings.forEach((dist, i) => {
          const d = headingToDir(odom.theta + angles[i]);
          if (dist < 0) {
            const px = toCanvas(odom.x + d.x * 6, odom.y + d.y * 6);
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
          const end = toCanvas(odom.x + d.x * dist, odom.y + d.y * dist);
          ctx.strokeStyle = "#f59e0b";
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          ctx.moveTo(odomPt.x, odomPt.y);
          ctx.lineTo(end.x, end.y);
          ctx.stroke();
        });
        ctx.setLineDash([]);
      }

      const raw = toCanvas(tick.raw_estimate.x, tick.raw_estimate.y);
      const rawDir = headingToDir(tick.raw_estimate.theta);
      const rawFront = toCanvas(
        tick.raw_estimate.x + rawDir.x * 4,
        tick.raw_estimate.y + rawDir.y * 4,
      );
      const rawBack = toCanvas(
        tick.raw_estimate.x - rawDir.x * 3,
        tick.raw_estimate.y - rawDir.y * 3,
      );
      ctx.strokeStyle = "#f97316";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(raw.x, raw.y, 5, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(rawBack.x, rawBack.y);
      ctx.lineTo(rawFront.x, rawFront.y);
      ctx.stroke();

      if (tick.odom_pose) {
        const odom = tick.odom_pose;
        const odomPt = toCanvas(odom.x, odom.y);
        const dx = tick.raw_estimate.x - odom.x;
        const dy = tick.raw_estimate.y - odom.y;
        const diff = Math.hypot(dx, dy);
        // White line links odom and MCL prediction to make drift obvious.
        ctx.strokeStyle = "#e4e4e7";
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(odomPt.x, odomPt.y);
        ctx.lineTo(raw.x, raw.y);
        ctx.stroke();
        ctx.fillStyle = "#f4f4f5";
        ctx.font = "11px sans-serif";
        ctx.textAlign = "left";
        ctx.textBaseline = "bottom";
        ctx.fillText(`d=${diff.toFixed(2)} in`, raw.x + 6, raw.y - 6);
      }
    }
  }, [
    activeFieldHalf,
    currentWaypointIdx,
    obstacles,
    parsedFailures,
    prevGroundTruth,
    selected,
    tick,
    waypoints,
  ]);

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
