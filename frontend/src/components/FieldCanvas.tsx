"use client";

import { useEffect, useMemo, useRef } from "react";

import { parseFailures } from "@/lib/failures";
import { isTickState } from "@/lib/types";
import type { AnyTick, MCLSnapshot, Obstacle, OverlayFlags, Particle } from "@/lib/types";

type Stage = "post_predict" | "post_update" | "post_resample";

export type HeatmapMode = "off" | "density" | "weight";

interface Props {
  tick: AnyTick | null;
  stage: Stage;
  obstacles: Obstacle[];
  fieldHalf?: number;
  waypoints?: { x: number; y: number }[];
  currentWaypointIdx?: number;
  prevGroundTruth?: { x: number; y: number } | null;
  navTarget?: { x: number; y: number } | null;
  /** When `tick` is null, draw this pose as the planned start (e.g. session setup). */
  placementPose?: { x: number; y: number; heading_deg: number } | null;
  onFieldClick?: (x: number, y: number) => void;
  heatmapMode?: HeatmapMode;
  overlayFlags?: OverlayFlags;
  className?: string;
}

const FIELD_HALF = 72;
const STEP_IN = 12;
const FIELD_IMAGE_PATH = "/vex-v5-field.png";
const PLAY_AREA_INSET_X = 0.026;
const PLAY_AREA_INSET_Y = 0.026;

function isRectObstacle(o: Obstacle): o is { min_x: number; min_y: number; max_x: number; max_y: number; type?: "rect"; color?: string } {
  return "min_x" in o && "min_y" in o && "max_x" in o && "max_y" in o;
}

function headingToDir(headingDeg: number) {
  const h = (headingDeg * Math.PI) / 180;
  return { x: Math.sin(h), y: Math.cos(h) };
}

const HEATMAP_CELLS = 24;

function renderHeatmap(
  ctx: CanvasRenderingContext2D,
  particles: Particle[],
  mode: "density" | "weight",
  fh: number,
  toCanvas: (x: number, y: number) => { x: number; y: number },
  cellScale: number,
) {
  const cellSize = (fh * 2) / HEATMAP_CELLS;
  const grid = new Float64Array(HEATMAP_CELLS * HEATMAP_CELLS);

  for (const p of particles) {
    const col = Math.floor((p.x + fh) / cellSize);
    const row = Math.floor((p.y + fh) / cellSize);
    if (col < 0 || col >= HEATMAP_CELLS || row < 0 || row >= HEATMAP_CELLS) continue;
    grid[row * HEATMAP_CELLS + col] += mode === "weight" ? p.weight : 1;
  }

  let totalVal = 0;
  for (let i = 0; i < grid.length; i++) totalVal += grid[i];
  if (totalVal <= 0) return;

  let maxFrac = 0;
  for (let i = 0; i < grid.length; i++) {
    const frac = grid[i] / totalVal;
    if (frac > maxFrac) maxFrac = frac;
  }
  if (maxFrac <= 0) return;

  const cellPx = cellSize * cellScale;
  ctx.save();
  for (let row = 0; row < HEATMAP_CELLS; row++) {
    for (let col = 0; col < HEATMAP_CELLS; col++) {
      const val = grid[row * HEATMAP_CELLS + col];
      if (val <= 0) continue;
      const t = Math.sqrt((val / totalVal) / maxFrac);
      const fieldX = -fh + col * cellSize;
      const fieldY = -fh + row * cellSize;
      const corner = toCanvas(fieldX, fieldY + cellSize);
      let r: number, g: number, b: number;
      if (t < 0.25) {
        const s = t / 0.25;
        r = Math.round(48 + s * (20 - 48));
        g = Math.round(18 + s * (150 - 18));
        b = Math.round(130 + s * (220 - 130));
      } else if (t < 0.5) {
        const s = (t - 0.25) / 0.25;
        r = Math.round(20 + s * (80 - 20));
        g = Math.round(150 + s * (220 - 150));
        b = Math.round(220 + s * (80 - 220));
      } else if (t < 0.75) {
        const s = (t - 0.5) / 0.25;
        r = Math.round(80 + s * (240 - 80));
        g = Math.round(220 + s * (180 - 220));
        b = Math.round(80 + s * (10 - 80));
      } else {
        const s = (t - 0.75) / 0.25;
        r = Math.round(240 + s * (220 - 240));
        g = Math.round(180 + s * (30 - 180));
        b = Math.round(10 + s * (30 - 10));
      }
      ctx.fillStyle = `rgba(${r}, ${g}, ${b}, ${0.15 + t * 0.6})`;
      ctx.fillRect(corner.x, corner.y, cellPx, cellPx);
    }
  }
  ctx.restore();
}

function pickSnapshot(tick: AnyTick, stage: Stage): MCLSnapshot | null {
  if (stage === "post_predict") return tick.post_predict ?? null;
  if (stage === "post_update") return tick.post_update ?? null;
  return tick.post_resample ?? null;
}

export function FieldCanvas({
  tick,
  stage,
  obstacles,
  fieldHalf,
  waypoints,
  currentWaypointIdx,
  prevGroundTruth,
  navTarget,
  placementPose,
  onFieldClick,
  heatmapMode,
  overlayFlags,
  className,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const bgImageRef = useRef<HTMLImageElement | null>(null);
  const selected = useMemo(() => (tick ? pickSnapshot(tick, stage) : null), [tick, stage]);
  const flags: OverlayFlags = overlayFlags ?? {
    robotTruth: true,
    odomPose: true,
    mclEstimate: true,
    acceptedEstimate: true,
    r90Circle: true,
    particles: true,
    heatmap: true,
    sensorReadings: true,
    sensorResiduals: true,
    diffMclPose: true,
    diffMclTruth: true,
    diffPoseTruth: true,
  };
  const activeFieldHalf = fieldHalf ?? FIELD_HALF;
  const parsedFailures = useMemo(
    () => (tick && isTickState(tick) ? parseFailures(tick.active_failures) : []),
    [tick],
  );

  useEffect(() => {
    const img = new Image();
    img.src = FIELD_IMAGE_PATH;
    bgImageRef.current = img;
    return () => {
      bgImageRef.current = null;
    };
  }, []);

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

    const tl = toCanvas(-activeFieldHalf, activeFieldHalf);
    const br = toCanvas(activeFieldHalf, -activeFieldHalf);
    const bg = bgImageRef.current;
    if (bg && bg.complete && bg.naturalWidth > 0 && bg.naturalHeight > 0) {
      const sx = bg.naturalWidth * PLAY_AREA_INSET_X;
      const sy = bg.naturalHeight * PLAY_AREA_INSET_Y;
      const sw = bg.naturalWidth * (1 - PLAY_AREA_INSET_X * 2);
      const sh = bg.naturalHeight * (1 - PLAY_AREA_INSET_Y * 2);
      ctx.save();
      ctx.globalAlpha = 0.35;
      ctx.drawImage(bg, sx, sy, sw, sh, tl.x, tl.y, br.x - tl.x, br.y - tl.y);
      ctx.restore();
    }

    ctx.strokeStyle = "#1f2a44";
    ctx.lineWidth = 1;
    for (let v = 0; v <= activeFieldHalf; v += STEP_IN) {
      const pXPos0 = toCanvas(v, -activeFieldHalf);
      const pXPos1 = toCanvas(v, activeFieldHalf);
      const pYPos0 = toCanvas(-activeFieldHalf, v);
      const pYPos1 = toCanvas(activeFieldHalf, v);
      ctx.beginPath();
      ctx.moveTo(pXPos0.x, pXPos0.y);
      ctx.lineTo(pXPos1.x, pXPos1.y);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(pYPos0.x, pYPos0.y);
      ctx.lineTo(pYPos1.x, pYPos1.y);
      ctx.stroke();
      if (v > 0) {
        const pXNeg0 = toCanvas(-v, -activeFieldHalf);
        const pXNeg1 = toCanvas(-v, activeFieldHalf);
        const pYNeg0 = toCanvas(-activeFieldHalf, -v);
        const pYNeg1 = toCanvas(activeFieldHalf, -v);
        ctx.beginPath();
        ctx.moveTo(pXNeg0.x, pXNeg0.y);
        ctx.lineTo(pXNeg1.x, pXNeg1.y);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(pYNeg0.x, pYNeg0.y);
        ctx.lineTo(pYNeg1.x, pYNeg1.y);
        ctx.stroke();
      }
    }

    ctx.strokeStyle = "#94a3b8";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(toCanvas(0, -activeFieldHalf).x, toCanvas(0, -activeFieldHalf).y);
    ctx.lineTo(toCanvas(0, activeFieldHalf).x, toCanvas(0, activeFieldHalf).y);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(toCanvas(-activeFieldHalf, 0).x, toCanvas(-activeFieldHalf, 0).y);
    ctx.lineTo(toCanvas(activeFieldHalf, 0).x, toCanvas(activeFieldHalf, 0).y);
    ctx.stroke();

    ctx.strokeStyle = "rgba(255,255,255,0.8)";
    ctx.lineWidth = 2;
    ctx.setLineDash([8, 6]);
    ctx.beginPath();
    ctx.moveTo(toCanvas(-activeFieldHalf, 0).x, toCanvas(-activeFieldHalf, 0).y);
    ctx.lineTo(toCanvas(activeFieldHalf, 0).x, toCanvas(activeFieldHalf, 0).y);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.strokeStyle = "#d4d4d8";
    ctx.lineWidth = 2;
    ctx.strokeRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);

    if (flags.heatmap && heatmapMode && heatmapMode !== "off" && tick?.post_predict?.particles) {
      renderHeatmap(ctx, tick.post_predict.particles, heatmapMode, activeFieldHalf, toCanvas, scale);
    }

    ctx.save();
    ctx.globalAlpha = 0.55;
    for (const o of obstacles) {
      const obstacleColor = ("color" in o && typeof o.color === "string" && o.color.length > 0)
        ? o.color
        : "rgb(251, 146, 60)";
      ctx.fillStyle = obstacleColor;
      if ("type" in o && o.type === "circle") {
        const center = toCanvas(o.cx, o.cy);
        ctx.beginPath();
        ctx.arc(center.x, center.y, o.radius * scale, 0, Math.PI * 2);
        ctx.fill();
        continue;
      }
      if ("type" in o && o.type === "triangle") {
        const p0 = toCanvas(o.x0, o.y0);
        const p1 = toCanvas(o.x1, o.y1);
        const p2 = toCanvas(o.x2, o.y2);
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        ctx.lineTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.closePath();
        ctx.fill();
        continue;
      }
      if (isRectObstacle(o)) {
        const pMin = toCanvas(o.min_x, o.max_y);
        const pMax = toCanvas(o.max_x, o.min_y);
        ctx.fillRect(pMin.x, pMin.y, pMax.x - pMin.x, pMax.y - pMin.y);
      }
    }
    ctx.restore();

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

    if (navTarget) {
      const p = toCanvas(navTarget.x, navTarget.y);
      ctx.strokeStyle = "#38bdf8";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 12, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(p.x - 16, p.y);
      ctx.lineTo(p.x + 16, p.y);
      ctx.moveTo(p.x, p.y - 16);
      ctx.lineTo(p.x, p.y + 16);
      ctx.stroke();
    }

    if (!tick && placementPose) {
      const gt = placementPose;
      const dir = headingToDir(gt.heading_deg);
      const perp = { x: dir.y, y: -dir.x };
      const front = toCanvas(gt.x + dir.x * 3, gt.y + dir.y * 3);
      const left = toCanvas(gt.x - dir.x * 2 + perp.x * 2, gt.y - dir.y * 2 + perp.y * 2);
      const right = toCanvas(gt.x - dir.x * 2 - perp.x * 2, gt.y - dir.y * 2 - perp.y * 2);
      ctx.strokeStyle = "#fbbf24";
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      ctx.moveTo(front.x, front.y);
      ctx.lineTo(left.x, left.y);
      ctx.lineTo(right.x, right.y);
      ctx.closePath();
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "rgba(251, 191, 36, 0.35)";
      ctx.fill();
      ctx.fillStyle = "#fbbf24";
      ctx.font = "11px sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "bottom";
      ctx.fillText("Start", toCanvas(gt.x, gt.y).x, toCanvas(gt.x, gt.y).y - 14);
    }

    if (!tick || !selected) return;

    if (isTickState(tick)) {
      const gt = tick.ground_truth;
      const hasGt = gt !== undefined;
      const hasKidnap = parsedFailures.some((f) => f.type === "kidnap");
      const hasOdomSpike = parsedFailures.some((f) => f.type === "odom_spike");
      const hasHeadingBias = parsedFailures.some((f) => f.type === "heading_bias");

      // Primary robot icon: chassis_pose (MCL-corrected, what robot believes)
      // Falls back to accepted_estimate, then raw_estimate, then ground_truth for old replays.
      const robotPose = tick.chassis_pose
        ?? (tick.accepted_estimate
          ? { x: tick.accepted_estimate.x, y: tick.accepted_estimate.y, theta: tick.observed_heading }
          : (tick.raw_estimate
            ? { x: tick.raw_estimate.x, y: tick.raw_estimate.y, theta: tick.raw_estimate.theta }
            : (hasGt
              ? { x: gt.x, y: gt.y, theta: gt.heading_deg }
              : { x: 0, y: 0, theta: 0 })));
      const robotDir = headingToDir(robotPose.theta);
      const robotPerp = { x: robotDir.y, y: -robotDir.x };
      const robotFront = toCanvas(robotPose.x + robotDir.x * 3, robotPose.y + robotDir.y * 3);
      const robotLeft = toCanvas(robotPose.x - robotDir.x * 2 + robotPerp.x * 2, robotPose.y - robotDir.y * 2 + robotPerp.y * 2);
      const robotRight = toCanvas(robotPose.x - robotDir.x * 2 - robotPerp.x * 2, robotPose.y - robotDir.y * 2 - robotPerp.y * 2);

      // Ground truth overlay (debug -- shown when robotTruth flag is ON and data exists)
      if (flags.robotTruth && hasGt) {
        const dir = headingToDir(gt.heading_deg);
        const perp = { x: dir.y, y: -dir.x };
        const front = toCanvas(gt.x + dir.x * 3, gt.y + dir.y * 3);
        const left = toCanvas(gt.x - dir.x * 2 + perp.x * 2, gt.y - dir.y * 2 + perp.y * 2);
        const right = toCanvas(gt.x - dir.x * 2 - perp.x * 2, gt.y - dir.y * 2 - perp.y * 2);
        ctx.fillStyle = hasKidnap ? "#ef4444" : "#22c55e";
        ctx.globalAlpha = 0.5;
        ctx.beginPath();
        ctx.moveTo(front.x, front.y);
        ctx.lineTo(left.x, left.y);
        ctx.lineTo(right.x, right.y);
        ctx.closePath();
        ctx.fill();
        ctx.globalAlpha = 1.0;
      }

      // Draw chassis_pose as the primary robot icon (blue/white)
      ctx.fillStyle = hasKidnap ? "#f87171" : "#3b82f6";
      ctx.strokeStyle = "#e2e8f0";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(robotFront.x, robotFront.y);
      ctx.lineTo(robotLeft.x, robotLeft.y);
      ctx.lineTo(robotRight.x, robotRight.y);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      const robot = toCanvas(robotPose.x, robotPose.y);

      if (hasOdomSpike) {
        ctx.fillStyle = "rgba(234, 179, 8, 0.25)";
        for (let i = 0; i < 3; i++) {
          const jitter = i - 1;
          ctx.beginPath();
          ctx.moveTo(robotFront.x + jitter, robotFront.y + jitter);
          ctx.lineTo(robotLeft.x + jitter, robotLeft.y + jitter);
          ctx.lineTo(robotRight.x + jitter, robotRight.y + jitter);
          ctx.closePath();
          ctx.fill();
        }
      }

      if (hasHeadingBias) {
        const f = parsedFailures.find((x) => x.type === "heading_bias");
        const bias = f?.param ?? 0;
        const start = (robotPose.theta * Math.PI) / 180;
        const end = ((robotPose.theta + bias) * Math.PI) / 180;
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
      if (flags.sensorReadings) {
        ctx.setLineDash([4, 4]);
      }
      tick.observed_readings.forEach((dist, i) => {
        if (!flags.sensorReadings) return;
        const dead = parsedFailures.some((f) => f.type === "sensor_dead" && f.sensor === i);
        const stuck = parsedFailures.some((f) => f.type === "sensor_stuck" && f.sensor === i);
        const spurious = parsedFailures.some((f) => f.type === "spurious_reflection" && f.sensor === i);
        const total = robotPose.theta + angles[i];
        const d = headingToDir(total);
        if (dead || dist < 0) {
          const px = toCanvas(robotPose.x + d.x * 6, robotPose.y + d.y * 6);
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

        const end = toCanvas(robotPose.x + d.x * dist, robotPose.y + d.y * dist);
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

      if (flags.odomPose && tick.odom_pose) {
        const od = toCanvas(tick.odom_pose.x, tick.odom_pose.y);
        ctx.strokeStyle = "#10b981";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(od.x, od.y, 5, 0, Math.PI * 2);
        ctx.stroke();
      }
      if (flags.mclEstimate && tick.raw_estimate) {
        const raw = toCanvas(tick.raw_estimate.x, tick.raw_estimate.y);
        ctx.strokeStyle = "#f97316";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(raw.x, raw.y, 5, 0, Math.PI * 2);
        ctx.stroke();
      }
      if (flags.acceptedEstimate && tick.accepted_estimate) {
        const acc = toCanvas(tick.accepted_estimate.x, tick.accepted_estimate.y);
        ctx.fillStyle = "#3b82f6";
        ctx.beginPath();
        ctx.arc(acc.x, acc.y, 4, 0, Math.PI * 2);
        ctx.fill();
      }
      if (flags.r90Circle && tick.raw_estimate && tick.post_resample?.radius_90 !== undefined) {
        const c = toCanvas(tick.raw_estimate.x, tick.raw_estimate.y);
        ctx.strokeStyle = "rgba(59,130,246,0.6)";
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(c.x, c.y, tick.post_resample.radius_90 * scale, 0, Math.PI * 2);
        ctx.stroke();
      }
      if (tick.raw_estimate && tick.odom_pose && flags.diffMclPose) {
        const a = toCanvas(tick.raw_estimate.x, tick.raw_estimate.y);
        const b = toCanvas(tick.odom_pose.x, tick.odom_pose.y);
        ctx.strokeStyle = "rgba(229,231,235,0.8)";
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      if (hasGt && tick.raw_estimate && flags.diffMclTruth) {
        const a = toCanvas(tick.raw_estimate.x, tick.raw_estimate.y);
        const b = toCanvas(gt.x, gt.y);
        ctx.strokeStyle = "rgba(244,114,182,0.8)";
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      if (hasGt && tick.odom_pose && flags.diffPoseTruth) {
        const a = toCanvas(tick.odom_pose.x, tick.odom_pose.y);
        const b = toCanvas(gt.x, gt.y);
        ctx.strokeStyle = "rgba(34,197,94,0.8)";
        ctx.setLineDash([3, 5]);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
        ctx.setLineDash([]);
      }
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

      if (tick.raw_estimate) {
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
    }
  }, [
    activeFieldHalf,
    currentWaypointIdx,
    flags,
    heatmapMode,
    obstacles,
    parsedFailures,
    prevGroundTruth,
    placementPose,
    selected,
    tick,
    navTarget,
    waypoints,
  ]);

  const kidnapActive = parsedFailures.some((f) => f.type === "kidnap");

  return (
    <canvas
      ref={canvasRef}
      onClick={(event) => {
        if (!onFieldClick) return;
        const canvas = canvasRef.current;
        if (!canvas) return;
        const rect = canvas.getBoundingClientRect();
        if (rect.width <= 0 || rect.height <= 0) return;
        const pad = 16;
        const span = activeFieldHalf * 2;
        const scale = Math.min((rect.width - 2 * pad) / span, (rect.height - 2 * pad) / span);
        const canvasX = event.clientX - rect.left;
        const canvasY = event.clientY - rect.top;
        const x = (canvasX - rect.width / 2) / scale;
        const y = (rect.height / 2 - canvasY) / scale;
        onFieldClick(x, y);
      }}
      className={`${className ?? "h-[640px] w-full rounded border border-zinc-700"} ${
        kidnapActive ? "border-red-500 shadow-[0_0_20px_rgba(239,68,68,0.45)]" : ""
      }`}
    />
  );
}
