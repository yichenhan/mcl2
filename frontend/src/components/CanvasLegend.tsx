"use client";

export function CanvasLegend() {
  return (
    <div className="rounded border border-zinc-700 p-3 text-xs">
      <h3 className="mb-2 text-sm font-semibold">Canvas Legend</h3>
      <div className="space-y-1 text-zinc-300">
        <div>
          <span className="font-mono text-emerald-300">▲</span> Odom pose
        </div>
        <div>
          <span className="font-mono text-amber-300">- - -</span> Sensor readings
        </div>
        <div>
          <span className="font-mono text-orange-300">o→</span> Raw estimate heading
        </div>
        <div>
          <span className="font-mono text-zinc-200">── d=#.# in</span> MCL diff
        </div>
      </div>
    </div>
  );
}
