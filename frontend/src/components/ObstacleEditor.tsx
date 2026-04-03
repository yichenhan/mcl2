"use client";

import { useState } from "react";

import type { AABB, Obstacle } from "@/lib/types";

interface Props {
  obstacles: Obstacle[];
  onChange: (obstacles: Obstacle[]) => void;
}

const DEFAULT_BOX: AABB = { min_x: -10, min_y: -10, max_x: 10, max_y: 10 };

export function ObstacleEditor({ obstacles, onChange }: Props) {
  const [draft, setDraft] = useState<AABB>(DEFAULT_BOX);

  const addObstacle = () => {
    onChange([...obstacles, draft]);
  };

  const removeObstacle = (idx: number) => {
    onChange(obstacles.filter((_, i) => i !== idx));
  };

  return (
    <div className="rounded border border-zinc-700 p-3 text-sm">
      <h3 className="mb-2 text-sm font-semibold">Obstacle Editor</h3>
      <div className="grid grid-cols-2 gap-2">
        {(["min_x", "min_y", "max_x", "max_y"] as const).map((k) => (
          <label key={k} className="flex flex-col gap-1">
            <span className="text-xs uppercase text-zinc-400">{k}</span>
            <input
              className="rounded border border-zinc-600 bg-zinc-900 px-2 py-1"
              type="number"
              value={draft[k]}
              onChange={(e) =>
                setDraft((prev) => ({ ...prev, [k]: Number(e.target.value) }))
              }
            />
          </label>
        ))}
      </div>
      <button
        type="button"
        className="mt-3 rounded bg-zinc-200 px-3 py-1 text-black"
        onClick={addObstacle}
      >
        Add Obstacle
      </button>
      <div className="mt-3 space-y-1 text-xs">
        {obstacles.map((o, idx) => (
          <div key={idx} className="flex items-center justify-between rounded bg-zinc-900 p-1">
            <span>
              {"min_x" in o
                ? `[${o.min_x}, ${o.min_y}] -> [${o.max_x}, ${o.max_y}]`
                : o.type === "circle"
                  ? `circle (${o.cx}, ${o.cy}) r=${o.radius}`
                  : `triangle (${o.x0}, ${o.y0})`}
            </span>
            <button
              type="button"
              className="rounded bg-red-600 px-2 py-0.5 text-white"
              onClick={() => removeObstacle(idx)}
            >
              Remove
            </button>
          </div>
        ))}
      </div>
    </div>
  );
}
