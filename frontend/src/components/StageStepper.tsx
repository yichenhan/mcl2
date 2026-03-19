"use client";

export type Stage = "post_predict" | "post_update" | "post_resample";

interface Props {
  stage: Stage;
  onStageChange: (stage: Stage) => void;
}

const OPTIONS: Stage[] = ["post_predict", "post_update", "post_resample"];

export function StageStepper({ stage, onStageChange }: Props) {
  return (
    <div className="rounded border border-zinc-700 p-3">
      <h3 className="mb-2 text-sm font-semibold">MCL Stage</h3>
      <div className="flex flex-col gap-2 text-sm">
        {OPTIONS.map((option) => (
          <label key={option} className="flex items-center gap-2 capitalize">
            <input
              type="radio"
              name="stage"
              checked={stage === option}
              onChange={() => onStageChange(option)}
            />
            {option.replace("_", " ")}
          </label>
        ))}
      </div>
    </div>
  );
}
