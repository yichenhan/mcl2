import type { SessionAnalytics } from "./types";

export type Grade = "great" | "good" | "fair" | "poor" | "bad";

export interface FieldGrade {
  field: keyof SessionAnalytics;
  label: string;
  value: number;
  grade: Grade;
}

export interface SessionGrade {
  overall: Grade;
  fields: FieldGrade[];
  summary: string;
}

type Rule = { great: number; good: number; fair: number; poor: number; lowerIsBetter?: boolean };

/** Thresholds: for `lowerIsBetter`, value ≤ great is great; else value ≥ great is great (fractions 0–1). */
const RULES: Partial<Record<keyof SessionAnalytics, Rule>> = {
  peak_accepted_error: { great: 6, good: 12, fair: 18, poor: 24, lowerIsBetter: true },
  mean_accepted_error: { great: 4, good: 8, fair: 12, poor: 16, lowerIsBetter: true },
  p95_accepted_error: { great: 10, good: 16, fair: 22, poor: 30, lowerIsBetter: true },
  gate_precision: { great: 0.95, good: 0.9, fair: 0.85, poor: 0.75 },
  gate_recall: { great: 0.95, good: 0.9, fair: 0.85, poor: 0.75 },
  false_accept_count: { great: 0, good: 2, fair: 5, poor: 10, lowerIsBetter: true },
  longest_false_accept_streak: { great: 0, good: 3, fair: 8, poor: 15, lowerIsBetter: true },
  false_reject_count: { great: 0, good: 5, fair: 15, poor: 30, lowerIsBetter: true },
  longest_rejection_streak: { great: 0, good: 10, fair: 25, poor: 50, lowerIsBetter: true },
  cold_start_convergence_ticks: { great: 30, good: 60, fair: 100, poor: 150, lowerIsBetter: true },
  worst_reacquisition_ticks: { great: 20, good: 40, fair: 70, poor: 100, lowerIsBetter: true },
  mean_reacquisition_ticks: { great: 15, good: 30, fair: 50, poor: 80, lowerIsBetter: true },
  post_convergence_error_stddev: { great: 2, good: 4, fair: 6, poor: 10, lowerIsBetter: true },
  neff_collapse_count: { great: 0, good: 2, fair: 5, poor: 10, lowerIsBetter: true },
  sensor_dropout_rate: { great: 0.02, good: 0.05, fair: 0.1, poor: 0.2, lowerIsBetter: true },
};

const LABELS: Partial<Record<keyof SessionAnalytics, string>> = {
  peak_accepted_error: "Peak accepted error (in)",
  mean_accepted_error: "Mean accepted error (in)",
  p95_accepted_error: "P95 accepted error (in)",
  gate_precision: "Gate precision",
  gate_recall: "Gate recall",
  false_accept_count: "False accepts",
  longest_false_accept_streak: "Longest false-accept streak (ticks)",
  false_reject_count: "False rejects",
  longest_rejection_streak: "Longest rejection streak (ticks)",
  cold_start_convergence_ticks: "Cold-start convergence (ticks)",
  worst_reacquisition_ticks: "Worst reacquisition (ticks)",
  mean_reacquisition_ticks: "Mean reacquisition (ticks)",
  post_convergence_error_stddev: "Post-convergence error σ (in)",
  neff_collapse_count: "N_eff collapse events",
  sensor_dropout_rate: "Sensor dropout rate",
};

const GRADE_ORDER: Record<Grade, number> = {
  great: 4,
  good: 3,
  fair: 2,
  poor: 1,
  bad: 0,
};

function gradeFromRule(value: number, rule: Rule): Grade {
  const { great, good, fair, poor, lowerIsBetter } = rule;
  if (lowerIsBetter) {
    if (value <= great) return "great";
    if (value <= good) return "good";
    if (value <= fair) return "fair";
    if (value <= poor) return "poor";
    return "bad";
  }
  if (value >= great) return "great";
  if (value >= good) return "good";
  if (value >= fair) return "fair";
  if (value >= poor) return "poor";
  return "bad";
}

function minGrade(a: Grade, b: Grade): Grade {
  return GRADE_ORDER[a] < GRADE_ORDER[b] ? a : b;
}

export function gradeField(field: keyof SessionAnalytics, value: number): Grade {
  const rule = RULES[field];
  if (!rule) return "good";
  return gradeFromRule(value, rule);
}

export function gradeSession(analytics: SessionAnalytics): SessionGrade {
  const fields: FieldGrade[] = [];
  for (const key of Object.keys(RULES) as (keyof SessionAnalytics)[]) {
    const rule = RULES[key];
    if (!rule) continue;
    const value = analytics[key];
    const grade = gradeFromRule(value, rule);
    fields.push({
      field: key,
      label: LABELS[key] ?? key,
      value,
      grade,
    });
  }
  let overall: Grade = "great";
  for (const f of fields) overall = minGrade(overall, f.grade);

  const summaryParts: string[] = [];
  if (analytics.false_accept_count > 0) {
    summaryParts.push(`${analytics.false_accept_count} false accept(s)`);
  }
  if (analytics.false_reject_count > 0) {
    summaryParts.push(`${analytics.false_reject_count} false reject(s)`);
  }
  if (analytics.peak_accepted_error > 12) {
    summaryParts.push(`peak error ${analytics.peak_accepted_error.toFixed(1)} in`);
  }
  const summary =
    summaryParts.length > 0
      ? summaryParts.join(" · ")
      : "No major issues flagged by heuristics.";

  return { overall, fields, summary };
}
