import { toTickState } from "@/lib/tick";
import type { TickState } from "@/lib/types";

const MCL_JSON_SEGMENT_RE = /\[MCL_JSON_START\]\s*([\s\S]*?)\s*\[MCL_JSON_FINISH\]/g;
const TICK_KV_RE = /^\[TICK=(\d+)\s*\|\|\s*([^\]]+)\]=(.*)$/;

function setNestedValue(obj: Record<string, unknown>, path: string, raw: string): void {
  const keys = path.split(".");
  let cur: Record<string, unknown> = obj;
  for (let i = 0; i < keys.length - 1; i++) {
    const k = keys[i];
    if (!(k in cur) || typeof cur[k] !== "object" || cur[k] === null) {
      const nextKey = keys[i + 1];
      cur[k] = /^\d+$/.test(nextKey) ? [] : {};
    }
    cur = cur[k] as Record<string, unknown>;
  }
  const leaf = keys[keys.length - 1];
  let value: unknown = raw;
  if (raw === "true") value = true;
  else if (raw === "false") value = false;
  else if (raw === "null") value = null;
  else if (raw.startsWith("\"") && raw.endsWith("\"")) {
    try {
      value = JSON.parse(raw);
    } catch {
      value = raw;
    }
  } else {
    const n = Number(raw);
    if (!Number.isNaN(n)) value = n;
  }
  if (Array.isArray(cur)) {
    cur[Number(leaf)] = value;
  } else {
    cur[leaf] = value;
  }
}

function parseTickKVLines(text: string): unknown[] {
  const tickMap = new Map<number, Record<string, unknown>>();
  for (const line of text.split("\n")) {
    const m = TICK_KV_RE.exec(line.trim());
    if (!m) continue;
    const tickNum = Number(m[1]);
    const key = m[2].trim();
    const val = m[3];
    if (!tickMap.has(tickNum)) tickMap.set(tickNum, {});
    setNestedValue(tickMap.get(tickNum)!, key, val);
  }
  return [...tickMap.entries()].sort((a, b) => a[0] - b[0]).map(([, obj]) => obj);
}

export function parseReplayText(text: string): TickState[] {
  try {
    const parsed = JSON.parse(text) as unknown;
    if (Array.isArray(parsed)) return parsed.map(toTickState);
    if (parsed && typeof parsed === "object" && Array.isArray((parsed as { ticks?: unknown[] }).ticks)) {
      return ((parsed as { ticks: unknown[] }).ticks).map(toTickState);
    }
  } catch {
    // Fall through to text formats.
  }

  const kvTicks = parseTickKVLines(text);
  if (kvTicks.length > 0) return kvTicks.map(toTickState);

  const segmentTicks: unknown[] = [];
  for (const match of text.matchAll(MCL_JSON_SEGMENT_RE)) {
    const raw = (match[1] ?? "").trim();
    if (!raw) continue;
    try {
      segmentTicks.push(JSON.parse(raw) as unknown);
    } catch {
      // Ignore malformed segment.
    }
  }
  if (segmentTicks.length > 0) return segmentTicks.map(toTickState);

  throw new Error(
    "Could not parse replay. Supported formats: JSON ticks, [TICK=..||..]=.. lines, or [MCL_JSON_START] segments.",
  );
}
