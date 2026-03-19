"use client";

import { useEffect, useMemo, useState } from "react";

import type { Action } from "@/lib/types";

function keyToAction(key: string): Action | null {
  switch (key.toLowerCase()) {
    case "w":
    case "arrowup":
      return "forward";
    case "s":
    case "arrowdown":
      return "backward";
    case "d":
    case "arrowright":
      return "cw";
    case "a":
    case "arrowleft":
      return "ccw";
    default:
      return null;
  }
}

export function useKeyboard() {
  const [pressed, setPressed] = useState<Set<string>>(new Set());

  useEffect(() => {
    const onDown = (e: KeyboardEvent) => {
      const action = keyToAction(e.key);
      if (!action) return;
      setPressed((prev) => {
        const next = new Set(prev);
        next.add(action);
        return next;
      });
    };
    const onUp = (e: KeyboardEvent) => {
      const action = keyToAction(e.key);
      if (!action) return;
      setPressed((prev) => {
        const next = new Set(prev);
        next.delete(action);
        return next;
      });
    };
    window.addEventListener("keydown", onDown);
    window.addEventListener("keyup", onUp);
    return () => {
      window.removeEventListener("keydown", onDown);
      window.removeEventListener("keyup", onUp);
    };
  }, []);

  const currentAction = useMemo<Action>(() => {
    if (pressed.has("forward")) return "forward";
    if (pressed.has("backward")) return "backward";
    if (pressed.has("cw")) return "cw";
    if (pressed.has("ccw")) return "ccw";
    return "none";
  }, [pressed]);

  return { currentAction, pressedActions: Array.from(pressed) as Action[] };
}
