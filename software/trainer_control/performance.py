"""Validated, ephemeral musician-facing performance overlays.

This module intentionally knows nothing about aircraft, transmitter, ARM,
throttle, heartbeat, or SafetyEnvelope configuration.  Its tiny allowlist is
the boundary that keeps a performance gesture artistic.
"""

from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MACROS_PATH = REPO_ROOT / "config" / "performance_macros.yaml"
ALLOWED_TARGETS = frozenset(
    {
        "mapping.lateral.gain",
        "mapping.lateral.deadzone",
        "mapping.lateral.expo_strength",
        "mapping.yaw_bias.bias",
        "mapping.yaw_bias.jitter",
        "mapping.glitch_intensity.base",
        "mapping.glitch_intensity.max",
    }
)


@dataclass(frozen=True)
class Macro:
    id: str
    label: str
    type: str
    default: float
    description: str
    targets: tuple[tuple[str, float, float], ...]


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def load_macros(
    path: Path = DEFAULT_MACROS_PATH,
) -> tuple[dict[str, Macro], int]:
    """Load the small declarative macro schema and reject any unsafe target."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, Mapping) or not isinstance(
        data.get("macros"), Mapping
    ):
        raise ValueError("performance macro config requires a macros mapping")
    slew_ms = int(_number(data.get("slew_ms", 220), "slew_ms"))
    if not 0 <= slew_ms <= 1000:
        raise ValueError("slew_ms must be between 0 and 1000")
    macros: dict[str, Macro] = {}
    for macro_id, raw in data["macros"].items():
        if (
            not isinstance(macro_id, str)
            or not macro_id.replace("_", "").isalnum()
        ):
            raise ValueError(
                "macro id must contain only letters, digits, and underscores"
            )
        if not isinstance(raw, Mapping):
            raise ValueError(f"macro {macro_id} must be a mapping")
        kind = raw.get("type")
        if kind not in {"unipolar", "bipolar"}:
            raise ValueError(f"macro {macro_id} has invalid type")
        default = _number(raw.get("default", 0.0), f"macro {macro_id}.default")
        low, high = (0.0, 1.0) if kind == "unipolar" else (-1.0, 1.0)
        if not low <= default <= high:
            raise ValueError(f"macro {macro_id}.default is outside its range")
        targets = raw.get("targets")
        if not isinstance(targets, list) or not targets:
            raise ValueError(f"macro {macro_id} requires targets")
        parsed_targets = []
        for target in targets:
            if not isinstance(target, Mapping):
                raise ValueError(f"macro {macro_id} target must be a mapping")
            target_path = target.get("path")
            if target_path not in ALLOWED_TARGETS:
                raise ValueError(
                    f"macro {macro_id} targets forbidden path {target_path!r}"
                )
            parsed_targets.append(
                (
                    target_path,
                    _number(target.get("min"), "target.min"),
                    _number(target.get("max"), "target.max"),
                )
            )
        macros[macro_id] = Macro(
            macro_id,
            str(raw.get("label", macro_id)),
            kind,
            default,
            str(raw.get("description", "")),
            tuple(parsed_targets),
        )
    return macros, slew_ms


def _set_path(mapping: dict[str, Any], path: str, value: float) -> None:
    cursor = mapping
    chunks = path.split(".")
    for chunk in chunks[:-1]:
        child = cursor.get(chunk)
        if not isinstance(child, dict):
            child = {}
            cursor[chunk] = child
        cursor = child
    cursor[chunks[-1]] = value


class PerformanceOverlay:
    """Thread-safe macro state with bounded artistic interpolation only."""

    def __init__(
        self, macros_path: Path = DEFAULT_MACROS_PATH, *, clock=time.monotonic
    ):
        self.macros, self.slew_ms = load_macros(macros_path)
        self.clock = clock
        self._lock = threading.RLock()
        self._current = {
            key: macro.default for key, macro in self.macros.items()
        }
        self._target = dict(self._current)
        self._enabled: set[str] = set()
        self._changed_at = self.clock()

    def set(self, values: Mapping[str, Any]) -> dict[str, float]:
        if not isinstance(values, Mapping):
            raise ValueError("macro update must be an object")
        # Validate the whole update before taking effect.
        # Malformed input leaves the prior valid state intact.
        parsed: dict[str, float] = {}
        for macro_id, value in values.items():
            macro = self.macros.get(str(macro_id))
            if macro is None:
                raise ValueError(f"unknown performance macro {macro_id!r}")
            numeric = _number(value, f"macro {macro_id}")
            low, high = (0.0, 1.0) if macro.type == "unipolar" else (-1.0, 1.0)
            if not low <= numeric <= high:
                raise ValueError(f"macro {macro_id} is outside its range")
            parsed[macro_id] = numeric
        with self._lock:
            self._advance_unlocked(self.clock())
            self._target.update(parsed)
            self._enabled.update(parsed)
            self._changed_at = self.clock()
            return dict(self._target)

    def reset(self) -> dict[str, float]:
        with self._lock:
            self._current = {
                key: macro.default for key, macro in self.macros.items()
            }
            self._target = dict(self._current)
            self._enabled.clear()
            self._changed_at = self.clock()
            return dict(self._target)

    def _advance_unlocked(self, now: float) -> None:
        duration = self.slew_ms / 1000.0
        if duration <= 0:
            self._current = dict(self._target)
            return
        progress = max(0.0, min(1.0, (now - self._changed_at) / duration))
        for key in self._current:
            self._current[key] += (
                self._target[key] - self._current[key]
            ) * progress
        if progress < 1.0:
            self._changed_at = now

    def apply(
        self, base: Mapping[str, Any], *, now: float | None = None
    ) -> dict[str, Any]:
        with self._lock:
            self._advance_unlocked(self.clock() if now is None else now)
            values = dict(self._current)
        result = copy.deepcopy(dict(base))
        for macro_id, macro in self.macros.items():
            if macro_id not in self._enabled:
                continue
            value = values[macro_id]
            normalized = (
                value if macro.type == "unipolar" else (value + 1.0) / 2.0
            )
            for target_path, minimum, maximum in macro.targets:
                _set_path(
                    result,
                    target_path,
                    minimum + (maximum - minimum) * normalized,
                )
        return result

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            self._advance_unlocked(self.clock())
            return {
                "slew_ms": self.slew_ms,
                "values": dict(self._current),
                "targets": dict(self._target),
                "active": sorted(self._enabled),
                "macros": [
                    {
                        "id": item.id,
                        "label": item.label,
                        "type": item.type,
                        "default": item.default,
                        "description": item.description,
                    }
                    for item in self.macros.values()
                ],
            }
