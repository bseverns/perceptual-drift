"""State management and mapping-curve helpers for the operator UI."""

from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

from software.swarm.mapping_loader import load_mapping


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _apply_deadzone(value: float, deadzone: float) -> float:
    if deadzone <= 0.0:
        return value
    if deadzone >= 1.0:
        return 0.0
    if abs(value) <= deadzone:
        return 0.0
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return math.copysign(scaled, value)


def _expo(value: float, strength: float) -> float:
    return math.copysign(abs(value) ** (1.0 + strength), value)


def shape_axis(axis_cfg: Dict[str, Any], value: float) -> float:
    deadzone = float(axis_cfg.get("deadzone", 0.0))
    deadzone = _clamp(deadzone, 0.0, 0.99)
    shaped = _apply_deadzone(value, deadzone)
    curve = str(axis_cfg.get("curve", "linear")).lower()
    if curve == "expo":
        strength = float(axis_cfg.get("expo_strength", axis_cfg.get("expo", 0.5)))
        shaped = _expo(shaped, strength)
    return _clamp(shaped, -1.0, 1.0)


class OperatorState:
    """Holds current operator-facing state for the web UI."""

    def __init__(self, base_mapping_path: Path, recipes_dir: Path) -> None:
        self.base_mapping_path = base_mapping_path
        self.recipes_dir = recipes_dir
        self._lock = threading.Lock()
        self._active_recipe: Optional[str] = None
        self._consent: int = 0
        self._updated_at: float = time.time()
        self._mapping = load_mapping(base_path=str(self.base_mapping_path))

    def list_recipes(self) -> List[Dict[str, str]]:
        recipes: List[Dict[str, str]] = [
            {
                "id": "base",
                "name": "Base Mapping",
                "description": "Use config/mapping.yaml without recipe overlay.",
            }
        ]
        for path in sorted(self.recipes_dir.glob("*.yaml")):
            entry = {"id": path.stem, "name": path.stem, "description": ""}
            try:
                parsed = yaml.safe_load(path.read_text()) or {}
            except yaml.YAMLError:
                recipes.append(entry)
                continue
            if isinstance(parsed, dict):
                entry["name"] = str(parsed.get("name", path.stem))
                entry["description"] = str(parsed.get("description", "")).strip()
            recipes.append(entry)
        return recipes

    def set_recipe(self, recipe_id: str) -> Dict[str, Any]:
        normalized = (recipe_id or "").strip()
        if not normalized or normalized == "base":
            mapping = load_mapping(base_path=str(self.base_mapping_path))
            active = None
        else:
            mapping = load_mapping(
                base_path=str(self.base_mapping_path),
                recipe_name=normalized,
                recipes_dir=str(self.recipes_dir),
            )
            active = normalized

        with self._lock:
            self._mapping = mapping
            self._active_recipe = active
            self._updated_at = time.time()
            return self.snapshot_unlocked()

    def set_consent(self, consent_value: Any) -> Dict[str, Any]:
        consent = 1 if float(consent_value) >= 0.5 else 0
        with self._lock:
            self._consent = consent
            self._updated_at = time.time()
            return self.snapshot_unlocked()

    def mapping_curves(self, points: int = 101) -> Dict[str, Any]:
        points = int(_clamp(points, 5, 401))
        with self._lock:
            mapping = dict(self._mapping.get("mapping", {}))

        x_values = [(-1.0 + 2.0 * idx / (points - 1)) for idx in range(points)]
        curves = {}
        for axis_name in ("altitude", "lateral"):
            axis_cfg = mapping.get(axis_name, {}) or {}
            gain = float(axis_cfg.get("gain", 1.0))
            series = []
            for x in x_values:
                shaped = shape_axis(axis_cfg, x)
                series.append({"x": x, "y": _clamp(shaped * gain, -1.0, 1.0)})
            curves[axis_name] = {
                "deadzone": float(axis_cfg.get("deadzone", 0.0)),
                "curve": str(axis_cfg.get("curve", "linear")),
                "gain": gain,
                "series": series,
            }

        yaw_cfg = mapping.get("yaw_bias", {}) or {}
        return {
            "curves": curves,
            "yaw_bias": {
                "bias": float(yaw_cfg.get("bias", 0.0)),
                "jitter": float(yaw_cfg.get("jitter", 0.0)),
            },
            "glitch_intensity": mapping.get("glitch_intensity", {}),
            "led_palette": mapping.get("leds", {}).get("palette", []),
        }

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return self.snapshot_unlocked()

    def snapshot_unlocked(self) -> Dict[str, Any]:
        osc_cfg = self._mapping.get("osc", {}) if isinstance(self._mapping, dict) else {}
        consent_cfg = (
            self._mapping.get("consent", {}) if isinstance(self._mapping, dict) else {}
        )
        return {
            "active_recipe": self._active_recipe or "base",
            "consent_state": self._consent,
            "updated_at": self._updated_at,
            "osc_port": int(osc_cfg.get("port", 9000)),
            "consent_mode": str(consent_cfg.get("mode", "binary")),
            "consent_gate_motion": bool(consent_cfg.get("gate_motion", True)),
        }
