"""Transport-neutral artistic shaping for tracker control intent."""

from __future__ import annotations

import copy
import importlib.util
import math
import random
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping, Optional

import yaml

from .intent import ControlIntent


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RECIPES_DIR = REPO_ROOT / "config/recipes"


@dataclass(frozen=True)
class TrackerSignals:
    """Fresh normalized tracker values before artistic shaping."""

    lateral: float
    yaw: float
    crowd: float
    consent: bool
    timestamp: float


def _clamp(value: float, low: float, high: float) -> float:
    if not math.isfinite(value):
        raise ValueError("mapping input must be finite")
    return max(low, min(high, value))


def _deep_merge(base: Mapping, overlay: Mapping) -> dict:
    merged = copy.deepcopy(dict(base))
    for key, value in overlay.items():
        if isinstance(merged.get(key), Mapping) and isinstance(value, Mapping):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _load_yaml(path: Path) -> Mapping:
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, Mapping):
        raise ValueError(f"mapping must be a YAML object: {path}")
    return loaded


def _validator_module():
    name = "trainer_control_config_validation"
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    path = REPO_ROOT / "software/control-bridge/config_validation.py"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load mapping validator: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_mapping_config(
    base_path: Path, recipe_path: Optional[Path] = None
) -> dict:
    """Load and validate a base mapping plus an optional recipe overlay."""

    base_path = base_path.expanduser().resolve()
    config = dict(_load_yaml(base_path))
    source = base_path
    if recipe_path is not None:
        recipe_path = recipe_path.expanduser().resolve()
        recipe = _load_yaml(recipe_path)
        control = recipe.get("control_bridge")
        if not isinstance(control, Mapping):
            raise ValueError(
                f"recipe lacks a control_bridge mapping: {recipe_path}"
            )
        extends = control.get("extends")
        if extends:
            config = dict(
                _load_yaml((recipe_path.parent / str(extends)).resolve())
            )
        overlay = {
            key: value for key, value in control.items() if key != "extends"
        }
        config = _deep_merge(config, overlay)
        source = recipe_path
    validator = _validator_module()
    try:
        validator.validate_mapping_config(config, str(source))
    except validator.ValidationError as exc:
        raise ValueError(str(exc)) from exc
    return config


class IntentMapper:
    """Apply expressive mapping while preserving the physical safety boundary."""

    def __init__(
        self,
        config: Mapping,
        *,
        uniform: Callable[[float, float], float] = random.uniform,
    ) -> None:
        self.config = config
        self.uniform = uniform

    @staticmethod
    def _shape_axis(
        axis: Mapping,
        value: float,
        *,
        deadzone_boost: float = 0.0,
        gain_scale: float = 1.0,
    ) -> float:
        deadzone = _clamp(
            float(axis.get("deadzone", 0.0)) + deadzone_boost,
            0.0,
            0.99,
        )
        if abs(value) <= deadzone:
            shaped = 0.0
        else:
            shaped = math.copysign(
                (abs(value) - deadzone) / (1.0 - deadzone), value
            )
        if str(axis.get("curve", "linear")).lower() == "expo":
            strength = float(axis.get("expo_strength", axis.get("expo", 0.5)))
            shaped = math.copysign(abs(shaped) ** (1.0 + strength), shaped)
        gain = float(axis.get("gain", 1.0)) * gain_scale
        return _clamp(shaped * gain, -1.0, 1.0)

    def map(self, signals: TrackerSignals) -> ControlIntent:
        mapping = self.config.get("mapping", {})
        bridge = self.config.get("bridge", {})
        mode_name = bridge.get("mode", "smooth")
        mode = bridge.get("modes", {}).get(mode_name, {})
        if mode.get("neutral_rc", False):
            return ControlIntent(
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                throttle=0.0,
                crowd=_clamp(signals.crowd, 0.0, 1.0),
                consent=signals.consent,
                timestamp=signals.timestamp,
            )
        lateral = self._shape_axis(
            mapping.get("lateral", {}),
            signals.lateral,
            deadzone_boost=float(
                mode.get("deadzone_boost", {}).get("lateral", 0.0)
            ),
            gain_scale=float(mode.get("gain_scale", {}).get("lateral", 1.0)),
        )
        yaw_config = mapping.get("yaw_bias", {})
        yaw = signals.yaw + float(yaw_config.get("bias", 0.0))
        jitter = float(yaw_config.get("jitter", 0.0)) * float(
            mode.get("jitter_scale", 1.0)
        )
        if jitter:
            yaw += self.uniform(-jitter, jitter)
        yaw *= float(mode.get("gain_scale", {}).get("yaw", 1.0))
        return ControlIntent(
            roll=_clamp(lateral, -1.0, 1.0),
            pitch=0.0,
            yaw=_clamp(yaw, -1.0, 1.0),
            throttle=0.0,
            crowd=_clamp(signals.crowd, 0.0, 1.0),
            consent=signals.consent,
            timestamp=signals.timestamp,
        )


class MappingController:
    """Thread-safe base/recipe mapper selected by CLI or OSC patch events."""

    def __init__(
        self,
        base_path: Path,
        recipes_dir: Path = DEFAULT_RECIPES_DIR,
        recipe: Optional[str] = None,
        *,
        uniform: Callable[[float, float], float] = random.uniform,
    ) -> None:
        self.base_path = base_path
        self.recipes_dir = recipes_dir
        self.uniform = uniform
        self._lock = threading.Lock()
        self.active_recipe = "base"
        self._mapper = IntentMapper(
            load_mapping_config(base_path), uniform=self.uniform
        )
        if recipe and recipe != "base":
            self.select(recipe)

    def _recipe_path(self, recipe: str) -> Path:
        candidate = Path(recipe)
        if candidate.suffix or candidate.parent != Path("."):
            return candidate
        return self.recipes_dir / f"{recipe}.yaml"

    def select(self, recipe: str) -> str:
        normalized = str(recipe).strip()
        if not normalized or normalized in {"base", "none"}:
            config = load_mapping_config(self.base_path)
            active = "base"
        else:
            config = load_mapping_config(
                self.base_path, self._recipe_path(normalized)
            )
            active = normalized
        replacement = IntentMapper(config, uniform=self.uniform)
        with self._lock:
            self._mapper = replacement
            self.active_recipe = active
        return active

    def map(self, signals: TrackerSignals) -> ControlIntent:
        with self._lock:
            mapper = self._mapper
        return mapper.map(signals)
