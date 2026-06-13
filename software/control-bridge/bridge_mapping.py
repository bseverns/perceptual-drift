"""Mapping/config loading and curve-to-RC shaping."""

from __future__ import annotations

import math
import random
from pathlib import Path

import yaml

from bridge_consent import to_binary_state
from bridge_msp import clamp, map_float_to_rc
from config_validation import ModeSettings, ValidationError


def deep_merge(base, override):
    """Recursively merge ``override`` into ``base`` returning a new dict."""

    if not isinstance(base, dict) or not isinstance(override, dict):
        return override
    merged = dict(base)
    for key, value in override.items():
        nested_override = (
            key in merged
            and isinstance(merged[key], dict)
            and isinstance(value, dict)
        )
        if nested_override:
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_yaml(path):
    """Read YAML/JSON from ``path`` and return the parsed structure."""

    with open(path) as fh:
        return yaml.safe_load(fh)


def load_recipe(recipe_path):
    """Load a recipe file and return (control_cfg, metadata dict)."""

    recipe_path = Path(recipe_path).expanduser().resolve()
    data = load_yaml(recipe_path)
    if not isinstance(data, dict):
        raise ValueError(f"Recipe at {recipe_path} must parse into a mapping")

    control_section = (
        data.get("control_bridge")
        or data.get("control")
        or data.get("bridge")
        or {
            k: v
            for k, v in data.items()
            if k in {"mapping", "osc", "bridge", "extends"}
        }
    )
    if not control_section:
        raise ValueError(
            "Recipe {path} needs a 'control_bridge' section describing "
            "the MSP mapping".format(path=recipe_path)
        )

    base_cfg = {}
    extends = control_section.get("extends")
    if extends:
        base_path = (recipe_path.parent / extends).resolve()
        base_cfg = load_yaml(base_path) or {}

    overlay = {k: v for k, v in control_section.items() if k != "extends"}
    cfg = deep_merge(base_cfg, overlay)

    metadata = {
        "name": data.get("name", recipe_path.stem),
        "slug": data.get("slug"),
        "description": data.get("description", ""),
        "intent": data.get("intent", ""),
        "leds": data.get("leds", {}),
        "video_pipeline": data.get("video_pipeline", {}),
    }
    return cfg, metadata


def resolve_mode_settings(cfg, requested_mode=None):
    """Return ModeSettings for the requested or default bridge mode."""

    bridge_cfg = cfg.get("bridge", {}) or {}
    modes_cfg = bridge_cfg.get("modes") or {}
    mode_name = requested_mode or bridge_cfg.get("mode", "smooth")
    if not modes_cfg:
        return ModeSettings(
            name=mode_name,
            description="",
            deadzone_boost={},
            gain_scale={},
            jitter_scale=1.0,
            neutral_rc=False,
            aux_strategy="full",
        )
    if mode_name not in modes_cfg:
        raise ValidationError(
            [f"bridge.mode '{mode_name}' missing from bridge.modes section"]
        )
    mode_cfg = modes_cfg[mode_name] or {}
    return ModeSettings(
        name=mode_name,
        description=mode_cfg.get("description", ""),
        deadzone_boost=mode_cfg.get("deadzone_boost", {}),
        gain_scale=mode_cfg.get("gain_scale", {}),
        jitter_scale=float(mode_cfg.get("jitter_scale", 1.0)),
        neutral_rc=bool(mode_cfg.get("neutral_rc", False)),
        aux_strategy=str(mode_cfg.get("aux_strategy", "full")),
    )


def resolve_bridge_rate(cfg, requested_hz=None):
    bridge_cfg = cfg.get("bridge", {}) or {}
    hz = requested_hz or bridge_cfg.get("hz", 50)
    try:
        hz = float(hz)
    except (TypeError, ValueError):
        raise ValidationError(["bridge.hz must be numeric"])
    if hz <= 0:
        raise ValidationError(["bridge.hz must be > 0"])
    return hz


class Mapper:
    def __init__(self, cfg, *, mode: ModeSettings | None = None):
        self.cfg = cfg
        self.mode = mode or resolve_mode_settings(cfg)
        consent_cfg = cfg.get("consent", {}) or {}
        default_consent = to_binary_state(consent_cfg.get("default_state", 0))
        self.state = {
            "alt": 0.0,
            "lat": 0.0,
            "yaw": 0.0,
            "crowd": 0.0,
            "consent": default_consent,
        }

    def set_mode(self, mode: ModeSettings) -> None:
        self.mode = mode

    def expo(self, x, k=0.5):
        """Push the midpoint flatter while keeping the sign intact."""

        return math.copysign((abs(x) ** (1 + k)), x)

    def apply_deadzone(self, value, deadzone):
        """Snap tiny inputs to zero while preserving full-throw range."""

        if deadzone <= 0:
            return value
        if deadzone >= 1:
            return 0.0
        if abs(value) <= deadzone:
            return 0.0
        scaled = (abs(value) - deadzone) / (1 - deadzone)
        return math.copysign(scaled, value)

    def shape_axis(self, axis_name, value):
        """Apply deadzone + curve shaping from the mapping config."""

        axis_cfg = self.cfg["mapping"].get(axis_name, {})
        deadzone = axis_cfg.get("deadzone", 0.0)
        deadzone += self.mode.deadzone_for(axis_name)
        deadzone = clamp(deadzone, 0.0, 0.99)
        shaped = self.apply_deadzone(value, deadzone)
        curve = axis_cfg.get("curve", "linear")
        if curve == "expo":
            strength = axis_cfg.get("expo_strength", axis_cfg.get("expo", 0.5))
            shaped = self.expo(shaped, strength)
        return clamp(shaped, -1.0, 1.0)

    def apply(self):
        """Convert the current sensor state into RC + AUX channel values."""

        mapping_cfg = self.cfg.get("mapping", {})

        alt = self.shape_axis("altitude", self.state["alt"])
        lat = self.shape_axis("lateral", self.state["lat"])

        yaw_cfg = mapping_cfg.get("yaw_bias", {})
        yaw_bias = yaw_cfg.get("bias", 0.0)
        jitter = yaw_cfg.get("jitter", 0.0) * self.mode.jitter_scale
        if jitter:
            yaw_bias += random.uniform(-jitter, jitter)
        yaw = clamp(self.state["yaw"] + yaw_bias, -1.0, 1.0)

        lat_gain = mapping_cfg.get("lateral", {}).get("gain", 1.0)
        lat_gain *= self.mode.gain_for("lateral", 1.0)
        alt_gain = mapping_cfg.get("altitude", {}).get("gain", 1.0)
        alt_gain *= self.mode.gain_for("altitude", 1.0)

        rc_roll = map_float_to_rc(lat, gain=lat_gain)
        rc_pitch = map_float_to_rc(-abs(lat) * 0.2)
        rc_thr = map_float_to_rc(alt / 2, gain=alt_gain)
        rc_yaw = map_float_to_rc(yaw)

        crowd = clamp(self.state["crowd"], 0.0, 1.0)
        aux_crowd = map_float_to_rc(crowd * 2 - 1)

        glitch_cfg = mapping_cfg.get("glitch_intensity", {})
        glitch_base = glitch_cfg.get("base", 0.0)
        glitch_max = glitch_cfg.get("max", 1.0)
        glitch_norm = clamp(
            glitch_base + (glitch_max - glitch_base) * crowd,
            0.0,
            1.0,
        )
        aux_glitch = map_float_to_rc(glitch_norm * 2 - 1)

        palette = mapping_cfg.get("leds", {}).get("palette", [])
        if palette:
            if len(palette) == 1:
                led_norm = 0.0
            else:
                slot = int(round(crowd * (len(palette) - 1)))
                slot = clamp(slot, 0, len(palette) - 1)
                led_norm = slot / (len(palette) - 1)
        else:
            led_norm = 0.0
        aux_led = map_float_to_rc(led_norm * 2 - 1)

        aux_channels = [aux_crowd, aux_glitch, aux_led, 1500]
        rc_channels = [rc_roll, rc_pitch, rc_thr, rc_yaw]

        if self.mode.neutral_rc:
            rc_channels = [
                map_float_to_rc(0.0),
                map_float_to_rc(0.0),
                map_float_to_rc(-1.0),
                map_float_to_rc(0.0),
            ]
        if self.mode.aux_strategy == "crowd_only":
            aux_channels = [aux_crowd, 1500, 1500, 1500]

        return rc_channels, aux_channels
