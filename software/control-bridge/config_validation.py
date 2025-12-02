"""Config validation helpers for the OSC→MSP bridge."""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Iterable, Mapping, MutableMapping

import yaml


class ValidationError(Exception):
    """Aggregates config validation failures."""

    def __init__(self, errors: Iterable[str]):
        messages = list(errors)
        super().__init__("; ".join(messages))
        self.errors = messages


@dataclasses.dataclass
class ModeSettings:
    name: str
    description: str
    deadzone_boost: Mapping[str, float]
    gain_scale: Mapping[str, float]
    jitter_scale: float
    neutral_rc: bool
    aux_strategy: str

    def deadzone_for(self, axis: str, default: float = 0.0) -> float:
        return float(self.deadzone_boost.get(axis, default))

    def gain_for(self, axis: str, default: float = 1.0) -> float:
        return float(self.gain_scale.get(axis, default))


REQUIRED_OSC_PATHS = {"altitude", "lateral", "yaw", "crowd", "consent"}
REQUIRED_AXES = {"altitude", "lateral"}
VALID_CURVES = {"linear", "expo"}
VALID_AUX_STRATEGIES = {"full", "crowd_only"}


# ---- validation primitives -------------------------------------------------


def _require_mapping(
    section: Mapping, key: str, path: str, errors: list[str]
) -> Mapping:
    if key not in section:
        errors.append(f"missing required key '{path}.{key}'")
        return {}
    if not isinstance(section[key], Mapping):
        errors.append(f"'{path}.{key}' must be a mapping")
        return {}
    return section[key]


def _require_number(
    section: Mapping,
    key: str,
    path: str,
    *,
    minimum=None,
    maximum=None,
    errors: list[str],
):
    if key not in section:
        errors.append(f"missing required key '{path}.{key}'")
        return None
    value = section[key]
    if not isinstance(value, (int, float)):
        errors.append(f"'{path}.{key}' must be a number")
        return None
    if minimum is not None and value < minimum:
        errors.append(f"'{path}.{key}' must be >= {minimum}")
    if maximum is not None and value > maximum:
        errors.append(f"'{path}.{key}' must be <= {maximum}")
    return value


def _check_palette(section: Mapping, path: str, errors: list[str]) -> None:
    palette = section.get("palette")
    if palette is None:
        return
    if not isinstance(palette, list):
        errors.append(f"'{path}.palette' must be a list of color strings")
        return
    if not palette:
        errors.append(f"'{path}.palette' must contain at least one entry when provided")


def validate_bridge_modes(bridge_cfg: Mapping, source: str, errors: list[str]) -> None:
    modes = bridge_cfg.get("modes")
    if not isinstance(modes, Mapping) or not modes:
        errors.append(f"{source}: bridge.modes must be a non-empty mapping")
        return
    for mode_name, mode_cfg in modes.items():
        mode_path = f"bridge.modes.{mode_name}"
        if not isinstance(mode_cfg, Mapping):
            errors.append(f"{source}: {mode_path} must be a mapping")
            continue
        aux_strategy = mode_cfg.get("aux_strategy", "full")
        if aux_strategy not in VALID_AUX_STRATEGIES:
            allowed_aux = sorted(VALID_AUX_STRATEGIES)
            errors.append(
                f"{source}: {mode_path}.aux_strategy must be one of {allowed_aux}"
            )
        neutral = mode_cfg.get("neutral_rc", False)
        if not isinstance(neutral, bool):
            errors.append(f"{source}: {mode_path}.neutral_rc must be true/false")
        jitter_scale = mode_cfg.get("jitter_scale", 1.0)
        if not isinstance(jitter_scale, (int, float)) or jitter_scale < 0:
            errors.append(f"{source}: {mode_path}.jitter_scale must be >= 0")
        dz_boost = mode_cfg.get("deadzone_boost", {})
        if dz_boost:
            if not isinstance(dz_boost, Mapping):
                errors.append(
                    f"{source}: {mode_path}.deadzone_boost must be a mapping "
                    "of axis→value"
                )
            else:
                for axis, val in dz_boost.items():
                    if not isinstance(val, (int, float)) or not (0 <= val < 1):
                        errors.append(
                            f"{source}: {mode_path}.deadzone_boost.{axis} "
                            "must be in [0,1)"
                        )
        gain_scale = mode_cfg.get("gain_scale", {})
        if gain_scale:
            if not isinstance(gain_scale, Mapping):
                errors.append(
                    f"{source}: {mode_path}.gain_scale must be a mapping of axis→value"
                )
            else:
                for axis, val in gain_scale.items():
                    if not isinstance(val, (int, float)):
                        errors.append(
                            f"{source}: {mode_path}.gain_scale.{axis} must be numeric"
                        )
                    elif val < 0:
                        errors.append(
                            f"{source}: {mode_path}.gain_scale.{axis} must be >= 0"
                        )


def validate_mapping_config(cfg: Mapping, source: str = "mapping") -> None:
    errors: list[str] = []
    if not isinstance(cfg, Mapping):
        raise ValidationError([f"{source}: config must be a mapping"])

    osc = _require_mapping(cfg, "osc", source, errors)
    address_space = _require_mapping(
        osc,
        "address_space",
        f"{source}.osc",
        errors,
    )
    for path_name in REQUIRED_OSC_PATHS:
        if path_name not in address_space:
            errors.append(f"{source}: osc.address_space missing '{path_name}' path")
        elif not isinstance(address_space[path_name], str):
            errors.append(f"{source}: osc.address_space.{path_name} must be a string")

    mapping = _require_mapping(cfg, "mapping", source, errors)
    for axis in REQUIRED_AXES:
        axis_cfg = _require_mapping(
            mapping,
            axis,
            f"{source}.mapping",
            errors,
        )
        if axis_cfg:
            dz = _require_number(
                axis_cfg,
                "deadzone",
                f"{source}.mapping.{axis}",
                minimum=0,
                maximum=0.99,
                errors=errors,
            )
            curve = axis_cfg.get("curve", "linear")
            if curve not in VALID_CURVES:
                allowed_curves = sorted(VALID_CURVES)
                errors.append(
                    f"{source}.mapping.{axis}.curve must be one of {allowed_curves}"
                )
            gain = axis_cfg.get("gain", 1.0)
            if not isinstance(gain, (int, float)) or gain < 0:
                errors.append(f"{source}.mapping.{axis}.gain must be >= 0")
            expo_strength = axis_cfg.get(
                "expo_strength",
                axis_cfg.get("expo", 0.5),
            )
            if curve == "expo" and (
                not isinstance(expo_strength, (int, float)) or expo_strength < 0
            ):
                errors.append(
                    f"{source}.mapping.{axis}.expo_strength must be >= 0 "
                    "for expo curves"
                )
            if dz is not None and dz >= 1:
                errors.append(f"{source}.mapping.{axis}.deadzone must be < 1")

    yaw_cfg = mapping.get("yaw_bias", {})
    if yaw_cfg and not isinstance(yaw_cfg, Mapping):
        errors.append(f"{source}.mapping.yaw_bias must be a mapping")
    else:
        if yaw_cfg:
            jitter = yaw_cfg.get("jitter", 0.0)
            if not isinstance(jitter, (int, float)) or jitter < 0:
                errors.append(f"{source}.mapping.yaw_bias.jitter must be >= 0")
            bias = yaw_cfg.get("bias", 0.0)
            if not isinstance(bias, (int, float)) or not -1.0 <= bias <= 1.0:
                errors.append(f"{source}.mapping.yaw_bias.bias must be within [-1, 1]")

    glitch_cfg = mapping.get("glitch_intensity", {})
    if glitch_cfg:
        if not isinstance(glitch_cfg, Mapping):
            errors.append(f"{source}.mapping.glitch_intensity must be a mapping")
        else:
            base = _require_number(
                glitch_cfg,
                "base",
                f"{source}.mapping.glitch_intensity",
                minimum=0,
                maximum=1,
                errors=errors,
            )
            max_val = _require_number(
                glitch_cfg,
                "max",
                f"{source}.mapping.glitch_intensity",
                minimum=0,
                maximum=1,
                errors=errors,
            )
            if base is not None and max_val is not None and base > max_val:
                errors.append(
                    f"{source}.mapping.glitch_intensity.base cannot exceed max"
                )

    leds_cfg = mapping.get("leds", {})
    if leds_cfg:
        if not isinstance(leds_cfg, Mapping):
            errors.append(f"{source}.mapping.leds must be a mapping")
        else:
            _check_palette(leds_cfg, f"{source}.mapping.leds", errors)

    bridge_cfg = cfg.get("bridge", {})
    if bridge_cfg and not isinstance(bridge_cfg, Mapping):
        errors.append(f"{source}.bridge must be a mapping when provided")
    else:
        hz = bridge_cfg.get("hz", 50)
        if not isinstance(hz, (int, float)) or hz <= 0:
            errors.append(f"{source}.bridge.hz must be > 0")
        mode = bridge_cfg.get("mode", "smooth")
        if not isinstance(mode, str):
            errors.append(f"{source}.bridge.mode must be a string")
        validate_bridge_modes(bridge_cfg, source, errors)
        modes_mapping = bridge_cfg.get("modes")
        if isinstance(modes_mapping, Mapping) and isinstance(mode, str):
            if mode not in bridge_cfg.get("modes", {}):
                errors.append(
                    f"{source}: bridge.mode '{mode}' not found in bridge.modes"
                )

    consent_cfg = cfg.get("consent", {})
    if consent_cfg:
        if not isinstance(consent_cfg, Mapping):
            errors.append(f"{source}.consent must be a mapping")
        else:
            mode = consent_cfg.get("mode", "binary")
            if mode not in {"binary", "smooth"}:
                errors.append(f"{source}.consent.mode must be 'binary' or 'smooth'")
            if "idle_altitude" in consent_cfg:
                _require_number(
                    consent_cfg,
                    "idle_altitude",
                    f"{source}.consent",
                    minimum=-1,
                    maximum=1,
                    errors=errors,
                )

    if errors:
        raise ValidationError(errors)


def load_yaml(path: Path) -> MutableMapping:
    return yaml.safe_load(Path(path).read_text()) or {}


def validate_file(path: Path, *, source_label: str | None = None) -> MutableMapping:
    cfg = load_yaml(path)
    validate_mapping_config(cfg, source_label or path.name)
    return cfg


def validate_recipe(path: Path, loader) -> MutableMapping:
    cfg, _meta = loader(path)
    validate_mapping_config(cfg, path.name)
    return cfg
