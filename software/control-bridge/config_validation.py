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
VALID_MIDI_TARGETS = {"altitude", "lateral", "yaw", "crowd", "consent"}
VALID_MIDI_RESPONSES = {"bipolar", "unipolar"}
VALID_MIDI_NOTE_MODES = {"gate", "toggle"}
_MISSING = object()


# ---- validation primitives -------------------------------------------------


def _render_value(value) -> str:
    if value is _MISSING:
        return "missing"
    text = repr(value)
    if len(text) > 120:
        return text[:117] + "..."
    return text


def _render_allowed(allowed) -> str:
    if isinstance(allowed, (set, list, tuple)):
        return ", ".join(repr(item) for item in allowed)
    return str(allowed)


def _issue(
    source: str,
    path: str,
    problem: str,
    *,
    received=_MISSING,
    allowed=None,
    example: str | None = None,
) -> str:
    parts = [
        f"File {source}",
        f"Key {path}",
        f"Problem: {problem}",
        f"Received: {_render_value(received)}",
    ]
    if allowed is not None:
        parts.append(f"Allowed: {_render_allowed(allowed)}")
    if example is not None:
        parts.append(f"Example fix: {example}")
    return " | ".join(parts)


def _add_error(
    errors: list[str],
    source: str,
    path: str,
    problem: str,
    *,
    received=_MISSING,
    allowed=None,
    example: str | None = None,
) -> None:
    errors.append(
        _issue(
            source,
            path,
            problem,
            received=received,
            allowed=allowed,
            example=example,
        )
    )


def _number_rule_text(minimum=None, maximum=None) -> str:
    if minimum is not None and maximum is not None:
        return f"a number between {minimum} and {maximum}"
    if minimum is not None:
        return f"a number >= {minimum}"
    if maximum is not None:
        return f"a number <= {maximum}"
    return "a number"


def _require_mapping(
    section: Mapping, key: str, path: str, source: str, errors: list[str]
) -> Mapping:
    full_path = f"{path}.{key}"
    if key not in section:
        _add_error(
            errors,
            source,
            full_path,
            "required section is missing",
            allowed="a mapping section",
            example=f"{full_path}: {{}}",
        )
        return {}
    if not isinstance(section[key], Mapping):
        _add_error(
            errors,
            source,
            full_path,
            "section must be a mapping",
            received=section[key],
            allowed="a YAML mapping / object",
            example=f"{full_path}: {{ key: value }}",
        )
        return {}
    return section[key]


def _require_number(
    section: Mapping,
    key: str,
    path: str,
    source: str,
    *,
    minimum=None,
    maximum=None,
    errors: list[str],
):
    full_path = f"{path}.{key}"
    if key not in section:
        _add_error(
            errors,
            source,
            full_path,
            "required value is missing",
            allowed=_number_rule_text(minimum, maximum),
            example=f"{full_path}: 0.5",
        )
        return None
    value = section[key]
    if not isinstance(value, (int, float)):
        _add_error(
            errors,
            source,
            full_path,
            "value must be numeric",
            received=value,
            allowed=_number_rule_text(minimum, maximum),
            example=f"{full_path}: 0.5",
        )
        return None
    if minimum is not None and value < minimum:
        _add_error(
            errors,
            source,
            full_path,
            "value is below the allowed minimum",
            received=value,
            allowed=_number_rule_text(minimum, maximum),
            example=f"{full_path}: {minimum}",
        )
    if maximum is not None and value > maximum:
        _add_error(
            errors,
            source,
            full_path,
            "value is above the allowed maximum",
            received=value,
            allowed=_number_rule_text(minimum, maximum),
            example=f"{full_path}: {maximum}",
        )
    return value


def _check_palette(
    section: Mapping, path: str, source: str, errors: list[str]
) -> None:
    palette = section.get("palette")
    if palette is None:
        return
    if not isinstance(palette, list):
        _add_error(
            errors,
            source,
            f"{path}.palette",
            "palette must be a list of color strings",
            received=palette,
            allowed='a YAML list like ["#1e90ff", "#ff1493"]',
            example=f'{path}.palette: ["#1e90ff", "#ff1493"]',
        )
        return
    if not palette:
        _add_error(
            errors,
            source,
            f"{path}.palette",
            "palette needs at least one color entry",
            received=palette,
            allowed='one or more color strings like ["#1e90ff"]',
            example=f'{path}.palette: ["#1e90ff"]',
        )


def _check_midi_input(
    section: Mapping, path: str, source: str, errors: list[str]
) -> None:
    if not isinstance(section, Mapping):
        _add_error(
            errors,
            source,
            path,
            "MIDI input section must be a mapping",
            received=section,
            allowed="a YAML mapping / object",
            example=f"{path}: {{ channel: 1, port_name: Launchpad }}",
        )
        return
    channel = section.get("channel")
    if channel is not None:
        if not isinstance(channel, int) or not 1 <= channel <= 16:
            _add_error(
                errors,
                source,
                f"{path}.channel",
                "channel must be an integer MIDI channel number",
                received=channel,
                allowed="an integer from 1 to 16",
                example=f"{path}.channel: 1",
            )
    port_name = section.get("port_name")
    if port_name is not None and not isinstance(port_name, str):
        _add_error(
            errors,
            source,
            f"{path}.port_name",
            "port_name must be text when provided",
            received=port_name,
            allowed="a string such as 'Launchpad Mini MK3'",
            example=f"{path}.port_name: Launchpad Mini MK3",
        )


def validate_bridge_modes(
    bridge_cfg: Mapping, source: str, errors: list[str]
) -> None:
    modes = bridge_cfg.get("modes")
    if not isinstance(modes, Mapping) or not modes:
        _add_error(
            errors,
            source,
            "bridge.modes",
            "bridge.modes must exist and include at least one mode",
            received=modes,
            allowed="a non-empty mapping of mode names to settings",
            example="bridge.modes: { smooth: { aux_strategy: full, neutral_rc: false, jitter_scale: 1.0 } }",
        )
        return
    for mode_name, mode_cfg in modes.items():
        mode_path = f"bridge.modes.{mode_name}"
        if not isinstance(mode_cfg, Mapping):
            _add_error(
                errors,
                source,
                mode_path,
                "mode settings must be a mapping",
                received=mode_cfg,
                allowed="a YAML mapping / object",
                example=f"{mode_path}: {{ aux_strategy: full, neutral_rc: false, jitter_scale: 1.0 }}",
            )
            continue
        aux_strategy = mode_cfg.get("aux_strategy", "full")
        if aux_strategy not in VALID_AUX_STRATEGIES:
            allowed_aux = sorted(VALID_AUX_STRATEGIES)
            _add_error(
                errors,
                source,
                f"{mode_path}.aux_strategy",
                "aux strategy is not recognized",
                received=aux_strategy,
                allowed=allowed_aux,
                example=f"{mode_path}.aux_strategy: full",
            )
        neutral = mode_cfg.get("neutral_rc", False)
        if not isinstance(neutral, bool):
            _add_error(
                errors,
                source,
                f"{mode_path}.neutral_rc",
                "neutral_rc must be true or false",
                received=neutral,
                allowed="true or false",
                example=f"{mode_path}.neutral_rc: false",
            )
        jitter_scale = mode_cfg.get("jitter_scale", 1.0)
        if not isinstance(jitter_scale, (int, float)) or jitter_scale < 0:
            _add_error(
                errors,
                source,
                f"{mode_path}.jitter_scale",
                "jitter_scale must be zero or a positive number",
                received=jitter_scale,
                allowed="a number >= 0",
                example=f"{mode_path}.jitter_scale: 1.0",
            )
        dz_boost = mode_cfg.get("deadzone_boost", {})
        if dz_boost:
            if not isinstance(dz_boost, Mapping):
                _add_error(
                    errors,
                    source,
                    f"{mode_path}.deadzone_boost",
                    "deadzone_boost must map axis names to values",
                    received=dz_boost,
                    allowed="a mapping like { altitude: 0.08, lateral: 0.08 }",
                    example=f"{mode_path}.deadzone_boost: {{ altitude: 0.08, lateral: 0.08 }}",
                )
            else:
                for axis, val in dz_boost.items():
                    if not isinstance(val, (int, float)) or not (0 <= val < 1):
                        _add_error(
                            errors,
                            source,
                            f"{mode_path}.deadzone_boost.{axis}",
                            "deadzone boost must stay within the safe range",
                            received=val,
                            allowed="a number from 0 up to, but not including, 1",
                            example=f"{mode_path}.deadzone_boost.{axis}: 0.08",
                        )
        gain_scale = mode_cfg.get("gain_scale", {})
        if gain_scale:
            if not isinstance(gain_scale, Mapping):
                _add_error(
                    errors,
                    source,
                    f"{mode_path}.gain_scale",
                    "gain_scale must map axis names to values",
                    received=gain_scale,
                    allowed="a mapping like { altitude: 0.6, lateral: 0.6 }",
                    example=f"{mode_path}.gain_scale: {{ altitude: 0.6, lateral: 0.6 }}",
                )
            else:
                for axis, val in gain_scale.items():
                    if not isinstance(val, (int, float)):
                        _add_error(
                            errors,
                            source,
                            f"{mode_path}.gain_scale.{axis}",
                            "gain scale must be numeric",
                            received=val,
                            allowed="a number >= 0",
                            example=f"{mode_path}.gain_scale.{axis}: 0.6",
                        )
                    elif val < 0:
                        _add_error(
                            errors,
                            source,
                            f"{mode_path}.gain_scale.{axis}",
                            "gain scale cannot be negative",
                            received=val,
                            allowed="a number >= 0",
                            example=f"{mode_path}.gain_scale.{axis}: 0.6",
                        )


def validate_midi_mapping_config(
    cfg: Mapping, source: str = "midi_mapping"
) -> None:
    errors: list[str] = []
    if not isinstance(cfg, Mapping):
        raise ValidationError(
            [
                _issue(
                    source,
                    "<root>",
                    "config must be a mapping",
                    received=cfg,
                    allowed="a YAML mapping / object",
                    example="input: { channel: 1 }\ngestures: [...]",
                )
            ]
        )

    midi_input = cfg.get("input", {})
    _check_midi_input(midi_input, "input", source, errors)

    gestures = cfg.get("gestures")
    if gestures is None:
        _add_error(
            errors,
            source,
            "gestures",
            "gestures list is required",
            allowed="a non-empty list of CC/note gesture mappings",
            example="gestures:\n  - type: cc\n    cc_number: 10\n    target: lateral",
        )
    elif not isinstance(gestures, list) or not gestures:
        _add_error(
            errors,
            source,
            "gestures",
            "gestures must be a non-empty list",
            received=gestures,
            allowed="a non-empty list of CC/note gesture mappings",
            example="gestures:\n  - type: cc\n    cc_number: 10\n    target: lateral",
        )
    else:
        for idx, gesture in enumerate(gestures):
            path = f"gestures[{idx}]"
            if not isinstance(gesture, Mapping):
                _add_error(
                    errors,
                    source,
                    path,
                    "gesture entry must be a mapping",
                    received=gesture,
                    allowed="a YAML mapping / object",
                    example=f"{path}: {{ type: cc, cc_number: 10, target: lateral }}",
                )
                continue
            target = gesture.get("target")
            if target not in VALID_MIDI_TARGETS:
                allowed_targets = sorted(VALID_MIDI_TARGETS)
                _add_error(
                    errors,
                    source,
                    f"{path}.target",
                    "target is not recognized",
                    received=target,
                    allowed=allowed_targets,
                    example=f"{path}.target: lateral",
                )
            gesture_type = gesture.get("type")
            if gesture_type not in {"cc", "note"}:
                _add_error(
                    errors,
                    source,
                    f"{path}.type",
                    "gesture type is not recognized",
                    received=gesture_type,
                    allowed=["cc", "note"],
                    example=f"{path}.type: cc",
                )
                continue
            if gesture_type == "cc":
                cc_number = gesture.get("cc_number")
                if not isinstance(cc_number, int) or not 0 <= cc_number <= 127:
                    _add_error(
                        errors,
                        source,
                        f"{path}.cc_number",
                        "CC number must be a valid MIDI controller number",
                        received=cc_number,
                        allowed="an integer from 0 to 127",
                        example=f"{path}.cc_number: 10",
                    )
                response = gesture.get("response", "bipolar")
                if response not in VALID_MIDI_RESPONSES:
                    allowed = sorted(VALID_MIDI_RESPONSES)
                    _add_error(
                        errors,
                        source,
                        f"{path}.response",
                        "response mode is not recognized",
                        received=response,
                        allowed=allowed,
                        example=f"{path}.response: bipolar",
                    )
            if gesture_type == "note":
                note_number = gesture.get("note_number")
                valid_note = (
                    isinstance(note_number, int) and 0 <= note_number <= 127
                )
                if not valid_note:
                    _add_error(
                        errors,
                        source,
                        f"{path}.note_number",
                        "note number must be a valid MIDI note",
                        received=note_number,
                        allowed="an integer from 0 to 127",
                        example=f"{path}.note_number: 60",
                    )
                mode = gesture.get("mode", "gate")
                if mode not in VALID_MIDI_NOTE_MODES:
                    allowed = sorted(VALID_MIDI_NOTE_MODES)
                    _add_error(
                        errors,
                        source,
                        f"{path}.mode",
                        "note mode is not recognized",
                        received=mode,
                        allowed=allowed,
                        example=f"{path}.mode: gate",
                    )

    if errors:
        raise ValidationError(errors)


def validate_mapping_config(cfg: Mapping, source: str = "mapping") -> None:
    errors: list[str] = []
    if not isinstance(cfg, Mapping):
        raise ValidationError(
            [
                _issue(
                    source,
                    "<root>",
                    "config must be a mapping",
                    received=cfg,
                    allowed="a YAML mapping / object",
                    example=(
                        "osc:\n  port: 9000\nmapping:\n"
                        "  altitude: { deadzone: 0.05, curve: linear, gain: 0.6 }"
                    ),
                )
            ]
        )

    osc = _require_mapping(cfg, "osc", "<root>", source, errors)
    address_space = _require_mapping(
        osc,
        "address_space",
        "osc",
        source,
        errors,
    )
    for path_name in REQUIRED_OSC_PATHS:
        if path_name not in address_space:
            _add_error(
                errors,
                source,
                f"osc.address_space.{path_name}",
                "OSC route is missing",
                allowed="a string route such as '/pd/consent'",
                example=(
                    f"osc.address_space.{path_name}: "
                    f'"/pd/{path_name if path_name != "altitude" else "alt"}"'
                ),
            )
        elif not isinstance(address_space[path_name], str):
            _add_error(
                errors,
                source,
                f"osc.address_space.{path_name}",
                "OSC route must be text",
                received=address_space[path_name],
                allowed="a string route such as '/pd/consent'",
                example=(
                    f"osc.address_space.{path_name}: "
                    f'"/pd/{path_name if path_name != "altitude" else "alt"}"'
                ),
            )

    mapping = _require_mapping(cfg, "mapping", "<root>", source, errors)
    for axis in REQUIRED_AXES:
        axis_cfg = _require_mapping(
            mapping,
            axis,
            "mapping",
            source,
            errors,
        )
        if axis_cfg:
            dz = _require_number(
                axis_cfg,
                "deadzone",
                f"mapping.{axis}",
                source,
                minimum=0,
                maximum=0.99,
                errors=errors,
            )
            curve = axis_cfg.get("curve", "linear")
            if curve not in VALID_CURVES:
                allowed_curves = sorted(VALID_CURVES)
                _add_error(
                    errors,
                    source,
                    f"mapping.{axis}.curve",
                    "curve name is not recognized",
                    received=curve,
                    allowed=allowed_curves,
                    example=f'mapping.{axis}.curve: "linear"',
                )
            gain = axis_cfg.get("gain", 1.0)
            if not isinstance(gain, (int, float)) or gain < 0:
                _add_error(
                    errors,
                    source,
                    f"mapping.{axis}.gain",
                    "gain must be zero or a positive number",
                    received=gain,
                    allowed="a number >= 0",
                    example=f"mapping.{axis}.gain: 0.6",
                )
            expo_strength = axis_cfg.get(
                "expo_strength",
                axis_cfg.get("expo", 0.5),
            )
            if curve == "expo" and (
                not isinstance(expo_strength, (int, float))
                or expo_strength < 0
            ):
                _add_error(
                    errors,
                    source,
                    f"mapping.{axis}.expo_strength",
                    "expo_strength must be zero or a positive number when curve is expo",
                    received=expo_strength,
                    allowed="a number >= 0",
                    example=f"mapping.{axis}.expo_strength: 0.5",
                )
            if dz is not None and dz >= 1:
                _add_error(
                    errors,
                    source,
                    f"mapping.{axis}.deadzone",
                    "deadzone must stay below 1 so the stick can still move",
                    received=dz,
                    allowed="a number from 0 up to, but not including, 1",
                    example=f"mapping.{axis}.deadzone: 0.05",
                )

    yaw_cfg = mapping.get("yaw_bias", {})
    if yaw_cfg and not isinstance(yaw_cfg, Mapping):
        _add_error(
            errors,
            source,
            "mapping.yaw_bias",
            "yaw_bias must be a mapping",
            received=yaw_cfg,
            allowed="a mapping like { bias: 0.1, jitter: 0.05 }",
            example="mapping.yaw_bias: { bias: 0.1, jitter: 0.05 }",
        )
    else:
        if yaw_cfg:
            jitter = yaw_cfg.get("jitter", 0.0)
            if not isinstance(jitter, (int, float)) or jitter < 0:
                _add_error(
                    errors,
                    source,
                    "mapping.yaw_bias.jitter",
                    "jitter must be zero or a positive number",
                    received=jitter,
                    allowed="a number >= 0",
                    example="mapping.yaw_bias.jitter: 0.05",
                )
            bias = yaw_cfg.get("bias", 0.0)
            if not isinstance(bias, (int, float)) or not -1.0 <= bias <= 1.0:
                _add_error(
                    errors,
                    source,
                    "mapping.yaw_bias.bias",
                    "bias is outside the supported range",
                    received=bias,
                    allowed="a number from -1 to 1",
                    example="mapping.yaw_bias.bias: 0.1",
                )

    glitch_cfg = mapping.get("glitch_intensity", {})
    if glitch_cfg:
        if not isinstance(glitch_cfg, Mapping):
            _add_error(
                errors,
                source,
                "mapping.glitch_intensity",
                "glitch_intensity must be a mapping",
                received=glitch_cfg,
                allowed="a mapping like { base: 0.2, max: 0.9 }",
                example="mapping.glitch_intensity: { base: 0.2, max: 0.9 }",
            )
        else:
            base = _require_number(
                glitch_cfg,
                "base",
                "mapping.glitch_intensity",
                source,
                minimum=0,
                maximum=1,
                errors=errors,
            )
            max_val = _require_number(
                glitch_cfg,
                "max",
                "mapping.glitch_intensity",
                source,
                minimum=0,
                maximum=1,
                errors=errors,
            )
            if base is not None and max_val is not None and base > max_val:
                _add_error(
                    errors,
                    source,
                    "mapping.glitch_intensity.base",
                    "base cannot be greater than max",
                    received=base,
                    allowed=f"a number <= mapping.glitch_intensity.max ({max_val})",
                    example="mapping.glitch_intensity.base: 0.2",
                )

    leds_cfg = mapping.get("leds", {})
    if leds_cfg:
        if not isinstance(leds_cfg, Mapping):
            _add_error(
                errors,
                source,
                "mapping.leds",
                "leds must be a mapping",
                received=leds_cfg,
                allowed='a mapping like { palette: ["#1e90ff"] }',
                example='mapping.leds: { palette: ["#1e90ff"] }',
            )
        else:
            _check_palette(leds_cfg, "mapping.leds", source, errors)

    bridge_cfg = cfg.get("bridge", {})
    if bridge_cfg and not isinstance(bridge_cfg, Mapping):
        _add_error(
            errors,
            source,
            "bridge",
            "bridge must be a mapping when provided",
            received=bridge_cfg,
            allowed="a YAML mapping / object",
            example=(
                "bridge: { hz: 50, mode: smooth, modes: { smooth: "
                "{ aux_strategy: full, neutral_rc: false, jitter_scale: 1.0 } } }"
            ),
        )
    else:
        hz = bridge_cfg.get("hz", 50)
        if not isinstance(hz, (int, float)) or hz <= 0:
            _add_error(
                errors,
                source,
                "bridge.hz",
                "bridge send rate must be greater than zero",
                received=hz,
                allowed="a number > 0",
                example="bridge.hz: 50",
            )
        mode = bridge_cfg.get("mode", "smooth")
        if not isinstance(mode, str):
            _add_error(
                errors,
                source,
                "bridge.mode",
                "bridge mode name must be text",
                received=mode,
                allowed="a string that matches a key under bridge.modes",
                example="bridge.mode: smooth",
            )
        ghost_mode = bridge_cfg.get("ghost_mode")
        if ghost_mode is not None and not isinstance(ghost_mode, bool):
            _add_error(
                errors,
                source,
                "bridge.ghost_mode",
                "ghost_mode must be true or false when provided",
                received=ghost_mode,
                allowed="true or false",
                example="bridge.ghost_mode: true",
            )
        ghost_buffer = bridge_cfg.get("ghost_buffer_seconds")
        if ghost_buffer is not None:
            if not isinstance(ghost_buffer, (int, float)):
                _add_error(
                    errors,
                    source,
                    "bridge.ghost_buffer_seconds",
                    "ghost buffer length must be numeric",
                    received=ghost_buffer,
                    allowed="a number >= 0",
                    example="bridge.ghost_buffer_seconds: 2.0",
                )
            elif ghost_buffer < 0:
                _add_error(
                    errors,
                    source,
                    "bridge.ghost_buffer_seconds",
                    "ghost buffer length cannot be negative",
                    received=ghost_buffer,
                    allowed="a number >= 0",
                    example="bridge.ghost_buffer_seconds: 2.0",
                )
        stale_after = bridge_cfg.get("stale_after")
        if stale_after is not None:
            if not isinstance(stale_after, (int, float)):
                _add_error(
                    errors,
                    source,
                    "bridge.stale_after",
                    "stale_after must be numeric",
                    received=stale_after,
                    allowed="a number >= 0",
                    example="bridge.stale_after: 0.35",
                )
            elif stale_after < 0:
                _add_error(
                    errors,
                    source,
                    "bridge.stale_after",
                    "stale_after cannot be negative",
                    received=stale_after,
                    allowed="a number >= 0",
                    example="bridge.stale_after: 0.35",
                )
        validate_bridge_modes(bridge_cfg, source, errors)
        modes_mapping = bridge_cfg.get("modes")
        if isinstance(modes_mapping, Mapping) and isinstance(mode, str):
            if mode not in bridge_cfg.get("modes", {}):
                _add_error(
                    errors,
                    source,
                    "bridge.mode",
                    "bridge.mode does not match any defined mode",
                    received=mode,
                    allowed=sorted(bridge_cfg.get("modes", {}).keys()),
                    example="bridge.mode: smooth",
                )

    consent_cfg = cfg.get("consent", {})
    if consent_cfg:
        if not isinstance(consent_cfg, Mapping):
            _add_error(
                errors,
                source,
                "consent",
                "consent must be a mapping",
                received=consent_cfg,
                allowed="a mapping like { default_state: 0, mode: binary }",
                example="consent: { default_state: 0, mode: binary }",
            )
        else:
            mode = consent_cfg.get("mode", "binary")
            if mode not in {"binary", "smooth"}:
                _add_error(
                    errors,
                    source,
                    "consent.mode",
                    "consent mode is not recognized",
                    received=mode,
                    allowed=["binary", "smooth"],
                    example='consent.mode: "binary"',
                )
            if "default_state" in consent_cfg:
                default_state = consent_cfg.get("default_state")
                if isinstance(default_state, bool):
                    pass
                elif isinstance(default_state, (int, float)):
                    if default_state < 0 or default_state > 1:
                        _add_error(
                            errors,
                            source,
                            "consent.default_state",
                            "default_state is outside the supported range",
                            received=default_state,
                            allowed="0 or 1 (or any number between 0 and 1)",
                            example="consent.default_state: 0",
                        )
                else:
                    _add_error(
                        errors,
                        source,
                        "consent.default_state",
                        "default_state must be numeric or boolean",
                        received=default_state,
                        allowed="0, 1, true, false, or any number between 0 and 1",
                        example="consent.default_state: 0",
                    )
            for flag_name in ("gate_motion", "auto_recipes"):
                if flag_name in consent_cfg and not isinstance(
                    consent_cfg.get(flag_name), bool
                ):
                    _add_error(
                        errors,
                        source,
                        f"consent.{flag_name}",
                        f"{flag_name} must be true or false",
                        received=consent_cfg.get(flag_name),
                        allowed="true or false",
                        example=f"consent.{flag_name}: false",
                    )
            if "idle_altitude" in consent_cfg:
                _require_number(
                    consent_cfg,
                    "idle_altitude",
                    "consent",
                    source,
                    minimum=-1,
                    maximum=1,
                    errors=errors,
                )

    if errors:
        raise ValidationError(errors)


def load_yaml(path: Path) -> MutableMapping:
    return yaml.safe_load(Path(path).read_text()) or {}


def validate_file(
    path: Path, *, source_label: str | None = None
) -> MutableMapping:
    source = source_label or str(Path(path).resolve())
    try:
        cfg = load_yaml(path)
    except yaml.YAMLError as exc:
        raise ValidationError(
            [
                _issue(
                    source,
                    "<file>",
                    "file could not be parsed as YAML/JSON",
                    received=(
                        str(exc).splitlines()[0] if str(exc) else str(exc)
                    ),
                    allowed="valid YAML/JSON with consistent indentation and ':' separators",
                    example="mapping:\n  altitude:\n    deadzone: 0.05",
                )
            ]
        ) from exc
    validate_mapping_config(cfg, source)
    return cfg


def validate_midi_file(
    path: Path, *, source_label: str | None = None
) -> MutableMapping:
    source = source_label or str(Path(path).resolve())
    try:
        cfg = load_yaml(path)
    except yaml.YAMLError as exc:
        raise ValidationError(
            [
                _issue(
                    source,
                    "<file>",
                    "file could not be parsed as YAML/JSON",
                    received=(
                        str(exc).splitlines()[0] if str(exc) else str(exc)
                    ),
                    allowed="valid YAML/JSON with consistent indentation and ':' separators",
                    example="input:\n  channel: 1\ngestures:\n  - type: cc",
                )
            ]
        ) from exc
    validate_midi_mapping_config(cfg, source)
    return cfg


def validate_recipe(path: Path, loader) -> MutableMapping:
    source = str(Path(path).resolve())
    try:
        cfg, _meta = loader(path)
    except ValidationError:
        raise
    except Exception as exc:
        raise ValidationError(
            [
                _issue(
                    source,
                    "control_bridge",
                    "recipe could not be loaded",
                    received=str(exc),
                    allowed=(
                        "a readable recipe file with a valid control_bridge "
                        "section and working extends path"
                    ),
                    example="control_bridge:\n  extends: ../mapping.yaml",
                )
            ]
        ) from exc
    validate_mapping_config(cfg, source)
    return cfg
