"""Loading and validation for aircraft and transmitter safety profiles."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping

import yaml


EVIDENCE_LABELS = {
    "SOFTWARE VERIFIED",
    "CONFIG EXPECTED",
    "OPERATOR VERIFIED",
    "UNVERIFIED",
}

IMMUTABLE_TRAINER_LIMITS = MappingProxyType(
    {"roll": 0.15, "pitch": 0.0, "yaw": 0.15, "throttle": 0.0}
)


class ProfileError(ValueError):
    """Raised when a hardware profile cannot enforce the safety contract."""


@dataclass(frozen=True)
class AircraftSafetyProfile:
    profile_id: str
    display_name: str
    max_influence: Mapping[str, float]
    software_arm_allowed: bool
    software_throttle_allowed: bool
    stale_after_seconds: float
    bridge_heartbeat_timeout_seconds: float
    raw: Mapping[str, object]


def _mapping(value: object, path: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ProfileError(f"{path} must be a mapping")
    return value


def _bool(value: object, path: str) -> bool:
    if not isinstance(value, bool):
        raise ProfileError(f"{path} must be true or false")
    return value


def _positive_number(value: object, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProfileError(f"{path} must be a positive finite number")
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise ProfileError(f"{path} must be a positive finite number")
    return parsed


def _axis_limit(value: object, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProfileError(f"{path} must be a finite number in [0, 1]")
    parsed = float(value)
    if not math.isfinite(parsed) or not 0 <= parsed <= 1:
        raise ProfileError(f"{path} must be a finite number in [0, 1]")
    return parsed


def load_yaml_mapping(path: Path) -> Mapping[str, object]:
    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ProfileError(f"could not load {path}: {exc}") from exc
    return _mapping(loaded, str(path))


def validate_evidence_labels(data: object, path: str = "profile") -> None:
    if isinstance(data, Mapping):
        for key, value in data.items():
            child = f"{path}.{key}"
            if key == "evidence" and value not in EVIDENCE_LABELS:
                labels = ", ".join(sorted(EVIDENCE_LABELS))
                raise ProfileError(f"{child} must be one of: {labels}")
            validate_evidence_labels(value, child)
    elif isinstance(data, list):
        for index, value in enumerate(data):
            validate_evidence_labels(value, f"{path}[{index}]")


def load_aircraft_profile(path: Path) -> AircraftSafetyProfile:
    raw = load_yaml_mapping(path)
    validate_evidence_labels(raw, "aircraft")
    if raw.get("kind") != "aircraft":
        raise ProfileError("aircraft.kind must be 'aircraft'")
    safety = _mapping(raw.get("safety"), "aircraft.safety")
    influence = _mapping(
        safety.get("max_normalized_influence"),
        "aircraft.safety.max_normalized_influence",
    )
    limits = {
        axis: _axis_limit(
            influence.get(axis),
            f"aircraft.safety.max_normalized_influence.{axis}",
        )
        for axis in ("roll", "pitch", "yaw", "throttle")
    }
    arm_allowed = _bool(
        safety.get("software_arm_allowed"),
        "aircraft.safety.software_arm_allowed",
    )
    throttle_allowed = _bool(
        safety.get("software_throttle_allowed"),
        "aircraft.safety.software_throttle_allowed",
    )
    if arm_allowed:
        raise ProfileError("aircraft profile must forbid software ARM")
    if throttle_allowed:
        raise ProfileError("aircraft profile must forbid software throttle")
    if limits["throttle"] != IMMUTABLE_TRAINER_LIMITS["throttle"]:
        raise ProfileError(
            "forbidden software throttle must have a zero limit"
        )
    if limits["pitch"] != IMMUTABLE_TRAINER_LIMITS["pitch"]:
        raise ProfileError("initial aircraft profile must disable pitch")
    for axis in ("roll", "yaw"):
        if limits[axis] > IMMUTABLE_TRAINER_LIMITS[axis]:
            raise ProfileError(
                "initial aircraft {} limit cannot exceed {}".format(
                    axis, IMMUTABLE_TRAINER_LIMITS[axis]
                )
            )
    return AircraftSafetyProfile(
        profile_id=str(raw.get("id", "")),
        display_name=str(raw.get("display_name", "")),
        max_influence=limits,
        software_arm_allowed=arm_allowed,
        software_throttle_allowed=throttle_allowed,
        stale_after_seconds=_positive_number(
            safety.get("stale_after_seconds"),
            "aircraft.safety.stale_after_seconds",
        ),
        bridge_heartbeat_timeout_seconds=_positive_number(
            safety.get("bridge_heartbeat_timeout_seconds"),
            "aircraft.safety.bridge_heartbeat_timeout_seconds",
        ),
        raw=raw,
    )


def validate_transmitter_profile(path: Path) -> Mapping[str, object]:
    raw = load_yaml_mapping(path)
    validate_evidence_labels(raw, "transmitter")
    if raw.get("kind") != "transmitter":
        raise ProfileError("transmitter.kind must be 'transmitter'")
    trainer = _mapping(raw.get("trainer"), "transmitter.trainer")
    axes = {
        axis: _mapping(trainer.get(axis), f"transmitter.trainer.{axis}")
        for axis in ("roll", "pitch", "yaw", "throttle")
    }
    for axis in ("roll", "yaw"):
        cfg = axes[axis]
        if cfg.get("mode") != "ADD":
            raise ProfileError(f"transmitter.trainer.{axis}.mode must be ADD")
        raw_weight = cfg.get("maximum_weight_percent")
        if isinstance(raw_weight, bool) or not isinstance(
            raw_weight, (int, float)
        ):
            raise ProfileError(
                f"transmitter.trainer.{axis}.maximum_weight_percent must be numeric"
            )
        weight = _axis_limit(
            float(raw_weight) / 100,
            f"transmitter.trainer.{axis}.maximum_weight_percent",
        )
        if weight > IMMUTABLE_TRAINER_LIMITS[axis]:
            raise ProfileError(
                f"transmitter.trainer.{axis}.maximum_weight_percent cannot exceed 15"
            )
    for axis in ("pitch", "throttle"):
        cfg = axes[axis]
        if cfg.get("mode") != "OFF" or cfg.get("maximum_weight_percent") != 0:
            raise ProfileError(
                f"transmitter.trainer.{axis} must be OFF with zero weight"
            )
    return raw
