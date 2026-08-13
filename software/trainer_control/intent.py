"""Normalized artistic intent, before physical safety policy is applied."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real
from typing import Mapping


class IntentError(ValueError):
    """Raised when untrusted input cannot become a normalized intent."""


def _finite_number(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise IntentError(f"{name} must be a finite number")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise IntentError(f"{name} must be a finite number")
    if not -1.0 <= parsed <= 1.0:
        raise IntentError(f"{name} must be normalized to [-1, 1]")
    return parsed


@dataclass(frozen=True)
class ControlIntent:
    """Protocol-independent audience contribution.

    There is deliberately no arm or flight-mode field. ``throttle`` exists so
    the safety boundary can explicitly reject it for profiles that forbid it.
    """

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0
    crowd: float = 0.0
    consent: bool = False
    timestamp: float = 0.0

    def validated(self) -> "ControlIntent":
        axes = {
            name: _finite_number(name, getattr(self, name))
            for name in ("roll", "pitch", "yaw", "throttle")
        }
        crowd = _finite_number("crowd", self.crowd)
        if crowd < 0:
            raise IntentError("crowd must be normalized to [0, 1]")
        timestamp = self.timestamp
        if isinstance(timestamp, bool) or not isinstance(timestamp, Real):
            raise IntentError("timestamp must be a finite number")
        timestamp = float(timestamp)
        if not math.isfinite(timestamp) or timestamp <= 0:
            raise IntentError("timestamp must be a positive finite number")
        if not isinstance(self.consent, bool):
            raise IntentError("consent must be true or false")
        return ControlIntent(
            **axes,
            crowd=crowd,
            consent=self.consent,
            timestamp=timestamp,
        )

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "ControlIntent":
        if not isinstance(value, Mapping):
            raise IntentError("intent must be a mapping")
        allowed = {
            "roll",
            "pitch",
            "yaw",
            "throttle",
            "crowd",
            "consent",
            "timestamp",
        }
        unexpected = sorted(set(value) - allowed)
        if unexpected:
            raise IntentError(
                f"unsupported intent fields: {', '.join(unexpected)}"
            )
        try:
            intent = cls(**value)
        except TypeError as exc:
            raise IntentError(f"malformed intent: {exc}") from exc
        return intent.validated()
