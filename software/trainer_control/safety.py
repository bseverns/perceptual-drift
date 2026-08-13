"""Fail-closed safety envelope between artistic intent and physical output."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .intent import ControlIntent, IntentError
from .profiles import (
    AircraftSafetyProfile,
    ProfileError,
    load_aircraft_profile,
)


@dataclass(frozen=True)
class SafetyState:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0
    crowd: float = 0.0
    consent: bool = False
    operator_enabled: bool = False
    input_fresh: bool = False
    heartbeat_fresh: bool = False
    active: bool = False
    neutral_reason: str = "startup_before_valid_input"
    profile_id: str = "invalid"

    @classmethod
    def neutral(
        cls, reason: str, profile_id: str = "invalid"
    ) -> "SafetyState":
        return cls(neutral_reason=reason, profile_id=profile_id)


class SafetyEnvelope:
    """Apply immutable aircraft limits and bypass mapping on any unsafe state."""

    def __init__(
        self,
        profile: Optional[AircraftSafetyProfile],
        profile_error: Optional[str] = None,
    ) -> None:
        self.profile = profile
        self.profile_error = profile_error

    @classmethod
    def from_path(cls, path: Path) -> "SafetyEnvelope":
        try:
            return cls(load_aircraft_profile(path))
        except (ProfileError, OSError) as exc:
            return cls(None, str(exc))

    def evaluate_raw(
        self,
        value: object,
        *,
        now: float,
        operator_enabled: bool,
        bridge_heartbeat_timestamp: Optional[float],
    ) -> SafetyState:
        try:
            intent = ControlIntent.from_mapping(value)  # type: ignore[arg-type]
        except (IntentError, TypeError, ValueError):
            return self._neutral("malformed_input")
        return self.evaluate(
            intent,
            now=now,
            operator_enabled=operator_enabled,
            bridge_heartbeat_timestamp=bridge_heartbeat_timestamp,
        )

    def evaluate(
        self,
        intent: Optional[ControlIntent],
        *,
        now: float,
        operator_enabled: bool,
        bridge_heartbeat_timestamp: Optional[float],
    ) -> SafetyState:
        if self.profile is None:
            return self._neutral("safety_profile_validation_failure")
        if intent is None:
            return self._neutral("startup_before_valid_input")
        try:
            intent = intent.validated()
        except IntentError:
            return self._neutral("malformed_input")
        if isinstance(now, bool) or not isinstance(now, (int, float)):
            return self._neutral("invalid_clock")
        now = float(now)
        if not math.isfinite(now):
            return self._neutral("invalid_clock")
        if not operator_enabled:
            return self._neutral("operator_enable_false")
        if not intent.consent:
            return self._neutral("consent_off")
        if (
            now - intent.timestamp < 0
            or now - intent.timestamp > self.profile.stale_after_seconds
        ):
            return self._neutral("stale_input")
        if bridge_heartbeat_timestamp is None:
            return self._neutral("trainer_bridge_heartbeat_lost")
        if isinstance(bridge_heartbeat_timestamp, bool) or not isinstance(
            bridge_heartbeat_timestamp, (int, float)
        ):
            return self._neutral("trainer_bridge_heartbeat_lost")
        heartbeat_age = now - float(bridge_heartbeat_timestamp)
        if not math.isfinite(heartbeat_age) or heartbeat_age < 0:
            return self._neutral("trainer_bridge_heartbeat_lost")
        if heartbeat_age > self.profile.bridge_heartbeat_timeout_seconds:
            return self._neutral("trainer_bridge_heartbeat_lost")

        limits = self.profile.max_influence
        return SafetyState(
            roll=self._bound(intent.roll, limits["roll"]),
            pitch=self._bound(intent.pitch, limits["pitch"]),
            yaw=self._bound(intent.yaw, limits["yaw"]),
            throttle=(
                self._bound(intent.throttle, limits["throttle"])
                if self.profile.software_throttle_allowed
                else 0.0
            ),
            crowd=intent.crowd,
            consent=True,
            operator_enabled=True,
            input_fresh=True,
            heartbeat_fresh=True,
            active=True,
            neutral_reason="none",
            profile_id=self.profile.profile_id,
        )

    def _neutral(self, reason: str) -> SafetyState:
        profile_id = self.profile.profile_id if self.profile else "invalid"
        return SafetyState.neutral(reason, profile_id)

    @staticmethod
    def _bound(value: float, limit: float) -> float:
        if limit == 0:
            return 0.0
        return max(-limit, min(limit, value))
