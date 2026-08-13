"""Small orchestration layer for Intent -> SafetyEnvelope -> Backend."""

from __future__ import annotations

from typing import Optional

from .backends import Backend
from .intent import ControlIntent
from .safety import SafetyEnvelope, SafetyState


class TrainerControlPipeline:
    def __init__(self, envelope: SafetyEnvelope, backend: Backend) -> None:
        self.envelope = envelope
        self.backend = backend

    def process(
        self,
        intent: Optional[ControlIntent],
        *,
        now: float,
        operator_enabled: bool,
        bridge_heartbeat_timestamp: Optional[float],
    ) -> SafetyState:
        state = self.envelope.evaluate(
            intent,
            now=now,
            operator_enabled=operator_enabled,
            bridge_heartbeat_timestamp=bridge_heartbeat_timestamp,
        )
        self.backend.send(state)
        return state

    def close(self) -> None:
        self.backend.close()
