"""Output backends downstream of the safety envelope."""

from __future__ import annotations

import json
import struct
import time
from abc import ABC, abstractmethod
from dataclasses import asdict
from typing import Callable, Optional

from .safety import SafetyState


class Backend(ABC):
    @abstractmethod
    def send(self, state: SafetyState) -> None:
        """Send one already-enveloped state."""

    def close(self) -> None:
        """Release backend resources."""


class DryRunBackend(Backend):
    """Inspectable backend that never opens a hardware device."""

    def __init__(self, output: Callable[[str], None] = print) -> None:
        self.output = output
        self.last_state = SafetyState.neutral("backend_not_started")
        self.frames = 0

    def send(self, state: SafetyState) -> None:
        self.last_state = state
        self.frames += 1
        self.output(json.dumps(asdict(state), sort_keys=True))


def _trainer_payload(sequence: int, state: SafetyState) -> str:
    """Return the inspectable host-to-bridge payload without its checksum."""

    return "PD1,{},{:.6f},{:.6f},{:.6f},0.000000".format(
        sequence,
        state.roll,
        state.pitch,
        state.yaw,
    )


def encode_trainer_packet(sequence: int, state: SafetyState) -> bytes:
    payload = _trainer_payload(sequence, state)
    checksum = 0
    for value in payload.encode("ascii"):
        checksum ^= value
    return f"{payload},{checksum:02X}\n".encode("ascii")


class TrainerBackend(Backend):
    """USB-serial backend for the wired trainer bridge.

    The bridge receives only additive normalized axes. Throttle is encoded as
    a constant zero even if an invalid ``SafetyState`` is supplied.
    """

    def __init__(
        self, serial_port, *, clock: Callable[[], float] = time.monotonic
    ):
        self.serial_port = serial_port
        self.clock = clock
        self.sequence = 0
        self.last_heartbeat_timestamp: Optional[float] = None

    def send(self, state: SafetyState) -> None:
        self.serial_port.write(encode_trainer_packet(self.sequence, state))
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF

    def observe_line(self, line: bytes) -> bool:
        """Record an independently generated bridge heartbeat."""

        try:
            text = line.decode("ascii").strip()
        except (UnicodeDecodeError, AttributeError):
            return False
        parts = text.split(",")
        if len(parts) != 2 or parts[0] != "HB":
            return False
        try:
            int(parts[1])
        except ValueError:
            return False
        self.last_heartbeat_timestamp = self.clock()
        return True

    def close(self) -> None:
        self.serial_port.close()


class LegacyMSPBackend(Backend):
    """Legacy/experimental Betaflight MSP output behind the new boundary."""

    MSP_SET_RAW_RC = 200

    def __init__(self, serial_port, *, span_microseconds: int = 400) -> None:
        self.serial_port = serial_port
        self.span_microseconds = span_microseconds

    @staticmethod
    def _packet(command: int, payload: bytes) -> bytes:
        size = len(payload)
        checksum = size ^ command
        for value in payload:
            checksum ^= value
        return b"$M<" + bytes([size, command]) + payload + bytes([checksum])

    def send(self, state: SafetyState) -> None:
        def channel(value: float) -> int:
            return max(
                1000,
                min(2000, int(round(1500 + value * self.span_microseconds))),
            )

        # This legacy adapter retains Betaflight's low-throttle convention. It
        # cannot ARM and never maps the intent throttle contribution.
        channels = (
            channel(state.roll),
            channel(state.pitch),
            1000,
            channel(state.yaw),
            1500,
            1500,
            1500,
            1500,
        )
        payload = struct.pack("<8H", *channels)
        self.serial_port.write(self._packet(self.MSP_SET_RAW_RC, payload))

    def close(self) -> None:
        self.serial_port.close()
