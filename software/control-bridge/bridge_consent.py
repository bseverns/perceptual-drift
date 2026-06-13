"""Consent gating, neutralization, and ghost-buffer helpers."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from typing import MutableMapping

from bridge_msp import map_float_to_rc

GESTURE_STATE_KEYS = ("alt", "lat", "yaw", "crowd")
NEUTRAL_AUX_CHANNELS = (1500, 1500, 1500, 1500)


def to_binary_state(value, default=0) -> int:
    """Normalize mixed bool/int/float input into a strict 0/1 state."""

    if isinstance(value, bool):
        return int(value)
    if isinstance(value, (int, float)):
        return 1 if float(value) >= 0.5 else 0
    return int(default)


def neutral_rc_channels() -> tuple[int, int, int, int]:
    return (
        map_float_to_rc(0.0),
        map_float_to_rc(0.0),
        map_float_to_rc(-1.0),
        map_float_to_rc(0.0),
    )


def gesture_snapshot(
    state: MutableMapping[str, float], keys=GESTURE_STATE_KEYS
) -> dict[str, float]:
    return {key: state[key] for key in keys}


def neutralize_gesture_state(state: MutableMapping[str, float]) -> None:
    state.update({"alt": 0.0, "lat": 0.0, "yaw": 0.0, "crowd": 0.0})


@dataclass
class ConsentUpdateResult:
    previous: int
    current: int
    changed: bool
    dropped_to_idle: bool
    replay_started: bool
    replay_depth: int


class GhostGestureBuffer:
    """Pre-roll gesture buffer that replays once consent goes live."""

    def __init__(self, window_seconds: float = 2.0):
        self.window_seconds = max(0.0, float(window_seconds))
        self.buffer: deque[tuple[float, dict[str, float]]] = deque()
        self._frozen: list[tuple[float, dict[str, float]]] = []
        self._replay_idx = 0
        self._replay_origin = 0.0
        self._buffer_origin = 0.0
        self.replaying = False

    def _prune(self, now: float) -> None:
        cutoff = now - self.window_seconds
        while self.buffer and self.buffer[0][0] < cutoff:
            self.buffer.popleft()

    def record(self, now: float, snapshot: dict[str, float]) -> None:
        if self.window_seconds <= 0:
            return
        self._prune(now)
        self.buffer.append((now, dict(snapshot)))

    def arm_replay(self, now: float) -> None:
        self._prune(now)
        self._frozen = list(self.buffer)
        self._replay_idx = 0
        self._replay_origin = now
        self.replaying = bool(self._frozen)
        if self.replaying:
            self._buffer_origin = self._frozen[0][0]

    def snapshot_for(self, now: float) -> dict[str, float] | None:
        if not self.replaying:
            return None
        elapsed = now - self._replay_origin
        target_time = self._buffer_origin + elapsed
        snapshot = None
        while (
            self._replay_idx < len(self._frozen)
            and self._frozen[self._replay_idx][0] <= target_time
        ):
            snapshot = dict(self._frozen[self._replay_idx][1])
            self._replay_idx += 1
        if snapshot:
            return snapshot
        if self._replay_idx >= len(self._frozen):
            self.reset()
        return None

    def reset(self) -> None:
        self.buffer.clear()
        self._frozen = []
        self._replay_idx = 0
        self._replay_origin = 0.0
        self._buffer_origin = 0.0
        self.replaying = False

    def depth(self) -> int:
        return len(self.buffer)


def apply_consent_update(
    state: MutableMapping[str, float],
    raw_value,
    *,
    ghost_buffer: GhostGestureBuffer | None = None,
) -> ConsentUpdateResult:
    previous = int(state.get("consent", 0))
    current = to_binary_state(raw_value, default=previous)
    state["consent"] = current

    if ghost_buffer and current != 1 and previous == 1:
        ghost_buffer.reset()

    replay_started = False
    replay_depth = 0
    if ghost_buffer and current == 1 and previous != 1:
        ghost_buffer.arm_replay(time.monotonic())
        replay_started = ghost_buffer.replaying
        replay_depth = ghost_buffer.depth()

    dropped_to_idle = current != 1 and previous == 1
    if dropped_to_idle:
        neutralize_gesture_state(state)

    return ConsentUpdateResult(
        previous=previous,
        current=current,
        changed=current != previous,
        dropped_to_idle=dropped_to_idle,
        replay_started=replay_started,
        replay_depth=replay_depth,
    )
