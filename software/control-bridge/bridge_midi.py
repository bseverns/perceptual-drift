"""MIDI backend setup and input listener glue."""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

import mido

from bridge_msp import clamp

if TYPE_CHECKING:
    from bridge_mapping import Mapper

MIDI_BACKEND = "rtmidi"
MIDI_BACKEND_ERROR: Exception | None = None
try:
    import mido.backends.rtmidi  # type: ignore # noqa: F401

    mido.set_backend("mido.backends.rtmidi")
except Exception as exc:  # noqa: BLE001
    MIDI_BACKEND = "dummy"
    MIDI_BACKEND_ERROR = exc
    mido.set_backend("mido.backends.dummy")


def normalize_cc(value: int, response: str = "bipolar") -> float:
    """Map a 0..127 MIDI value into the mapper's normalized ranges."""

    norm = clamp(value / 127.0, 0.0, 1.0)
    if response == "unipolar":
        return norm
    return norm * 2 - 1


class MidiListener:
    def __init__(self, mapper: "Mapper", midi_cfg: dict, audit):
        self.mapper = mapper
        self.audit = audit
        self.midi_cfg = midi_cfg or {}
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._port: mido.ports.BaseInput | None = None
        input_cfg = self.midi_cfg.get("input", {}) or {}
        channel = input_cfg.get("channel")
        self.channel = channel - 1 if isinstance(channel, int) else None
        self.port_name = input_cfg.get("port_name")
        self.cc_routes: dict[int, dict] = {}
        self.note_routes: dict[int, dict] = {}
        self.note_states: dict[int, int] = {}
        for gesture in self.midi_cfg.get("gestures", []):
            if gesture.get("type") == "cc":
                self.cc_routes[int(gesture["cc_number"])] = gesture
            elif gesture.get("type") == "note":
                self.note_routes[int(gesture["note_number"])] = gesture

    def start(self) -> None:
        if not (self.cc_routes or self.note_routes):
            return
        if MIDI_BACKEND != "rtmidi":
            reason = (
                "python-rtmidi is missing"
                if MIDI_BACKEND_ERROR
                else "backend not available"
            )
            msg = (
                "MIDI listener disabled: {reason}. OSC still flows; "
                "install the optional MIDI deps when you want faders "
                "in the loop."
            ).format(reason=reason)
            print(msg)
            self.audit.write(
                "midi_bridge_boot",
                status="warning",
                message=msg,
                details={
                    "backend": MIDI_BACKEND,
                    "error": (
                        str(MIDI_BACKEND_ERROR) if MIDI_BACKEND_ERROR else None
                    ),
                },
            )
            return
        self._port = (
            mido.open_input(self.port_name)
            if self.port_name
            else mido.open_input()
        )
        msg = "MIDI listener bound to {port} (channel {channel})".format(
            port=self._port.name,
            channel=(self.channel + 1) if self.channel is not None else "any",
        )
        print(msg)
        self.audit.write(
            "midi_bridge_boot",
            status="info",
            message=msg,
            details={
                "port": self._port.name,
                "channel": self.channel,
                "cc_gestures": sorted(self.cc_routes.keys()),
                "note_gestures": sorted(self.note_routes.keys()),
            },
        )
        self._thread = threading.Thread(
            target=self._loop, name="midi_listener", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._port:
            self._port.close()

    def _loop(self) -> None:
        while not self._stop_event.is_set():
            if self._port is None:
                break
            for msg in self._port.iter_pending():
                self._handle_message(msg)
            time.sleep(0.01)

    def _handle_message(self, msg: mido.Message) -> None:
        if self.channel is not None and hasattr(msg, "channel"):
            if msg.channel != self.channel:
                return
        if msg.type == "control_change":
            gesture = self.cc_routes.get(msg.control)
            if not gesture:
                return
            response = gesture.get("response", "bipolar")
            value = normalize_cc(msg.value, response)
            self._update_state(gesture.get("target"), value)
        elif msg.type in {"note_on", "note_off"}:
            gesture = self.note_routes.get(msg.note)
            if not gesture:
                return
            mode = gesture.get("mode", "gate")
            velocity = getattr(msg, "velocity", 0)
            if mode == "toggle" and msg.type == "note_on" and velocity > 0:
                prev = self.note_states.get(msg.note, 0)
                new_val = 0 if prev else 1
                self.note_states[msg.note] = new_val
                self._update_state(gesture.get("target"), new_val)
            else:
                active = 1 if msg.type == "note_on" and velocity > 0 else 0
                self._update_state(gesture.get("target"), active)

    def _update_state(self, target: str | None, value: float) -> None:
        if target is None:
            return
        if target in {"altitude", "lateral", "yaw"}:
            if target == "altitude":
                key = "alt"
            elif target == "lateral":
                key = "lat"
            else:
                key = "yaw"
            self.mapper.state[key] = float(clamp(value, -1.0, 1.0))
        elif target == "crowd":
            self.mapper.state["crowd"] = float(clamp(value, 0.0, 1.0))
        elif target == "consent":
            self.mapper.state["consent"] = 1 if value >= 0.5 else 0
