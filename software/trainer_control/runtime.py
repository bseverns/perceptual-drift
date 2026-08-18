"""Live OSC-to-wired-trainer runtime with a fail-closed safety envelope."""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Dict, Iterable, Mapping, Optional

import serial
import yaml
from pythonosc import dispatcher, osc_server

from .audit import AuditLogger
from .backends import TrainerBackend
from .mapping import DEFAULT_RECIPES_DIR, MappingController, TrackerSignals
from .profiles import ProfileError, validate_transmitter_profile
from .safety import SafetyEnvelope, SafetyState


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_AIRCRAFT_PROFILE = REPO_ROOT / "config/aircraft/ez_pilot_pro.yaml"
DEFAULT_MAPPING = REPO_ROOT / "config/mapping.yaml"
DEFAULT_TRANSMITTER_PROFILE = REPO_ROOT / "config/transmitters/tx16s_pd.yaml"


class LiveOscIntentSource:
    """Collect complete, independently fresh trainer intent from OSC."""

    REQUIRED_FRESH_AXES = ("roll", "yaw", "consent")

    def __init__(self, *, clock: Callable[[], float] = time.monotonic):
        self.clock = clock
        self._lock = threading.Lock()
        self._values = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "throttle": 0.0,
            "crowd": 0.0,
            "consent": False,
        }
        self._timestamps: Dict[str, Optional[float]] = {
            key: None for key in self.REQUIRED_FRESH_AXES
        }

    @staticmethod
    def _finite_first(values: Iterable[object]) -> Optional[float]:
        values = tuple(values)
        if not values:
            return None
        try:
            value = float(values[0])
        except (TypeError, ValueError, OverflowError):
            return None
        return value if math.isfinite(value) else None

    def _set_axis(
        self, key: str, values: Iterable[object], low: float, high: float
    ) -> bool:
        value = self._finite_first(values)
        if value is None:
            return False
        value = max(low, min(high, value))
        with self._lock:
            self._values[key] = value
            if key in self._timestamps:
                self._timestamps[key] = self.clock()
        return True

    def on_lateral(self, _address: str, *values: object) -> None:
        self._set_axis("roll", values, -1.0, 1.0)

    def on_yaw(self, _address: str, *values: object) -> None:
        self._set_axis("yaw", values, -1.0, 1.0)

    def on_crowd(self, _address: str, *values: object) -> None:
        self._set_axis("crowd", values, 0.0, 1.0)

    def on_consent(self, _address: str, *values: object) -> None:
        value = self._finite_first(values)
        if value is None:
            return
        with self._lock:
            self._values["consent"] = value >= 0.5
            self._timestamps["consent"] = self.clock()

    def snapshot(self) -> Optional[TrackerSignals]:
        with self._lock:
            timestamps = tuple(self._timestamps.values())
            if any(value is None for value in timestamps):
                return None
            values = dict(self._values)
        return TrackerSignals(
            lateral=float(values["roll"]),
            yaw=float(values["yaw"]),
            crowd=float(values["crowd"]),
            consent=bool(values["consent"]),
            timestamp=min(float(value) for value in timestamps),
        )


def _load_osc_routes(path: Path) -> Mapping[str, str]:
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    try:
        address_space = loaded["osc"]["address_space"]
        routes = {
            key: address_space[key]
            for key in ("lateral", "yaw", "crowd", "consent")
        }
    except (KeyError, TypeError) as exc:
        raise ValueError(f"mapping lacks trainer OSC routes: {path}") from exc
    if not all(isinstance(value, str) and value for value in routes.values()):
        raise ValueError(f"mapping has invalid trainer OSC routes: {path}")
    return routes


def _is_safe_recipe_name(value: str) -> bool:
    return bool(value) and all(
        character.isalnum() or character in {"-", "_"} for character in value
    )


def _start_osc_server(server) -> threading.Thread:
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return thread


class HeartbeatLineBuffer:
    """Retain split nonblocking serial lines until a newline arrives."""

    def __init__(self, max_buffer_bytes: int = 4096) -> None:
        self.buffer = bytearray()
        self.max_buffer_bytes = max_buffer_bytes

    def feed(self, chunk: bytes, backend: TrainerBackend) -> int:
        if not isinstance(chunk, bytes):
            return 0
        self.buffer.extend(chunk)
        if len(self.buffer) > self.max_buffer_bytes:
            del self.buffer[: -self.max_buffer_bytes]
        accepted = 0
        while b"\n" in self.buffer:
            line, _, remaining = self.buffer.partition(b"\n")
            self.buffer = bytearray(remaining)
            if backend.observe_line(bytes(line) + b"\n"):
                accepted += 1
        return accepted


def _drain_heartbeats(
    serial_port,
    backend: TrainerBackend,
    line_buffer: Optional[HeartbeatLineBuffer] = None,
) -> int:
    line_buffer = line_buffer or HeartbeatLineBuffer()
    accepted = 0
    for _index in range(64):
        chunk = serial_port.readline()
        if not chunk:
            break
        accepted += line_buffer.feed(chunk, backend)
    return accepted


class StateTransitionReporter:
    """Report effective trainer state only when it changes."""

    def __init__(
        self, audit: AuditLogger, output: Callable[[str], None] = print
    ):
        self.audit = audit
        self.output = output
        self._last_key = None

    def report(self, state: SafetyState) -> bool:
        key = (state.active, state.neutral_reason)
        if key == self._last_key:
            return False
        self._last_key = key
        if state.active:
            message = "TRAINER STATE → ACTIVE"
            status = "success"
        else:
            message = f"TRAINER STATE → NEUTRAL: {state.neutral_reason}"
            status = "warning"
        self.output(message)
        self.audit.write(
            "trainer_state_transition",
            status=status,
            message=message,
            details={
                "active": state.active,
                "reason": state.neutral_reason,
                "profile_id": state.profile_id,
            },
        )
        return True


def _evaluate_frame(
    *,
    source: LiveOscIntentSource,
    mapper: MappingController,
    envelope: SafetyEnvelope,
    backend: TrainerBackend,
    reporter: StateTransitionReporter,
    now: float,
    operator_enabled: bool,
) -> SafetyState:
    signals = source.snapshot()
    intent = mapper.map(signals) if signals is not None else None
    state = envelope.evaluate(
        intent,
        now=now,
        operator_enabled=operator_enabled,
        bridge_heartbeat_timestamp=backend.last_heartbeat_timestamp,
    )
    effective = backend.send(state)
    reporter.report(effective)
    return effective


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the live fail-closed wired trainer control path."
    )
    parser.add_argument("--serial", required=True, help="Trainer bridge port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--osc-port", type=int, default=9000)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument(
        "--aircraft-profile", type=Path, default=DEFAULT_AIRCRAFT_PROFILE
    )
    parser.add_argument(
        "--transmitter-profile",
        type=Path,
        default=DEFAULT_TRANSMITTER_PROFILE,
    )
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING)
    parser.add_argument(
        "--recipe", help="Initial artistic recipe name or path"
    )
    parser.add_argument(
        "--recipes-dir", type=Path, default=DEFAULT_RECIPES_DIR
    )
    parser.add_argument(
        "--recipe-route",
        default="/pd/patch",
        help="OSC route for validated live recipe selection",
    )
    parser.add_argument(
        "--operator-enable",
        action="store_true",
        help=(
            "Permit bounded output when consent/input/heartbeat are valid. "
            "The physical TX trainer-enable switch remains authoritative."
        ),
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after N frames (0 runs until interrupted).",
    )
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 1 <= args.baud:
        parser.error("--baud must be positive")
    if not 1 <= args.osc_port <= 65535:
        parser.error("--osc-port must be in 1..65535")
    if not math.isfinite(args.hz) or args.hz <= 0:
        parser.error("--hz must be a positive finite number")
    if args.max_frames < 0:
        parser.error("--max-frames must be zero or positive")
    bind_host = args.bind.strip()
    if not bind_host:
        parser.error("--bind must not be empty")
    if not args.recipe_route.startswith("/"):
        parser.error("--recipe-route must be an OSC path beginning with '/'")

    audit = AuditLogger()
    envelope = SafetyEnvelope.from_path(args.aircraft_profile)
    if envelope.profile is None:
        message = f"Safety profile rejected: {envelope.profile_error}"
        print(message, file=sys.stderr)
        audit.write(
            "trainer_startup_rejected",
            status="error",
            message=message,
            details={"reason": "invalid_aircraft_profile"},
        )
        return 2
    try:
        validate_transmitter_profile(args.transmitter_profile)
    except (ProfileError, OSError) as exc:
        message = f"Transmitter profile rejected: {exc}"
        print(message, file=sys.stderr)
        audit.write(
            "trainer_startup_rejected",
            status="error",
            message=message,
            details={"reason": "invalid_transmitter_profile"},
        )
        return 2
    try:
        routes = _load_osc_routes(args.mapping)
        intent_mapper = MappingController(
            args.mapping,
            recipes_dir=args.recipes_dir,
            recipe=args.recipe,
        )
    except (OSError, ValueError, yaml.YAMLError) as exc:
        message = f"Mapping rejected: {exc}"
        print(message, file=sys.stderr)
        audit.write(
            "trainer_startup_rejected",
            status="error",
            message=message,
            details={"reason": "invalid_mapping"},
        )
        return 2

    if bind_host not in {"127.0.0.1", "::1", "localhost"}:
        print(
            "WARNING: trainer OSC is exposed beyond loopback; use only on a "
            "physically isolated, trusted control network.",
            file=sys.stderr,
        )

    try:
        serial_port = serial.Serial(args.serial, args.baud, timeout=0)
    except serial.SerialException as exc:
        print(f"Trainer bridge serial rejected: {exc}", file=sys.stderr)
        audit.write(
            "trainer_serial_open_failed",
            status="error",
            message=str(exc),
            details={"serial": args.serial, "baud": args.baud},
        )
        return 2
    backend = TrainerBackend(serial_port)
    source = LiveOscIntentSource()
    reporter = StateTransitionReporter(audit)
    heartbeat_buffer = HeartbeatLineBuffer()
    osc_dispatcher = dispatcher.Dispatcher()
    osc_dispatcher.map(routes["lateral"], source.on_lateral)
    osc_dispatcher.map(routes["yaw"], source.on_yaw)
    osc_dispatcher.map(routes["crowd"], source.on_crowd)
    osc_dispatcher.map(routes["consent"], source.on_consent)

    def select_recipe(_address: str, *values: object) -> None:
        if not values or not isinstance(values[0], str):
            audit.write(
                "trainer_recipe_rejected",
                status="warning",
                message="Recipe selection requires a recipe name string.",
            )
            return
        requested = values[0].strip()
        if not _is_safe_recipe_name(requested):
            audit.write(
                "trainer_recipe_rejected",
                status="warning",
                message="Recipe selection rejected an invalid recipe name.",
            )
            return
        try:
            active = intent_mapper.select(requested)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            message = f"TRAINER RECIPE REJECTED: {requested}: {exc}"
            print(message, file=sys.stderr)
            audit.write(
                "trainer_recipe_rejected",
                status="warning",
                message=message,
                details={"requested": requested},
            )
            return
        message = f"TRAINER RECIPE → {active}"
        print(message)
        audit.write(
            "trainer_recipe_selected",
            status="success",
            message=message,
            details={"recipe": active},
        )

    osc_dispatcher.map(args.recipe_route, select_recipe)
    server = osc_server.ThreadingOSCUDPServer(
        (bind_host, args.osc_port), osc_dispatcher
    )
    osc_thread = _start_osc_server(server)

    print(f"Trainer OSC listening on {server.server_address}")
    print(f"Trainer bridge serial open: {args.serial} @ {args.baud}")
    print(f"Trainer artistic mapping: {intent_mapper.active_recipe}")
    print(
        "Operator software gate: "
        + ("ENABLED" if args.operator_enable else "OFF (neutral only)")
    )

    period = 1.0 / args.hz
    frames = 0
    exit_code = 0
    try:
        while args.max_frames == 0 or frames < args.max_frames:
            started = time.monotonic()
            _drain_heartbeats(serial_port, backend, heartbeat_buffer)
            _evaluate_frame(
                source=source,
                mapper=intent_mapper,
                envelope=envelope,
                backend=backend,
                reporter=reporter,
                now=started,
                operator_enabled=args.operator_enable,
            )
            frames += 1
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("Trainer runtime interrupted; sending neutral output.")
    except (OSError, serial.SerialException) as exc:
        exit_code = 1
        reporter.report(
            SafetyState.neutral(
                "backend_rejection", envelope.profile.profile_id
            )
        )
        audit.write(
            "trainer_backend_error",
            status="error",
            message=str(exc),
        )
    finally:
        profile_id = envelope.profile.profile_id
        try:
            try:
                shutdown_state = backend.send(
                    SafetyState.neutral("runtime_shutdown", profile_id)
                )
                reporter.report(shutdown_state)
            except (OSError, serial.SerialException) as exc:
                audit.write(
                    "trainer_shutdown_neutral_failed",
                    status="error",
                    message=str(exc),
                )
        finally:
            server.shutdown()
            server.server_close()
            osc_thread.join(timeout=2)
            backend.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
