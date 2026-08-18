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

from .backends import TrainerBackend
from .intent import ControlIntent
from .safety import SafetyEnvelope, SafetyState


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_AIRCRAFT_PROFILE = REPO_ROOT / "config/aircraft/ez_pilot_pro.yaml"
DEFAULT_MAPPING = REPO_ROOT / "config/mapping.yaml"


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

    def snapshot(self) -> Optional[ControlIntent]:
        with self._lock:
            timestamps = tuple(self._timestamps.values())
            if any(value is None for value in timestamps):
                return None
            values = dict(self._values)
        return ControlIntent(
            roll=float(values["roll"]),
            pitch=0.0,
            yaw=float(values["yaw"]),
            throttle=0.0,
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


def _start_osc_server(server) -> threading.Thread:
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return thread


def _drain_heartbeats(serial_port, backend: TrainerBackend) -> int:
    accepted = 0
    for _index in range(64):
        line = serial_port.readline()
        if not line:
            break
        if backend.observe_line(line):
            accepted += 1
    return accepted


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
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING)
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

    envelope = SafetyEnvelope.from_path(args.aircraft_profile)
    if envelope.profile is None:
        print(
            f"Safety profile rejected: {envelope.profile_error}",
            file=sys.stderr,
        )
        return 2
    try:
        routes = _load_osc_routes(args.mapping)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"Mapping rejected: {exc}", file=sys.stderr)
        return 2

    if bind_host not in {"127.0.0.1", "::1", "localhost"}:
        print(
            "WARNING: trainer OSC is exposed beyond loopback; use only on a "
            "physically isolated, trusted control network.",
            file=sys.stderr,
        )

    serial_port = serial.Serial(args.serial, args.baud, timeout=0)
    backend = TrainerBackend(serial_port)
    source = LiveOscIntentSource()
    osc_dispatcher = dispatcher.Dispatcher()
    osc_dispatcher.map(routes["lateral"], source.on_lateral)
    osc_dispatcher.map(routes["yaw"], source.on_yaw)
    osc_dispatcher.map(routes["crowd"], source.on_crowd)
    osc_dispatcher.map(routes["consent"], source.on_consent)
    server = osc_server.ThreadingOSCUDPServer(
        (bind_host, args.osc_port), osc_dispatcher
    )
    osc_thread = _start_osc_server(server)

    print(f"Trainer OSC listening on {server.server_address}")
    print(f"Trainer bridge serial open: {args.serial} @ {args.baud}")
    print(
        "Operator software gate: "
        + ("ENABLED" if args.operator_enable else "OFF (neutral only)")
    )

    period = 1.0 / args.hz
    frames = 0
    try:
        while args.max_frames == 0 or frames < args.max_frames:
            started = time.monotonic()
            _drain_heartbeats(serial_port, backend)
            state = envelope.evaluate(
                source.snapshot(),
                now=started,
                operator_enabled=args.operator_enable,
                bridge_heartbeat_timestamp=backend.last_heartbeat_timestamp,
            )
            backend.send(state)
            frames += 1
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("Trainer runtime interrupted; sending neutral output.")
    finally:
        profile_id = envelope.profile.profile_id
        try:
            backend.send(SafetyState.neutral("runtime_shutdown", profile_id))
        finally:
            server.shutdown()
            server.server_close()
            osc_thread.join(timeout=2)
            backend.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
