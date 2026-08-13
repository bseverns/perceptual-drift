"""Safe-by-default trainer-path dry rehearsal CLI."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from .backends import DryRunBackend
from .intent import ControlIntent
from .pipeline import TrainerControlPipeline
from .safety import SafetyEnvelope


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_AIRCRAFT_PROFILE = REPO_ROOT / "config/aircraft/ez_pilot_pro.yaml"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Exercise Intent -> SafetyEnvelope -> DryRunBackend."
    )
    parser.add_argument(
        "--aircraft-profile",
        type=Path,
        default=DEFAULT_AIRCRAFT_PROFILE,
    )
    parser.add_argument("--frames", type=int, default=3)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--crowd", type=float, default=0.0)
    parser.add_argument("--consent", action="store_true")
    parser.add_argument("--operator-enable", action="store_true")
    parser.add_argument(
        "--simulate-bridge-heartbeat",
        action="store_true",
        help="Software-only heartbeat simulation; never claims hardware presence.",
    )
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.frames < 1:
        raise SystemExit("--frames must be at least 1")
    envelope = SafetyEnvelope.from_path(args.aircraft_profile)
    backend = DryRunBackend()
    pipeline = TrainerControlPipeline(envelope, backend)
    print("PERCEPTUAL DRIFT TRAINER DRY RUN")
    print(f"profile: {args.aircraft_profile}")
    print(
        "backend: DryRunBackend (SIMULATED; no serial, RF, or EdgeTX changes)"
    )
    for _index in range(args.frames):
        now = time.monotonic()
        intent = ControlIntent(
            roll=args.roll,
            pitch=args.pitch,
            yaw=args.yaw,
            throttle=0.0,
            crowd=args.crowd,
            consent=args.consent,
            timestamp=now,
        )
        heartbeat = now if args.simulate_bridge_heartbeat else None
        pipeline.process(
            intent,
            now=now,
            operator_enabled=args.operator_enable,
            bridge_heartbeat_timestamp=heartbeat,
        )
    print(
        "STATE: {}".format(
            "BOUNDED SIMULATION"
            if backend.last_state.active
            else "SAFE — ZERO ALGORITHMIC AUTHORITY"
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
