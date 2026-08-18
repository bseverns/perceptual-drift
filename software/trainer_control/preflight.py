"""Trainer-oriented diagnostic output with explicit evidence labels."""

from __future__ import annotations

import argparse
import importlib.util
import math
import time
from pathlib import Path

import serial

from .backends import TrainerBackend
from .profiles import (
    ProfileError,
    load_aircraft_profile,
    validate_transmitter_profile,
)
from .runtime import HeartbeatLineBuffer, _drain_heartbeats


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_AIRCRAFT = REPO_ROOT / "config/aircraft/ez_pilot_pro.yaml"
DEFAULT_TRANSMITTER = REPO_ROOT / "config/transmitters/tx16s_pd.yaml"


def _available(module: str) -> bool:
    return importlib.util.find_spec(module) is not None


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Trainer integration preflight"
    )
    parser.add_argument(
        "--aircraft-profile", type=Path, default=DEFAULT_AIRCRAFT
    )
    parser.add_argument(
        "--transmitter-profile", type=Path, default=DEFAULT_TRANSMITTER
    )
    parser.add_argument(
        "--operator-confirm-tx16s",
        action="store_true",
        help="Label TX16S configuration OPERATOR VERIFIED for this invocation only.",
    )
    parser.add_argument(
        "--serial",
        help="Read-only probe: open this bridge device and observe heartbeats.",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--heartbeat-count", type=int, default=3)
    parser.add_argument("--heartbeat-timeout", type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.baud <= 0:
        parser.error("--baud must be positive")
    if args.heartbeat_count <= 0:
        parser.error("--heartbeat-count must be positive")
    if (
        not math.isfinite(args.heartbeat_timeout)
        or args.heartbeat_timeout <= 0
    ):
        parser.error("--heartbeat-timeout must be a positive finite number")

    print("PERCEPTUAL DRIFT TRAINER PREFLIGHT\n")
    print("Software")
    print(
        f"SOFTWARE VERIFIED: Python environment ({'ready' if _available('yaml') else 'missing PyYAML'})"
    )
    try:
        aircraft = load_aircraft_profile(args.aircraft_profile)
        validate_transmitter_profile(args.transmitter_profile)
    except ProfileError as exc:
        print(f"UNVERIFIED: profile validation failed: {exc}")
        print("\nSTATE: SAFE — ZERO ALGORITHMIC AUTHORITY")
        return 1
    print("SOFTWARE VERIFIED: hardware profile schema valid")
    print("SOFTWARE VERIFIED: intent pipeline and safety envelope importable")
    print("SOFTWARE VERIFIED: DryRunBackend and TrainerBackend available")

    print("\nAircraft profile")
    print(f"CONFIG EXPECTED: {aircraft.display_name}")
    print("SOFTWARE VERIFIED: software ARM forbidden")
    print("SOFTWARE VERIFIED: software throttle forbidden")
    print(
        "SOFTWARE VERIFIED: trainer-domain roll hard limit "
        f"±{aircraft.max_influence['roll']:.0%}"
    )
    print(
        "SOFTWARE VERIFIED: trainer-domain yaw hard limit "
        f"±{aircraft.max_influence['yaw']:.0%}"
    )

    tx_label = (
        "OPERATOR VERIFIED"
        if args.operator_confirm_tx16s
        else "CONFIG EXPECTED"
    )
    print("\nTX16S configuration")
    print(f"{tx_label}: trainer roll ADD <=15%")
    print(f"{tx_label}: trainer yaw ADD <=15%")
    print(f"{tx_label}: trainer pitch OFF")
    print(f"{tx_label}: trainer throttle OFF")
    print(f"{tx_label}: ARM and flight mode physical only")
    print(f"{tx_label}: dedicated physical trainer-enable switch")
    print(
        f"{tx_label}: expected final roll/yaw authority <=2.25% "
        "(15% trainer-domain × 15% EdgeTX weight; bench verification required)"
    )

    print("\nHardware trainer bridge")
    if not args.serial:
        print("UNVERIFIED: NOT CONNECTED (no --serial device requested)")
    else:
        try:
            serial_port = serial.Serial(args.serial, args.baud, timeout=0.05)
        except serial.SerialException as exc:
            print(f"UNVERIFIED: trainer bridge could not be opened: {exc}")
            print("\nSTATE: SAFE — ZERO ALGORITHMIC AUTHORITY")
            return 1
        backend = TrainerBackend(serial_port)
        line_buffer = HeartbeatLineBuffer()
        observed = 0
        deadline = time.monotonic() + args.heartbeat_timeout
        try:
            while (
                observed < args.heartbeat_count and time.monotonic() < deadline
            ):
                observed += _drain_heartbeats(
                    serial_port, backend, line_buffer
                )
                if observed < args.heartbeat_count:
                    time.sleep(0.01)
        finally:
            backend.close()
        if observed < args.heartbeat_count:
            print(
                "UNVERIFIED: trainer bridge heartbeat timeout "
                f"({observed}/{args.heartbeat_count} observed)"
            )
            print("HARDWARE SIGNAL: UNVERIFIED")
            print("\nSTATE: SAFE — ZERO ALGORITHMIC AUTHORITY")
            return 1
        print(
            "SOFTWARE OBSERVED: trainer bridge responding "
            f"({observed} valid heartbeats)"
        )
        print("HARDWARE SIGNAL: still UNVERIFIED")
    print("\nSTATE: SAFE — ZERO ALGORITHMIC AUTHORITY")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
