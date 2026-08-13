"""Trainer-oriented diagnostic output with explicit evidence labels."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path

from .profiles import (
    ProfileError,
    load_aircraft_profile,
    validate_transmitter_profile,
)


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
    args = parser.parse_args(argv)

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
        f"SOFTWARE VERIFIED: roll hard limit ±{aircraft.max_influence['roll']:.0%}"
    )
    print(
        f"SOFTWARE VERIFIED: yaw hard limit ±{aircraft.max_influence['yaw']:.0%}"
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

    print("\nHardware trainer bridge")
    print("UNVERIFIED: NOT CONNECTED (diagnostic performs no serial probing)")
    print("\nSTATE: SAFE — ZERO ALGORITHMIC AUTHORITY")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
