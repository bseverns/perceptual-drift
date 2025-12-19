#!/usr/bin/env python3
"""Poll ground-station telemetry for OBS overlays.

The goal: keep a tiny daemon that turns a JSON endpoint into two files
OBS can ingest without plugins:

* `telemetry_overlay.txt`  — Text source reads this and refreshes at 2–5 Hz.
* `telemetry_overlay.json` — Optional Browser/Text+GDI references the same
  content if you want richer styling.

Run it next to OBS (usually the laptop also running `software/video-pipeline`)
so capture + overlay latency stay in lockstep.
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.error import URLError
from urllib.request import urlopen


@dataclass
class TelemetrySnapshot:
    """Normalized telemetry fields we care about on the OBS side."""

    voltage: Optional[float]
    rssi: Optional[float]
    source_url: str
    fetched_at: datetime
    raw_payload: Dict[str, Any]
    error: Optional[str] = None

    @property
    def display_line(self) -> str:
        """Human-friendly status line for OBS Text sources."""

        volts = "--" if self.voltage is None else f"{self.voltage:.2f} V"
        rssi = "--" if self.rssi is None else f"{self.rssi:.0f}%"
        badge = "OK" if self.error is None else "DEGRADED"
        return f"BAT {volts}  |  RSSI {rssi}  |  {badge}"

    def as_json(self) -> Dict[str, Any]:
        return {
            "voltage": self.voltage,
            "rssi": self.rssi,
            "source_url": self.source_url,
            "fetched_at": self.fetched_at.isoformat(),
            "display": self.display_line,
            "error": self.error,
            "raw": self.raw_payload,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--url",
        default="http://127.0.0.1:9750/telemetry",
        help=(
            "HTTP(s) endpoint that returns JSON with battery + RSSI. "
            "Supports keys like voltage/vbat and rssi/link_quality."
        ),
    )
    parser.add_argument(
        "--out-dir",
        default="runtime/obs_overlay",
        help="Where to write telemetry_overlay.(txt|json).",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=2.0,
        help="Update rate in Hertz (2.0 = refresh every 0.5 s).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="HTTP timeout in seconds."
    )
    return parser.parse_args()


def normalize_metrics(payload: Dict[str, Any], source_url: str) -> TelemetrySnapshot:
    """Extract voltage + RSSI from common field names.

    We accept a handful of aliases so the same script can watch Betaflight
    OSD relays, Crazyflie ROS shims, or ad-hoc ground-station JSON dumps.
    """

    def pick(keys):
        for key in keys:
            if key in payload:
                return payload[key]
        return None

    raw_voltage = pick(["voltage", "vbat", "battery", "battery_voltage"])
    raw_rssi = pick(["rssi", "link_quality", "linkQuality", "lq"])

    voltage = float(raw_voltage) if raw_voltage is not None else None
    rssi = float(raw_rssi) if raw_rssi is not None else None

    return TelemetrySnapshot(
        voltage=voltage,
        rssi=rssi,
        source_url=source_url,
        fetched_at=datetime.now(timezone.utc),
        raw_payload=payload,
    )


def fetch_snapshot(url: str, timeout: float) -> TelemetrySnapshot:
    try:
        with urlopen(url, timeout=timeout) as resp:
            body = resp.read()
            payload = json.loads(body)
        snapshot = normalize_metrics(payload, url)
    except (URLError, OSError) as exc:
        snapshot = TelemetrySnapshot(
            voltage=None,
            rssi=None,
            source_url=url,
            fetched_at=datetime.now(timezone.utc),
            raw_payload={},
            error=f"network error: {exc}",
        )
    except json.JSONDecodeError as exc:
        snapshot = TelemetrySnapshot(
            voltage=None,
            rssi=None,
            source_url=url,
            fetched_at=datetime.now(timezone.utc),
            raw_payload={},
            error=f"bad JSON: {exc}",
        )
    return snapshot


def write_outputs(snapshot: TelemetrySnapshot, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    text_path = out_dir / "telemetry_overlay.txt"
    json_path = out_dir / "telemetry_overlay.json"

    text_path.write_text(snapshot.display_line + "\n", encoding="utf-8")
    json_path.write_text(json.dumps(snapshot.as_json(), indent=2), encoding="utf-8")



def main() -> None:
    args = parse_args()
    poll_interval = 1.0 / max(args.hz, 0.1)
    out_dir = Path(args.out_dir)

    print(
        f"Starting OBS telemetry overlay poller at {args.hz:.2f} Hz → {out_dir} "
        f"(source: {args.url})"
    )

    while True:
        snapshot = fetch_snapshot(args.url, args.timeout)
        write_outputs(snapshot, out_dir)
        if snapshot.error:
            print(f"WARN: {snapshot.error} @ {snapshot.fetched_at.isoformat()}")
        time.sleep(poll_interval)


if __name__ == "__main__":
    main()
